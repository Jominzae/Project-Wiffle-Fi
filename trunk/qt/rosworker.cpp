#include "rosworker.h"
#include <QDateTime>
#include <QThread>
#include <cstdio>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include "rosworker.h"
#include <cmath>
#include <random>

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

void RosWorker::requestNavigateTo(double x, double y, double yaw_rad)
{
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_x_ = x;
    goal_y_ = y;
    goal_yaw_ = yaw_rad;
    goal_pending_ = true;
}


void RosWorker::run()
{
    rclcpp::NodeOptions opts;
    // 실제 로봇이면 보통 sim_time 아님. (시뮬이면 true)
    // opts.append_parameter_override("use_sim_time", true);

    node_ = std::make_shared<rclcpp::Node>("wifi_qt_node", opts);
    exec_.add_node(node_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    pose_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            try {
                // 최신 TF (가장 최근 transform)
                auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                const double yaw = tf2::getYaw(tf.transform.rotation);

                last_x_ = x; last_y_ = y; last_yaw_ = yaw;
                have_pose_ = true;

                emit robotPose(x, y, yaw);

            } catch (...) {
            }
        }
        );


    emit statusChanged("Subscribing /amcl_pose (MVP dummy RSSI) ...");

    // 더미 RSSI 모델 파라미터
    const double apX = 0.0;   // map frame
    const double apY = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> noise(0.0f, 2.0f); // N(0,2dB)

    amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(10),
        [this, apX, apY, &gen, &noise](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            const double x = msg->pose.pose.position.x;
            const double y = msg->pose.pose.position.y;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
                );
            double roll=0, pitch=0, yaw=0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // AMCL covariance 기반 유효성 판단 (값이 크면 아직 수렴 전)
            // covariance 배열에서 x,y,yaw 분산(대각 성분) 사용
            const auto &C = msg->pose.covariance;
            const double var_x   = C[0];   // xx
            const double var_y   = C[7];   // yy
            const double var_yaw = C[35];  // yawyaw

            // 임계값은 환경마다 조절 가능 (처음엔 보수적으로)
            const bool stable = (var_x < 0.25) && (var_y < 0.25) && (var_yaw < 0.5);

            if (!stable) {
                // 수렴 전에는 UI에 (0,0)같은 값이 찍히지 않도록 emit 안 함
                return;
            }

            // Dummy RSSI
            const double d = std::hypot(x - apX, y - apY);
            float rssi = static_cast<float>(-30.0 - 20.0 * std::log10(d + 1.0));
            rssi += noise(gen);

            emit robotPose(x, y, yaw);
            emit sample(x, y, yaw, rssi);
        }
        );


    // nav2 액션 클라이언트 생성
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    emit statusChanged("Waiting Nav2 action server: navigate_to_pose ...");

    // 서버 대기
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
            emit statusChanged("Nav2 action server connected.");
            break;
        }
    }

    while (rclcpp::ok() && !isInterruptionRequested()) {
        exec_.spin_some();

        // pending goal 처리
        bool do_send = false;
        double gx=0, gy=0, gyaw=0;
        {
            std::lock_guard<std::mutex> lk(goal_mtx_);
            if (goal_pending_) {
                gx = goal_x_;
                gy = goal_y_;
                gyaw = goal_yaw_;
                goal_pending_ = false;
                do_send = true;
            }
        }

        if (do_send) {
            if (!nav_client_ || !nav_client_->action_server_is_ready()) {
                emit goalStatus("Nav2 not ready: action server not available.");
            } else {
                // 이전 goal 취소 - 최소 구현에서는 생략 가능
                // if (active_goal_) nav_client_->async_cancel_goal(active_goal_);

                NavigateToPose::Goal goal_msg;
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = node_->now();
                goal_msg.pose.pose.position.x = gx;
                goal_msg.pose.pose.position.y = gy;
                goal_msg.pose.pose.position.z = 0.0;
                goal_msg.pose.pose.orientation = yawToQuat(gyaw);

                emit goalStatus(QString("Sending goal: (%1, %2, yaw=%3)")
                                    .arg(gx,0,'f',2).arg(gy,0,'f',2).arg(gyaw,0,'f',2));

                rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

                opts.goal_response_callback =
                    [this](std::shared_ptr<GoalHandleNav> gh)
                {
                    if (!gh) {
                        emit goalStatus("Goal rejected.");
                        return;
                    }
                    active_goal_ = gh;
                    emit goalStatus("Goal accepted.");
                };

                opts.feedback_callback =
                    [this](std::shared_ptr<GoalHandleNav>,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
                {
                    (void)feedback;
                };

                opts.result_callback =
                    [this](const GoalHandleNav::WrappedResult &result)
                {
                    switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        emit goalStatus("Goal SUCCEEDED.");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        emit goalStatus("Goal ABORTED.");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        emit goalStatus("Goal CANCELED.");
                        break;
                    default:
                        emit goalStatus("Goal result unknown.");
                        break;
                    }
                    active_goal_.reset();
                };

                nav_client_->async_send_goal(goal_msg, opts);
            }
        }

        QThread::msleep(20);
    }
}
