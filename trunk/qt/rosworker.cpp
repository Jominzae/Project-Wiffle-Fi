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

RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

void RosWorker::run()
{
    rclcpp::NodeOptions opts;
    // ✅ 실제 로봇이면 보통 sim_time 아님. (시뮬이면 true)
    // opts.append_parameter_override("use_sim_time", true);

    node_ = std::make_shared<rclcpp::Node>("wifi_qt_node", opts);
    exec_.add_node(node_);

    emit statusChanged("Subscribing /amcl_pose (MVP dummy RSSI) ...");

    // ✅ 더미 RSSI 모델 파라미터
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

            // ✅ Dummy RSSI
            const double d = std::hypot(x - apX, y - apY);
            float rssi = static_cast<float>(-30.0 - 20.0 * std::log10(d + 1.0));
            rssi += noise(gen);

            emit robotPose(x, y, yaw);         // 기존 표시용
            emit sample(x, y, yaw, rssi);      // ✅ 히트맵용
        }
        );

    while (rclcpp::ok() && !isInterruptionRequested()) {
        exec_.spin_some();
        QThread::msleep(20); // 50Hz spin, 콜백은 들어오는 대로
    }
}
