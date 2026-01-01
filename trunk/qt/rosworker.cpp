#include "rosworker.h"

#include <QThread>
#include <cmath>
#include <future>


#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

geometry_msgs::msg::Quaternion RosWorker::yawToQuat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

void RosWorker::requestNavigateTo(double x, double y, double yaw_rad)
{
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_req_.pending = true;
    goal_req_.x = x;
    goal_req_.y = y;
    goal_req_.yaw = yaw_rad;
}

void RosWorker::requestHeatmap(const QString& session_id,
                               const QString& ssid,
                               bool thr_enable,
                               int thr_rssi,
                               uint32_t limit,
                               uint32_t offset)
{
    std::lock_guard<std::mutex> lk(heat_mtx_);
    heat_req_.pending    = true;
    heat_req_.session_id = session_id;
    heat_req_.ssid       = ssid;
    heat_req_.thr_enable = thr_enable;
    heat_req_.thr_rssi   = thr_rssi;
    heat_req_.limit      = limit;
    heat_req_.offset     = offset;
}

void RosWorker::requestStartSession()
{
    std::lock_guard<std::mutex> lk(sess_mtx_);
    sess_req_.start_pending = true;
}

void RosWorker::requestStopSession()
{
    std::lock_guard<std::mutex> lk(sess_mtx_);
    sess_req_.stop_pending = true;
}

void RosWorker::requestListSessions(uint32_t limit, uint32_t offset)
{
    std::lock_guard<std::mutex> lk(list_mtx_);
    list_req_.pending = true;
    list_req_.limit = limit;
    list_req_.offset = offset;
}

void RosWorker::requestListSsid(const QString& session_id, uint32_t limit, uint32_t offset)
{
    std::lock_guard<std::mutex> lk(ssid_mtx_);
    ssid_req_.pending = true;
    ssid_req_.session_id = session_id;
    ssid_req_.limit = limit;
    ssid_req_.offset = offset;
}

void RosWorker::requestDeleteSession(const QString& session_id)
{
    std::lock_guard<std::mutex> lk(del_mtx_);
    del_req_.pending = true;
    del_req_.session_id = session_id;
}

void RosWorker::run()
{
    // rclcpp init
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp::Node>("qt_visualizer_node", opts);
    exec_.add_node(node_);

    // TF
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    pose_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            try {
                auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                const double yaw = tf2::getYaw(tf.transform.rotation);

                last_x_ = x; last_y_ = y; last_yaw_ = yaw;
                have_pose_ = true;

                have_pose_tf_ = true;
                last_tf_stamp_ = node_->now();

                emit robotPose(x, y, yaw);
            } catch (...) {
                // ignore
            }
        }
        );

    // /wifi/fused (UI only)
    emit statusChanged("Subscribing /wifi/fused ...");
    fused_sub_ = node_->create_subscription<wifi_interface::msg::WifiFused>(
        "/wifi/fused", rclcpp::QoS(10),
        [this](wifi_interface::msg::WifiFused::SharedPtr msg)
        {
            if (!msg) return;
            if (msg->ssid.empty()) return;

            const int rssi_i = (int)std::lround(msg->rssi);
            emit fusedSample(msg->x, msg->y, QString::fromStdString(msg->ssid), rssi_i);
        }
        );

    // /amcl_pose fallback
    emit statusChanged("Subscribing /amcl_pose (fallback when TF missing) ...");
    amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            if (!msg) return;

            // gating: ignore before stable
            const auto &C = msg->pose.covariance;
            const double var_x   = C[0];
            const double var_y   = C[7];
            const double var_yaw = C[35];
            const bool stable = (var_x < 0.25) && (var_y < 0.25) && (var_yaw < 0.5);
            if (!stable) return;

            bool tf_recent = false;
            if (have_pose_tf_) {
                const auto now = node_->now();
                const double dt = (now - last_tf_stamp_).seconds();
                tf_recent = (dt >= 0.0 && dt < 0.5);
            }
            if (tf_recent) return;

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

            last_x_ = x; last_y_ = y; last_yaw_ = yaw;
            have_pose_ = true;

            emit robotPose(x, y, yaw);
        }
        );

    // Services
    start_sess_client_ = node_->create_client<std_srvs::srv::Trigger>("/db/start_session");
    stop_sess_client_  = node_->create_client<std_srvs::srv::Trigger>("/db/stop_session");
    heatmap_client_    = node_->create_client<wifi_interface::srv::GetHeatmap>("/db/get_heatmap");

    list_sessions_client_ = node_->create_client<wifi_interface::srv::ListSessions>("/db/list_sessions");
    list_ssid_client_     = node_->create_client<wifi_interface::srv::ListSsids>("/db/list_ssids");
    delete_sess_client_   = node_->create_client<wifi_interface::srv::DeleteSession>("/db/delete_session");

    emit statusChanged("Service clients ready: "
                       "/db/start_session /db/stop_session /db/get_heatmap "
                       "/db/list_sessions /db/list_ssids /db/delete_session");

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    emit statusChanged("Waiting Nav2 action server: navigate_to_pose ...");
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
            emit statusChanged("Nav2 action server connected.");
            break;
        }
    }

    while (rclcpp::ok() && !isInterruptionRequested()) {
        exec_.spin_some();

        // =========================
        // (A) Session start/stop (Trigger)
        // =========================
        bool do_start=false, do_stop=false;
        {
            std::lock_guard<std::mutex> lk(sess_mtx_);
            do_start = sess_req_.start_pending;
            do_stop  = sess_req_.stop_pending;
            sess_req_.start_pending = false;
            sess_req_.stop_pending  = false;
        }

        if (do_start) {
            if (!start_sess_client_ || !start_sess_client_->service_is_ready()) {
                emit startSessionReply(false, "Service not ready: /db/start_session");
            } else {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                start_sess_client_->async_send_request(
                    req,
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
                        auto res = future.get();
                        if (!res) { emit startSessionReply(false, "Null response"); return; }
                        emit startSessionReply(res->success, QString::fromStdString(res->message));
                    }
                    );
            }
        }

        if (do_stop) {
            if (!stop_sess_client_ || !stop_sess_client_->service_is_ready()) {
                emit stopSessionReply(false, "Service not ready: /db/stop_session");
            } else {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                stop_sess_client_->async_send_request(
                    req,
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
                        auto res = future.get();
                        if (!res) { emit stopSessionReply(false, "Null response"); return; }
                        emit stopSessionReply(res->success, QString::fromStdString(res->message));
                    }
                    );
            }
        }

        // =========================
        // (B) ListSessions
        // =========================
        ListSessReq lreq;
        {
            std::lock_guard<std::mutex> lk(list_mtx_);
            if (list_req_.pending) {
                lreq = list_req_;
                list_req_.pending = false;
            } else lreq.pending = false;
        }

        if (lreq.pending) {
            if (!list_sessions_client_ || !list_sessions_client_->service_is_ready()) {
                emit listSessionsReply(false, "Service not ready: /db/list_sessions", {}, {}, {});
            } else {
                auto sreq = std::make_shared<wifi_interface::srv::ListSessions::Request>();
                sreq->limit  = lreq.limit;
                sreq->offset = lreq.offset;

                list_sessions_client_->async_send_request(
                    sreq,
                    [this](rclcpp::Client<wifi_interface::srv::ListSessions>::SharedFuture future)
                    {
                        auto res = future.get();
                        if (!res) { emit listSessionsReply(false, "Null response", {}, {}, {}); return; }

                        QList<QString> ids, started, ended;
                        const size_t n = res->session_ids.size();
                        ids.reserve((int)n); started.reserve((int)n); ended.reserve((int)n);

                        for (size_t i=0; i<n; ++i) {
                            ids.push_back(QString::fromStdString(res->session_ids[i]));
                            started.push_back(i < res->started_at.size() ? QString::fromStdString(res->started_at[i]) : "");
                            ended.push_back(i < res->ended_at.size() ? QString::fromStdString(res->ended_at[i]) : "");
                        }

                        emit listSessionsReply(res->ok, QString::fromStdString(res->message), ids, started, ended);
                    }
                    );

            }
        }

        // =========================
        // (C) ListSsid
        // =========================
        ListSsidReq ssreq;
        {
            std::lock_guard<std::mutex> lk(ssid_mtx_);
            if (ssid_req_.pending) {
                ssreq = ssid_req_;
                ssid_req_.pending = false;
            } else ssreq.pending = false;
        }

        if (ssreq.pending) {
            if (!list_ssid_client_ || !list_ssid_client_->service_is_ready()) {
                emit listSsidsReply(false, "Service not ready: /db/list_ssid", ssreq.session_id, {});
            } else {
                auto sreq = std::make_shared<wifi_interface::srv::ListSsids::Request>();
                sreq->session_id = ssreq.session_id.toStdString();
                sreq->limit      = ssreq.limit;
                sreq->offset     = ssreq.offset;

                list_ssid_client_->async_send_request(
                    sreq,
                    [this, ssreq](rclcpp::Client<wifi_interface::srv::ListSsids>::SharedFuture future)
                    {
                        auto res = future.get();
                        if (!res) { emit listSsidsReply(false, "Null response", ssreq.session_id, {}); return; }

                        // QStringList ssids;
                        QList<QString> ssids;
                        ssids.reserve((int)res->ssids.size());
                        for (auto &s : res->ssids) ssids.push_back(QString::fromStdString(s));

                        emit listSsidsReply(res->ok, QString::fromStdString(res->message), ssreq.session_id, ssids);
                    }
                    );
            }
        }

        // =========================
        // (D) DeleteSession
        // =========================
        DeleteReq dreq;
        {
            std::lock_guard<std::mutex> lk(del_mtx_);
            if (del_req_.pending) {
                dreq = del_req_;
                del_req_.pending = false;
            } else dreq.pending = false;
        }

        if (dreq.pending) {
            if (!delete_sess_client_ || !delete_sess_client_->service_is_ready()) {
                emit deleteSessionReply(false, "Service not ready: /db/delete_session", dreq.session_id);
            } else {
                auto sreq = std::make_shared<wifi_interface::srv::DeleteSession::Request>();
                sreq->session_id = dreq.session_id.toStdString();

                delete_sess_client_->async_send_request(
                    sreq,
                    [this, dreq](rclcpp::Client<wifi_interface::srv::DeleteSession>::SharedFuture future)
                    {
                        auto res = future.get();
                        if (!res) {
                            emit deleteSessionReply(false, "Null response", dreq.session_id);
                            return;
                        }
                        emit deleteSessionReply(res->ok, QString::fromStdString(res->message), dreq.session_id);
                    }
                    );

            }
        }

        // =========================
        // (E) GetHeatmap
        // =========================
        HeatReq hreq;
        {
            std::lock_guard<std::mutex> lk(heat_mtx_);
            if (heat_req_.pending) {
                hreq = heat_req_;
                heat_req_.pending = false;
            } else {
                hreq.pending = false;
            }
        }

        if (hreq.pending) {
            if (!heatmap_client_ || !heatmap_client_->service_is_ready()) {
                emit heatmapReplyArrived(false, "Service not ready: /db/get_heatmap",
                                         hreq.session_id, hreq.ssid,
                                         hreq.thr_enable, hreq.thr_rssi,
                                         {}, {}, {}, {}, {});
            } else {
                auto sreq = std::make_shared<wifi_interface::srv::GetHeatmap::Request>();
                sreq->session_id = hreq.session_id.toStdString();
                sreq->ssid       = hreq.ssid.toStdString();
                sreq->thr_enable = hreq.thr_enable;
                sreq->thr_rssi   = hreq.thr_rssi;
                sreq->limit      = hreq.limit;
                sreq->offset     = hreq.offset;

                heatmap_client_->async_send_request(
                    sreq,
                    [this, hreq](rclcpp::Client<wifi_interface::srv::GetHeatmap>::SharedFuture future)
                    {
                        auto res = future.get();
                        if (!res) {
                            emit heatmapReplyArrived(false, "Null response",
                                                     hreq.session_id, hreq.ssid,
                                                     hreq.thr_enable, hreq.thr_rssi,
                                                     {}, {}, {}, {}, {});
                            return;
                        }

                        QVector<double> xs, ys;
                        QVector<int> rssis;
                        QVector<QString> ssids, stamps;

                        const size_t n = res->xs.size();
                        xs.reserve((int)n);
                        ys.reserve((int)n);
                        rssis.reserve((int)n);
                        ssids.reserve((int)n);
                        stamps.reserve((int)n);

                        for (size_t i=0; i<n; ++i) {
                            xs.push_back(res->xs[i]);
                            ys.push_back(res->ys[i]);
                            rssis.push_back(res->rssis[i]);
                            if (i < res->ssids.size())  ssids.push_back(QString::fromStdString(res->ssids[i]));
                            else                        ssids.push_back("");
                            if (i < res->stamps.size()) stamps.push_back(QString::fromStdString(res->stamps[i]));
                            else                        stamps.push_back("");
                        }

                        emit heatmapReplyArrived(res->ok,
                                                 QString::fromStdString(res->message),
                                                 hreq.session_id, hreq.ssid,
                                                 hreq.thr_enable, hreq.thr_rssi,
                                                 xs, ys, rssis, ssids, stamps);
                    }
                    );
            }
        }

        // =========================
        // (F) Nav goal
        // =========================
        GoalReq greq;
        {
            std::lock_guard<std::mutex> lk(goal_mtx_);
            greq = goal_req_;
            goal_req_.pending = false;
        }

        if (greq.pending) {
            if (!nav_client_ || !nav_client_->action_server_is_ready()) {
                emit goalStatus("Nav2 not ready: action server not available.");
            } else {
                NavigateToPose::Goal goal_msg;
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = node_->now();
                goal_msg.pose.pose.position.x = greq.x;
                goal_msg.pose.pose.position.y = greq.y;
                goal_msg.pose.pose.orientation = yawToQuat(greq.yaw);

                emit goalStatus(QString("Sending goal: (%1, %2, yaw=%3)")
                                    .arg(greq.x,0,'f',2).arg(greq.y,0,'f',2).arg(greq.yaw,0,'f',2));

                rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

                opts.goal_response_callback =
                    [this](std::shared_ptr<GoalHandleNav> gh)
                {
                    if (!gh) { emit goalStatus("Goal rejected."); return; }
                    active_goal_ = gh;
                    emit goalStatus("Goal accepted.");
                };

                opts.result_callback =
                    [this](const GoalHandleNav::WrappedResult &result)
                {
                    switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED: emit goalStatus("Goal SUCCEEDED."); break;
                    case rclcpp_action::ResultCode::ABORTED:   emit goalStatus("Goal ABORTED.");   break;
                    case rclcpp_action::ResultCode::CANCELED:  emit goalStatus("Goal CANCELED.");  break;
                    default:                                   emit goalStatus("Goal result unknown."); break;
                    }
                    active_goal_.reset();
                };

                nav_client_->async_send_goal(goal_msg, opts);
            }
        }

        QThread::msleep(20);
    }

    exec_.remove_node(node_);
    node_.reset();
}
