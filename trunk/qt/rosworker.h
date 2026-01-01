#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <QStringList>
#include <QList>

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <QTimer>

#include "wifi_interface/msg/wifi_fused.hpp"
#include "wifi_interface/srv/get_heatmap.hpp"
#include "wifi_interface/srv/list_sessions.hpp"
#include "wifi_interface/srv/list_ssids.hpp"
#include "wifi_interface/srv/delete_session.hpp"


class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);

    // ---- Requests from UI thread ----
    void requestNavigateTo(double x, double y, double yaw_rad);

    void requestHeatmap(const QString& session_id,
                        const QString& ssid,
                        bool thr_enable,
                        int thr_rssi,
                        uint32_t limit,
                        uint32_t offset);

    void requestStartSession();
    void requestStopSession();

    void requestListSessions(uint32_t limit = 200, uint32_t offset = 0);
    void requestListSsid(const QString& session_id, uint32_t limit = 200, uint32_t offset = 0);
    void requestDeleteSession(const QString& session_id);

signals:
    // Status / debug
    void statusChanged(const QString& msg);

    // Live data
    void robotPose(double x, double y, double yaw);
    void fusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    // Session start/stop replies
    void startSessionReply(bool ok, const QString& message);
    void stopSessionReply(bool ok, const QString& message);

    // DB list replies
    void listSessionsReply(bool ok,
                           const QString& message,
                           const QList<QString>& session_ids,
                           const QList<QString>& started_at,
                           const QList<QString>& ended_at);

    void listSsidsReply(bool ok,
                        const QString& message,
                        const QString& session_id,
                        const QList<QString>& ssids);

    void deleteSessionReply(bool ok, const QString& message,
                            const QString& session_id);

    // Heatmap reply
    void heatmapReplyArrived(bool ok,
                             const QString& message,
                             const QString& session_id,
                             const QString& ssid,
                             bool thr_enable,
                             int thr_rssi,
                             const QVector<double>& xs,
                             const QVector<double>& ys,
                             const QVector<int>& rssis,
                             const QVector<QString>& ssids,
                             const QVector<QString>& stamps);

    // Nav status
    void goalStatus(const QString& msg);

protected:
    void run() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    geometry_msgs::msg::Quaternion yawToQuat(double yaw);

    // -----------------------------
    // Pending request structures
    // -----------------------------
    struct GoalReq {
        bool pending=false;
        double x=0.0, y=0.0, yaw=0.0;
    };

    struct HeatReq {
        bool pending=false;
        QString session_id;
        QString ssid;
        bool thr_enable=false;
        int thr_rssi=-70;
        uint32_t limit=200000;
        uint32_t offset=0;
    };

    struct SessReq {
        bool start_pending=false;
        bool stop_pending=false;
    };

    struct ListSessReq {
        bool pending=false;
        uint32_t limit=200;
        uint32_t offset=0;
    };

    struct ListSsidReq {
        bool pending=false;
        QString session_id;
        uint32_t limit=200;
        uint32_t offset=0;
    };

    struct DeleteReq {
        bool pending=false;
        QString session_id;
    };

    // -----------------------------
    // Mutex / request buffers
    // -----------------------------
    std::mutex goal_mtx_;
    std::mutex heat_mtx_;
    std::mutex sess_mtx_;
    std::mutex list_mtx_;
    std::mutex ssid_mtx_;
    std::mutex del_mtx_;

    GoalReq goal_req_;
    HeatReq heat_req_;
    SessReq sess_req_;
    ListSessReq list_req_;
    ListSsidReq ssid_req_;
    DeleteReq del_req_;

    // -----------------------------
    // ROS Node / Executor
    // -----------------------------
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Node::SharedPtr node_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    // Fallback pose
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    // Live fused
    rclcpp::Subscription<wifi_interface::msg::WifiFused>::SharedPtr fused_sub_;

    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_sess_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_sess_client_;
    rclcpp::Client<wifi_interface::srv::GetHeatmap>::SharedPtr heatmap_client_;

    rclcpp::Client<wifi_interface::srv::ListSessions>::SharedPtr list_sessions_client_;
    rclcpp::Client<wifi_interface::srv::ListSsids>::SharedPtr list_ssid_client_;
    rclcpp::Client<wifi_interface::srv::DeleteSession>::SharedPtr delete_sess_client_;

    // Nav2 action
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<GoalHandleNav> active_goal_;

    // Pose state
    bool have_pose_ = false;
    bool have_pose_tf_ = false;
    rclcpp::Time last_tf_stamp_;
    double last_x_=0, last_y_=0, last_yaw_=0;

};

#endif // ROSWORKER_H
