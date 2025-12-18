#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <QString>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>



class RosWorker : public QThread
{
    Q_OBJECT
public:
    explicit RosWorker(QObject *parent = nullptr);

signals:
    void statusChanged(const QString &msg);
    void robotPose(double x, double y, double yaw);
    void sample(double x, double y, double yaw, float rssi);

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
};

#endif
