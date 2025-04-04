/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <Eigen/Dense>
#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdometryPublisher : public rclcpp::Node {
 public:
  // Constructor
  OdometryPublisher() : Node("odometry_publisher") {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/nanoauv/odometry", 10);

    // Get the value of the odom_rate parameter
    this->declare_parameter("odom_rate", rclcpp::PARAMETER_DOUBLE);
    rate_ = this->get_parameter("odom_rate").as_double();

    // Print the value of the odom_rate parameter
    RCLCPP_INFO(this->get_logger(), "Odometry rate: %f", rate_);

    // Create a timer to publish the odometry message at user-defined rate
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&OdometryPublisher::publishOdometry, this));

    tf2Broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    time_since_start_ = this->now().seconds();

    // timerTf2Broadcaster_ =
    //     this->create_wall_timer(std::chrono::milliseconds(100),
    //                             std::bind(&OdometryPublisher::publishTf, this));

    // // Create a timer to publish TF data at a fixed rate
    //     timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    //   publishTF();
  }

 private:
  // Publish odometry message
  void publishOdometry() {
    // Get current time in seconds
    double time_seconds = this->now().seconds() - time_since_start_;

    // Create odometry message
    nav_msgs::msg::Odometry odometryMsg;

    // Populate odometry header message
    odometryMsg.header.stamp = this->now();
    odometryMsg.header.frame_id = "world";
    odometryMsg.child_frame_id = "base_link";

    // Populate odometry pose message with sine/cosine signals
    odometryMsg.pose.pose.position.x =
        325 * std::sin(2 * M_PI * time_seconds * 0.0002);
    odometryMsg.pose.pose.position.y = 0.0;
    // 20.0 * std::cos(2 * M_PI * time_seconds * 0.02);
    odometryMsg.pose.pose.position.z = 0.0;
    // 5.0 * std::sin(2 * M_PI * time_seconds * 0.03);

    // Create Euler angles from sine/cosine signals
    double roll = 120 * M_PI / 180.0 * std::sin(2 * M_PI * time_seconds * 0.01);
    double pitch = 75 * M_PI / 180.0 * std::cos(2 * M_PI * time_seconds * 0.02);
    double yaw = 170 * M_PI / 180.0 * std::sin(2 * M_PI * time_seconds * 0.03);

    Eigen::Vector3d sensorRotationEulerRpy(roll, pitch, yaw);

    // Convert Euler angles to rotation matrix
    Eigen::Matrix3d sensorRotation;
    sensorRotation =
        Eigen::AngleAxisd(sensorRotationEulerRpy(0), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(sensorRotationEulerRpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(sensorRotationEulerRpy(2), Eigen::Vector3d::UnitX());

    // Convert rotation matrix to quaternion
    Eigen::Quaterniond sensorRotationQuat = Eigen::Quaterniond(sensorRotation);

    // Normalize quaternion
    sensorRotationQuat.normalize();

    // Fill quaternion
    odometryMsg.pose.pose.orientation.w = sensorRotationQuat.w();
    odometryMsg.pose.pose.orientation.x = sensorRotationQuat.x();
    odometryMsg.pose.pose.orientation.y = sensorRotationQuat.y();
    odometryMsg.pose.pose.orientation.z = sensorRotationQuat.z();

    // Populate odometry twist message with sine/cosine signals
    odometryMsg.twist.twist.linear.x =
        0.25 * std::cos(2 * M_PI * time_seconds * 0.01);
    odometryMsg.twist.twist.linear.y =
        0.01 * std::sin(2 * M_PI * time_seconds * 0.02);
    odometryMsg.twist.twist.linear.z =
        0.02 * std::cos(2 * M_PI * time_seconds * 0.03);

    odometryMsg.twist.twist.angular.x =
        1 * std::cos(2 * M_PI * time_seconds * 0.01);
    odometryMsg.twist.twist.angular.y =
        2 * std::sin(2 * M_PI * time_seconds * 0.02);
    odometryMsg.twist.twist.angular.z =
        0.5 * std::cos(2 * M_PI * time_seconds * 0.03);

    // Publish odometry message
    publisher_->publish(odometryMsg);

    geometry_msgs::msg::TransformStamped tfMsg;
    tfMsg.header.stamp = now();
    tfMsg.header.frame_id = "world";
    tfMsg.child_frame_id = "base_link";

    tfMsg.transform.translation.x = odometryMsg.pose.pose.position.x;
    tfMsg.transform.translation.y = odometryMsg.pose.pose.position.y;
    tfMsg.transform.translation.z = odometryMsg.pose.pose.position.z;

    tfMsg.transform.rotation.w = odometryMsg.pose.pose.orientation.w;
    tfMsg.transform.rotation.x = odometryMsg.pose.pose.orientation.x;
    tfMsg.transform.rotation.y = odometryMsg.pose.pose.orientation.y;
    tfMsg.transform.rotation.z = odometryMsg.pose.pose.orientation.z;

    tf2Broadcaster_->sendTransform(tfMsg);
  }
  // Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  // Timer to publish odometry message
  rclcpp::TimerBase::SharedPtr timer_;

  //   rclcpp::TimerBase::SharedPtr timerTf2Broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2Broadcaster_;

  // Rate of odometry message publication
  double rate_;

  // Time since start
  double time_since_start_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto odometry_publisher = std::make_shared<OdometryPublisher>();
  rclcpp::spin(odometry_publisher);
  rclcpp::shutdown();
  return 0;
}