// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "wall_follower_msgs/msg/scan.hpp"

#include <string>
#include <limits>
#include <vector>

constexpr double WHEEL_OFFSET{ 0.064 };

constexpr double DEG2RAD{ M_PI / 180.0 };
constexpr double RAD2DEG{ 180.0 / M_PI };

enum Angles : unsigned int { CENTER, LEFT, RIGHT };

constexpr double LINEAR_VELOCITY{ 0.1 };
constexpr double ANGULAR_VELOCITY{ 0.3 };

constexpr double START_RANGE_TOLERANCE{ 0.005 };

enum TB3_States : unsigned int {
  GET_TB3_DIRECTION,
  TB3_DRIVE_FORWARD,
  TB3_DRIVE_FORWARD_SL,
  TB3_DRIVE_FORWARD_SR,
  TB3_RIGHT_TURN,
  TB3_LEFT_TURN,
  TB3_DRIVE_STOP
};

class WallFollower : public rclcpp::Node {
public:
  WallFollower();
  ~WallFollower();

  // helper functions
  bool inStartRange(double x, double y);
  void driveRadius(double velocity, double radius);

protected:
  // Variables
  double robot_pose_{};
  double prev_robot_pose_{};
  std::vector<double> scan_data_;
  bool started_{};
  bool off_start_range_{};
  double startx_{};
  double starty_{};

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<wall_follower_msgs::msg::Scan>::SharedPtr wf_scan_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_