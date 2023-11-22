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
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>
#include <numeric>
#include <set>

using namespace std::chrono_literals;

WallFollower::WallFollower() :
	Node("wall_follower_node"),
	scan_data_(3, 0.0),
	started_{false},
	off_start_range_{false}
{
	/************************************************************
	 ** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos{ rclcpp::QoS(rclcpp::KeepLast(10)) };

	// Initialise publishers
	cmd_vel_pub_ =
		this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
	wf_scan_pub_ =
		this->create_publisher<wall_follower_msgs::msg::Scan>("wall_follower_scan", rclcpp::SensorDataQoS());

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(),
		std::bind(&WallFollower::scan_callback, this, std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos,
		std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	 ** Initialise ROS timers
	************************************************************/
  	update_timer_ = this->create_wall_timer(
      	10ms, std::bind(&WallFollower::update_callback, this));

  	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower() {
  	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
						msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	if (!started_) {
		// record starting position
		startx_ = msg->pose.pose.position.x;
		starty_ = msg->pose.pose.position.y;
		started_ = true;
	}

  	if (
		inStartRange(msg->pose.pose.position.x, msg->pose.pose.position.y) &&
		off_start_range_
	) {
		update_cmd_vel(0.0, 0.0);
		// exports map to ~/colcon_ws
		system("ros2 run nav2_map_server map_saver_cli -f src/wall_follower/my_map");
		exit(0);
	}
}

// computes the smallest 8% of values from the data, then return their average
template <typename T>
T computeMin(const std::vector<T>& data) {
	if (data.size() == 0)
		return 0;
	std::set<T> s(data.begin(), data.end());
	int sumCountDown{
		static_cast<int>(0.08 * data.size()) == 0 ?
		1 : static_cast<int>(0.08 * data.size())
	};
	const int averageSize{sumCountDown};
	T sum{};
	for (auto it{s.begin()}; it != s.end(); ++it) {
		if (sumCountDown == 0) break;
		if (*it == 0) continue;
		sum += *it;
		sumCountDown--;
	}
	if (sumCountDown == averageSize)
		return static_cast<T>(0);
	return static_cast<T>(sum/averageSize);
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	// shift scanned LiDar data forward to center of 2 wheels
	auto calcDerivedDist = [&](const uint16_t& currAngle) {
		double rawDist{msg->ranges.at(currAngle)};
		if (rawDist < 0)
			rawDist = 0;
		double derivedDist{
			std::sqrt(
				std::pow(rawDist, 2.0) + std::pow(WHEEL_OFFSET, 2.0)
				- 2 * rawDist * WHEEL_OFFSET * cosf(M_PI - currAngle)
			)
		};
		return derivedDist;
	};

	std::pair<uint16_t, uint16_t> scan_angle[3] = {
		{0, 100}, {50, 40}, {310, 40}
	};

	for (int num = 0; num < 3; num++) {
		double min_angle{ scan_angle[num].first - (scan_angle[num].second / 2.0) };
		double max_angle{ scan_angle[num].first + (scan_angle[num].second / 2.0) };

		std::vector<double>raw_scan_data{};

		for (double angle { min_angle }; angle <= max_angle; ++angle) {
			double currAngle{angle};
			if (angle < 0) currAngle += 360.0;
			else if (angle >= 360.0) currAngle -= 360.0;
			if (std::isinf(msg->ranges.at(static_cast<uint16_t>(currAngle)))) {
				raw_scan_data.push_back(msg->range_max);
			} else {
				raw_scan_data.push_back(calcDerivedDist(currAngle));
			}
		}

		scan_data_.at(num) = computeMin(raw_scan_data);
	}

	auto message = wall_follower_msgs::msg::Scan();
	message.front = scan_data_.at(0);
	message.left = scan_data_.at(1);
	message.right = scan_data_.at(2);
	wf_scan_pub_->publish(message);
}

void WallFollower::update_cmd_vel(double linear, double angular) {
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback() {

	static TB3_States turtlebot3_state_num = GET_TB3_DIRECTION;
	// static TB3_States turtlebot3_state_num { TB3_DRIVE_STOP };

	/**
	 * TB3 variables (configure during testing)
	 */
	// set maximum turning angle (for TB3_LEFT_TURN)
	const double escape_range{ 5.0 * DEG2RAD };
	// set distances from the robot
	// the robot travels along the wall between min and max distances
	const double min_side_dist{ 0.2 };
	const double max_side_dist{ min_side_dist + 0.05 };
	const double max_forward_dist{ (min_side_dist + max_side_dist) / 2.0 + 0.05 };

	/**
	 * Wall Following cases:
	 * 1. distance from front wall > MAX or distance from right wall > MAX > MIN
	 *    - Turns slightly right (until it finds a wall)
	 * 2. distance from front wall < MAX
	 *    - Turns left
	 * 3. distance from right wall < MIN < MAX
	 *    - Turns slightly left
	 * 4. MIN < distance from right wall < MAX
	 *    - Goes straight
	 */

	switch (turtlebot3_state_num) {
		case GET_TB3_DIRECTION:
			/**
			 * Pause if all sensors read zero
			 * TODO: comment out during demo
			 */
			if (scan_data_[LEFT] == 0 && scan_data_[CENTER] == 0 &&
				scan_data_[RIGHT] == 0) {
				turtlebot3_state_num = TB3_DRIVE_STOP;
				break;
			}

			/**
			 * Wall Following Algorithm
			 */
			if (scan_data_[CENTER] > max_forward_dist) {
				if (
					scan_data_[RIGHT] < max_side_dist &&
					scan_data_[RIGHT] > min_side_dist
				) {
					// case 4
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				} else if (
					scan_data_[RIGHT] < max_side_dist &&
					scan_data_[RIGHT] < min_side_dist
				) {
					// case 3
					turtlebot3_state_num = TB3_DRIVE_FORWARD_SL;
				} else if (
					scan_data_[RIGHT] > max_side_dist &&
					scan_data_[RIGHT] > min_side_dist
				) {
					// case 1
					turtlebot3_state_num = TB3_DRIVE_FORWARD_SR;
				}
			} else {
				// case 2
				prev_robot_pose_ = robot_pose_;
				turtlebot3_state_num = TB3_LEFT_TURN;
			}
			break;

		case TB3_DRIVE_FORWARD:
			std::cout << "Drive Forward" << '\n';
			update_cmd_vel(LINEAR_VELOCITY, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_DRIVE_FORWARD_SR:
			std::cout << "Drive slight right" << '\n';
			driveRadius(LINEAR_VELOCITY, -min_side_dist);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_DRIVE_FORWARD_SL:
			std::cout << "Drive slight left" << '\n';
			driveRadius(LINEAR_VELOCITY/1.5, 0.1);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_RIGHT_TURN:
			std::cout << "Drive  Right" << '\n';
			// turtlebot3_state_num = GET_TB3_DIRECTION;
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
				turtlebot3_state_num = GET_TB3_DIRECTION;
			} else {
				update_cmd_vel(0.0, -ANGULAR_VELOCITY);
			}
			break;

		case TB3_LEFT_TURN:
			std::cout << "Drive Left" << '\n';
			// turtlebot3_state_num = GET_TB3_DIRECTION;
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
				turtlebot3_state_num = GET_TB3_DIRECTION;
			} else {
				update_cmd_vel(0.0, ANGULAR_VELOCITY);
			}
			break;

		/**
		 * Stop turtlebot
		 */
		case TB3_DRIVE_STOP:
			std::cout << "STOP!" << '\n';
			update_cmd_vel(0.0, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			// turtlebot3_state_num = TB3_DRIVE_STOP;
			break;

		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
	}
}

/********************************************************************************
** Helper functions
********************************************************************************/

bool WallFollower::inStartRange(double x, double y) {
	std::cout << x-startx_ << " " << y-starty_ << std::endl;
	const bool isInRange {
		(fabs (x-startx_) <= 0.05) &&
		(fabs(y-starty_) <= 0.05)
	};
	if (!(off_start_range_||isInRange))
		off_start_range_ = true;

	return isInRange;
}

void WallFollower::driveRadius(double linear, double radius) {
	double angular{};
	if (radius == 0.0)
		angular = ANGULAR_VELOCITY;
	else if (radius == INFINITY)
		angular = 0.0;
	else
		angular = linear/radius;
	update_cmd_vel(linear, angular);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}