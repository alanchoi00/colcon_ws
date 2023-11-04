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
#include "circle_model/circle_model.hpp"

#include <memory>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std::chrono_literals;

WallFollower::WallFollower() : Node("wall_follower_node") {
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;
  scan_data_[3] = 0.0;
  scan_data_[4] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;


  at_start_ = false;
  tolerance_ = 0.15;
  tb3_path_filename_ = "src/wall_follower/turtlebot3_path.txt";

  // clears path file
  std::ofstream outfile(tb3_path_filename_);
  outfile.close();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

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

  // Prints coordinate to turtlebot3_path.txt
  std::ofstream outfile(tb3_path_filename_, std::ios::app);
  if (!outfile.tellp()) {
    outfile << "Starting X:" << msg->pose.pose.position.x << ' '
            << "Starting Y:" << msg->pose.pose.position.y
            << '\n';
		startx_ = msg->pose.pose.position.x;
		starty_ = msg->pose.pose.position.y;
  } else {
	  outfile << "X:" << msg->pose.pose.position.x
            << " Y:" << msg->pose.pose.position.y
            << '\n';
  }

  if (inRange(msg->pose.pose.position.x, msg->pose.pose.position.y) && at_start_) {
    update_cmd_vel(0.0, 0.0);
    // exports map to ~/colcon_ws
    system("ros2 run nav2_map_server map_saver_cli -f src/wall_follower/my_map");
    exit(0);
	}
}

void printVector(const std::vector<double>& data) {
  std::cout << "{";
  for (double val : data)
    std::cout << val << ", ";
  std::cout << "}" << '\n';
}

double computeAverageWithoutOutliers(const std::vector<double>& data) {
  if (data.size() < 2) {
      throw std::runtime_error("Data set is too small");
  }
  // printVector(data);

  // Step 1: Sort the data
  std::vector<double> sortedData(data);
  std::sort(sortedData.begin(), sortedData.end());

  // Step 2: Compute Q1 and Q3
  int n = sortedData.size();
  double Q1 = (n % 2) ? sortedData[n / 4] : 0.5 * (sortedData[n / 4 - 1] + sortedData[n / 4]);
  double Q3 = (n % 2) ? sortedData[3 * n / 4] : 0.5 * (sortedData[3 * n / 4 - 1] + sortedData[3 * n / 4]);

  // Step 3: Compute IQR
  double IQR = Q3 - Q1;

  // Step 4: Determine outliers threshold
  double lowerBound = Q1 - 1.5 * IQR;
  double upperBound = Q3 + 1.5 * IQR;

  // Step 5: Compute average without outliers
  double sum = 0;
  int validCount = 0;
  for (double val : data) {
    if (val >= lowerBound && val <= upperBound) {
      sum += val;
      validCount++;
    }
  }

  if (validCount == 0) {
    throw std::runtime_error("All data points are outliers");
  }
  return sum / validCount;
}

double computeMin(const std::vector<double>& data) {
  double min = data.at(0);
  for (double val : data) {
    if (val < min) {
      min = val;
    }
  }
  return min;
}

void WallFollower::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	//											  f  fl  l   r    fr
  std::pair<uint16_t, uint16_t> scan_angle[3] = {
    {0, 40}, {50, 40}, {310, 40}};

  for (int num = 0; num < 3; num++) {
    double min_angle{ scan_angle[num].first - (scan_angle[num].second / 2.0) };
    double max_angle{ scan_angle[num].first + (scan_angle[num].second / 2.0) };

    std::vector<double>raw_scan_data{};

    for (double angle = min_angle; angle <= max_angle; ++angle) {
      double tmp_angle{angle};
      if (angle < 0) tmp_angle += 360.0;
      else if (angle >= 360.0) tmp_angle -= 360.0;
      if (std::isinf(msg->ranges.at(static_cast<uint16_t>(tmp_angle)))) {
        raw_scan_data.push_back(msg->range_max);
      } else {
        raw_scan_data.push_back(msg->ranges.at(static_cast<uint16_t>(tmp_angle)));
      }
    }

    // printVector(raw_scan_data);
    scan_data_[num] = computeMin(raw_scan_data);
  }

  std::cout << "scan_data_[" << "Left" << "]" << "->" << scan_data_[1] << '\n';
  std::cout << "scan_data_[" << "Front" << "]" << "->" << scan_data_[0] << '\n';
  std::cout << "scan_data_[" << "Right" << "]" << "->" << scan_data_[2] << '\n';
  std::cout << "------------------------" << '\n';
  // uint16_t scan_angle[5] = {0, 50, 70, 280, 310};

  // for (int num = 0; num < 5; num++) {
  //   if (std::isinf(msg->ranges.at(scan_angle[num]))) {
  //     scan_data_[num] = msg->range_max;
  //   } else {
  //     scan_data_[num] = msg->ranges.at(scan_angle[num]);
  //   }
  // }
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

	/**
	 * Prints all 5 degrees of sensors (debugging on site)
	 */
	/*
  std::cout << "scan_data_[]:\n"\
						<< " - Left: " << scan_data_[LEFT] << '\n'\
						<< " - Front Left: " << scan_data_[F_LEFT] << '\n'\
						<< " - Center: " << scan_data_[CENTER] << '\n'\
						<< " - Front Right: " << scan_data_[RIGHT] << '\n'\
						<< " - Right: " << scan_data_[RIGHT] << '\n';
	*/

  // static TB3_States turtlebot3_state_num = GET_TB3_DIRECTION;
  static TB3_States turtlebot3_state_num = TB3_DRIVE_STOP;

  /**
   * TB3 variables (configure during testing)
   */
  // set maximum turning angle (for TB3_LEFT_TURN)
  double escape_range = 2.0 * DEG2RAD;
  // set distances from the robot
  // the robot travels along the wall between min and max distances
  double min_side_dist = 0.4;
  double max_side_dist = min_side_dist + 0.05;
  double max_forward_dist = max_side_dist + 0.05;

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
      driveRadius(LINEAR_VELOCITY, INFINITY);
      // update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_DRIVE_FORWARD_SR:
      std::cout << "Drive Forward - slight right" << '\n';
      // update_cmd_vel(LINEAR_VELOCITY, -(ANGULAR_VELOCITY-0.3));
      driveRadius(LINEAR_VELOCITY, -min_side_dist);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

		case TB3_DRIVE_FORWARD_SL:
			std::cout << "Drive Forward - slight left" << '\n';
			// update_cmd_vel(LINEAR_VELOCITY-0.08, (ANGULAR_VELOCITY-0.1));
      driveRadius(LINEAR_VELOCITY/1.5, 0.1);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

    case TB3_RIGHT_TURN:
      std::cout << "Turn Right" << '\n';
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, -ANGULAR_VELOCITY);
      }
      break;

    case TB3_LEFT_TURN:
      std::cout << "Turn Left" << '\n';
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
				std::cout << "fabs(prev_robot_pose_ - robot_pose_) >= "
                  << escape_range << "deg"
                  << '\n';
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

		/**
		 * Stop turtlebot
		 */
    case TB3_DRIVE_STOP:
      update_cmd_vel(0.0, 0.0);
      // turtlebot3_state_num = GET_TB3_DIRECTION;
      turtlebot3_state_num = TB3_DRIVE_STOP;
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

/********************************************************************************
** Helper functions
********************************************************************************/

bool WallFollower::inRange(double x, double y) {
  bool isInRange =
    (fabs (x-startx_) <= tolerance_) && (fabs(y-starty_) <= tolerance_);
  if (!(at_start_||isInRange))
	  at_start_ = true;

  return isInRange;
}

std::string WallFollower::getPathFileName() const {
  return tb3_path_filename_;
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