#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wall_follower_msgs/msg/scan.hpp"

class WallFollowerScanLog : public rclcpp::Node
{
public:
  WallFollowerScanLog()
  : Node("wall_follower_scan_log")
  {
    wf_scan_sub = this->create_subscription<wall_follower_msgs::msg::Scan>(
      "wall_follower_scan", rclcpp::SensorDataQoS(),
      std::bind(&WallFollowerScanLog::wall_follower_scan_callback, this, std::placeholders::_1));
  }

private:
  void wall_follower_scan_callback(const wall_follower_msgs::msg::Scan::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "L: %f F: %f R: %f", msg->left, msg->front, msg->right);
  }
  rclcpp::Subscription<wall_follower_msgs::msg::Scan>::SharedPtr wf_scan_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollowerScanLog>());
  rclcpp::shutdown();
  return 0;
}
