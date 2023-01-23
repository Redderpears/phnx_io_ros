#include <cstdio>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp> 

#include "gz_io_ros/gz_io_ros.hpp"


namespace gir{

GzIoRos::GzIoRos(rclcpp::NodeOptions options)
: Node("gz_io_ros", options){
  _max_throttle_speed = this->declare_parameter("max_throttle_speed", 1.0);
  _max_braking_speed = this->declare_parameter("max_brake_speed", 1.0);

  _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/odom_ack", 10);

  _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/robot/cmd_vel", 10, std::bind(&GzIoRos::cmd_vel_cb, this, std::placeholders::_1));
}

void GzIoRos::odom_cb(nav_msgs::msg::Odometry::SharedPtr msg){}

void GzIoRos::cmd_vel_cb(geometry_msgs::msg::TwistStamped::SharedPtr msg){}

} //namespace gir


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto gir_node = std::make_shared<gir::GzIoRos>(options);
  exec.add_node(gir_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
