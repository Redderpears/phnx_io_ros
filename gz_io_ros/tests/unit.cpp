#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "gz_io_ros/gz_io_ros.hpp"

TEST(gz_io_ros, MsgConvertTest) {
    //Test that output ackermann messages are being converted correctly
    rclcpp::NodeOptions opts;
    gir::GzIoRos node{opts};

    ackermann_msgs::msg::AckermannDrive a{};
    nav_msgs::msg::Odometry o{};
    a.speed = 5.0;
    a.steering_angle = 1.57;
    o.twist.twist.linear.x = 2.5;
    ackermann_msgs::msg::AckermannDrive msg =
        node.convert_data(std::make_shared<nav_msgs::msg::Odometry>(o), a);
    EXPECT_FLOAT_EQ(msg.acceleration, a.speed) << "Acceleration must be equal to acks speed!";
    EXPECT_FLOAT_EQ(msg.speed, o.twist.twist.linear.x) << "Speed must be equal to odom's linear x!";
    EXPECT_FLOAT_EQ(msg.jerk, 0.0) << "Jerk must be equal to zero!";
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}
