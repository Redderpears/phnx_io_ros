#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "phnx_io_ros/phnx_io_ros.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto pir_node = std::make_shared<pir::PhnxIoRos>(options);
    exec.add_node(pir_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
