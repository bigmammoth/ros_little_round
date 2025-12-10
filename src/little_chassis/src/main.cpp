#include "little_chassis/little_chassis_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<little_chassis::LittleChassisNode>();
  node->InitializeInterfaces();
  const auto threads = std::max(1u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), threads);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
