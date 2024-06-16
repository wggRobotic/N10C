#include <N10C/communicator.hpp>
#include <N10C/davinci.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

int main(const int argc, const char **argv)
{
  std::cout << "Initializing ROS" << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "Initializing Davinci" << std::endl;
  Davinci davinci(argc, argv);
  std::cout << "Launching Davinci" << std::endl;
  davinci.Launch();
  std::cout << "Shutting down ROS" << std::endl;
  rclcpp::shutdown();
  std::cout << "Exit" << std::endl;
  return 0;
}