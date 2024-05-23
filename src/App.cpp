#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <N10C/Communicator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

int main(const int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Communicator>());
  rclcpp::shutdown();
  return 0;
}