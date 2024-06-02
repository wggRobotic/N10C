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
  rclcpp::init(argc, argv);
  Davinci davinci(argc, argv);
  davinci.Launch();
  rclcpp::shutdown();
  return 0;
}