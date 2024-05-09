#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "Communicator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Communicator>());
  rclcpp::shutdown();
  return 0;
}