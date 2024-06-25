#include <N10C/n10c.hpp>

void N10C::OnStop()
{
  if (rclcpp::ok()) rclcpp::shutdown();
}
