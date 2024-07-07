#include <N10C/n10c.hpp>

int main(const int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  const auto n10c = std::make_shared<N10C>(argc, argv);
  image_transport::ImageTransport it(n10c);
  n10c->SetupWithImageTransport(it);
  std::thread gui([&n10c] { n10c->Launch(); });
  rclcpp::spin(n10c);
  rclcpp::shutdown();
  return 0;
}
