#include <N10C/n10c.hpp>

int main(const int argc, const char **argv)
{
	std::cout << "Init RCL" << std::endl;
	rclcpp::init(argc, argv);

	std::cout << "Init N10C" << std::endl;
	const auto n10c = std::make_shared<N10C>(argc, argv);

	std::cout << "Init Image Transport" << std::endl;
	image_transport::ImageTransport it(n10c);
	n10c->SetupWithImageTransport(it);

	std::cout << "Launch GUI" << std::endl;
	std::thread gui([&n10c] { n10c->Launch(); });

	std::cout << "Spin RCL" << std::endl;
	rclcpp::spin(n10c);

	std::cout << "Shutdown RCL" << std::endl;
	rclcpp::shutdown();
}
