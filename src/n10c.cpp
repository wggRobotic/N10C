#include <N10C/n10c.hpp>

using namespace std::chrono_literals;

N10C::N10C(int argc, const char **argv) : rclcpp::Node("n10c"), guitar::Application(argc, argv), m_Images(3) {}

float N10C::NoStickDrift(float x) { return std::fabs(x) > 0.1f ? x : 0.0f; }
