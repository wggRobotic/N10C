#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class Communicator : public rclcpp::Node
{
public:
    Communicator();

private:
    void timer_callback();
    void primary_img_topic_callback();
    void secondary_img_topic_callback();
    void motion_img_topic_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr primaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr secondaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr motionImgSubscriber;
    size_t count_;
};

#endif // COMMUNICATOR_HPP
