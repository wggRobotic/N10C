#pragma once

#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class Communicator : public rclcpp::Node
{
public:
    Communicator();

    std::string enableMotors(bool status);

private:
    void timer_callback();
    void primary_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void secondary_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void motion_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void barcode_topic_callback(const std_msgs::msg::String::ConstSharedPtr &msg);

    rclcpp::TimerBase::SharedPtr m_Timer;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_VelocityPublisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_PrimaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_SecondaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_MotionImgSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_BarCodeSubscriber;

    std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> m_EnableMotor;

    size_t m_Count;
};

#endif // COMMUNICATOR_HPP
