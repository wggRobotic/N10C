#pragma once

#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

using namespace std::chrono_literals;

class Communicator : public rclcpp::Node
{
public:
  explicit Communicator(class Davinci &davinci);

  void Init(image_transport::ImageTransport &it);

  std::string EnableMotors(bool status);

private:
  void TimerCallback();
  void PrimaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg);
  void SecondaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg);
  void MotionImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg);
  void BarcodeCallback(const std_msgs::msg::String::ConstSharedPtr &msg);

  rclcpp::TimerBase::SharedPtr m_Timer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_VelocityPublisher;
  image_transport::Subscriber m_PrimaryImgSubscriber;
  image_transport::Subscriber m_SecondaryImgSubscriber;
  image_transport::Subscriber m_MotionImgSubscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_BarCodeSubscriber;

  std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> m_EnableMotor;

  size_t m_Count;

  class Davinci &m_Davinci;
};

#endif // COMMUNICATOR_HPP
