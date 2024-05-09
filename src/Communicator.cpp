#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Communicator : public rclcpp::Node
{
  public:
    Communicator()
    : Node("n10c"), count_(0)
    {
      velocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>("n10/cmd_vel", 10);
      primaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("test",10,primary_img_topic_callback);
      secondaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("test",10,secondary_img_topic_callback);
      motionImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("test",10,motion_img_topic_callback);
      timer_ = this->create_wall_timer(500ms, std::bind(&Communicator::timer_callback, this));
    }

    //topic callbacks

    void primary_img_topic_callback(){

    }
    void secondary_img_topic_callback(){

    }
    void motion_img_topic_callback(){

    }


  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x;
      message.linear.y;
      message.angular.z;
      velocityPublisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr primaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr secondaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr motionImgSubscriber;
    size_t count_;
};
