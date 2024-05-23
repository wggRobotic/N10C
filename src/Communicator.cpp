#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
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


      primaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test",10,std::bind(&Communicator::primary_img_topic_callback,this,std::placeholders::_1));
      secondaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test",10,std::bind(&Communicator::secondary_img_topic_callback,this,std::placeholders::_1));
      motionImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test",10,std::bind(&Communicator::motion_img_topic_callback,this,std::placeholders::_1));

      barCodeSubscriber = this->create_subscription<std_msgs::msg::String>("/n10/barcode",10,std::bind(&Communicator::barcode_topic_callback,this,std::placeholders::_1));

      // ToDo: Add ServiceClient for Enable Eduard
      enableMotor = this->create_client<std_srvs::srv::SetBool>("eduard/enable");
      timer_ = this->create_wall_timer(500ms, std::bind(&Communicator::timer_callback, this));
    }

    // --- topic callbacks ---
    void primary_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
      
    }
    void secondary_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){

    }
    void motion_img_topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){

    }
    void barcode_topic_callback(const std_msgs::msg::String::ConstSharedPtr &msg){

    }

    std::string enableMotors(bool status){
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = status;
      while(!enableMotor->wait_for_service(1s)){
        if(!rclcpp::ok()){
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Interrupted while waiting for the sevice. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"serice not available, waiting again ...");
      }
      auto result = enableMotor->async_send_request(request);
      return result.get()->message;
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


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr primaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr secondaryImgSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr motionImgSubscriber;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barCodeSubscriber;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enableMotor;


    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
