#include <N10C/communicator.hpp>
#include <N10C/davinci.hpp>

using namespace std::chrono_literals;

Communicator::Communicator(Davinci &davinci) : Node("n10c"), m_Count(0), m_Davinci(davinci)
{
  m_VelocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>("n10/cmd_vel", 10);

  m_PrimaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test", 10, std::bind(&Communicator::PrimaryImageCallback, this, std::placeholders::_1));
  m_SecondaryImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test", 10, std::bind(&Communicator::SecondaryImageCallback, this, std::placeholders::_1));
  m_MotionImgSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/n10/test", 10, std::bind(&Communicator::MotionImageCallback, this, std::placeholders::_1));

  m_BarCodeSubscriber = this->create_subscription<std_msgs::msg::String>("/n10/barcode", 10, std::bind(&Communicator::BarcodeCallback, this, std::placeholders::_1));

  m_EnableMotor = this->create_client<std_srvs::srv::SetBool>("eduard/enable");
  m_Timer = this->create_wall_timer(500ms, std::bind(&Communicator::TimerCallback, this));
}

std::string Communicator::EnableMotors(bool status)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = status;
  while (!m_EnableMotor->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the sevice. Exiting.");
      return {};
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "serice not available, waiting again ...");
  }
  auto result = m_EnableMotor->async_send_request(request);
  return result.get()->message;
}

void Communicator::PrimaryImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  m_Davinci.SetPrimaryImage(msg->data, msg->width, msg->height, msg->step, msg->encoding, msg->is_bigendian);
}

void Communicator::SecondaryImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &) {}

void Communicator::MotionImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &) {}

void Communicator::BarcodeCallback(const std_msgs::msg::String::ConstSharedPtr &) {}

void Communicator::TimerCallback()
{
  auto message = geometry_msgs::msg::Twist();
  (void)message.linear.x;
  (void)message.linear.y;
  (void)message.angular.z;
  m_VelocityPublisher->publish(message);
}
