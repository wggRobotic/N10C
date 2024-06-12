#include <N10C/communicator.hpp>
#include <N10C/davinci.hpp>

using namespace std::chrono_literals;

Communicator::Communicator(Davinci &davinci) : Node("n10c"), m_Count(0), m_Davinci(davinci) {}

void Communicator::Init(image_transport::ImageTransport &it)
{
  m_PrimaryImgSubscriber = it.subscribe("/n10/camera/front", 10, &Communicator::PrimaryImageCallback, this);
  m_SecondaryImgSubscriber = it.subscribe("/n10/camera/rear", 10, &Communicator::SecondaryImageCallback, this);
  m_MotionImgSubscriber = it.subscribe("/n10/camera/motion", 10, &Communicator::MotionImageCallback, this);

  m_VelocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>("n10/cmd_vel", 10);
  m_BarCodeSubscriber = this->create_subscription<std_msgs::msg::String>("/n10/barcode", 10, std::bind(&Communicator::BarcodeCallback, this, std::placeholders::_1));

  m_EnableMotor = this->create_client<std_srvs::srv::SetBool>("eduard/enable");
  m_Timer = this->create_wall_timer(500ms, std::bind(&Communicator::TimerCallback, this));
}

std::string Communicator::EnableMotors(bool status)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = status;
  int tries = 0;
  for (; tries < 5 && !m_EnableMotor->wait_for_service(1s); ++tries)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("n10c"), "Interrupted while waiting for the service. Exiting.");
      return {};
    }
    RCLCPP_INFO(rclcpp::get_logger("n10c"), "Service not available, waiting again ...");
  }
  if (tries >= 5)
  {
    RCLCPP_INFO(rclcpp::get_logger("n10c"), "Tried more than 5 times to reconnect with service; timeout.");
    return {};
  }

  auto result = m_EnableMotor->async_send_request(request);
  return result.get()->message;
}

geometry_msgs::msg::Twist &Communicator::Twist() { return m_TwistMessage; }

const geometry_msgs::msg::Twist &Communicator::Twist() const { return m_TwistMessage; }

void Communicator::PrimaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg)
{
  m_Davinci.SetPrimaryImage(msg->data, msg->width, msg->height, msg->step, msg->encoding, msg->is_bigendian);
}

void Communicator::SecondaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &) {}

void Communicator::MotionImageCallback(const image_transport::ImageTransport::ImageConstPtr &) {}

void Communicator::BarcodeCallback(const std_msgs::msg::String::ConstSharedPtr &) {}

void Communicator::TimerCallback() { m_VelocityPublisher->publish(m_TwistMessage); }
