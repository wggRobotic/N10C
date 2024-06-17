#include <N10C/communicator.hpp>
#include <N10C/davinci.hpp>
#include <thread>

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
  m_Timer = this->create_wall_timer(10ms, std::bind(&Communicator::TimerCallback, this));
}

void Communicator::EnableMotors(bool status)
{
  if (m_RequestingEnable)
  {
    std::cout << "Waiting for previous request to return" << std::endl;
    return;
  }

  m_RequestingEnable = true;

  m_Tasks.emplace_back(
      [this, status]
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = status;
        int tries = 0;
        for (; tries < 5 && !m_EnableMotor->wait_for_service(1s); ++tries)
        {
          if (!rclcpp::ok())
          {
	    std::cout << "Interrupted while waiting for the service. Exiting." << std::endl;
	    m_RequestingEnable = false;
	    return;
          }
          std::cout << "Service not available, waiting again ..." << std::endl;
        }
        if (tries >= 5)
        {
          std::cout << "Tried more than 5 times to reconnect with service; timeout." << std::endl;
          m_RequestingEnable = false;
          return;
        }

        auto result = m_EnableMotor->async_send_request(request);
        std::cout << result.get()->message << std::endl;

        m_RequestingEnable = false;
      });
}

geometry_msgs::msg::Twist &Communicator::Twist() { return m_TwistMessage; }

const geometry_msgs::msg::Twist &Communicator::Twist() const { return m_TwistMessage; }

void Communicator::PrimaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg)
{
  m_Davinci.SetImage(0, msg->data, msg->width, msg->height, msg->step, msg->encoding, msg->is_bigendian);
}

void Communicator::SecondaryImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg)
{
  m_Davinci.SetImage(1, msg->data, msg->width, msg->height, msg->step, msg->encoding, msg->is_bigendian);
}

void Communicator::MotionImageCallback(const image_transport::ImageTransport::ImageConstPtr &msg)
{
  m_Davinci.SetImage(2, msg->data, msg->width, msg->height, msg->step, msg->encoding, msg->is_bigendian);
}

void Communicator::BarcodeCallback(const std_msgs::msg::String::ConstSharedPtr &) {}

void Communicator::TimerCallback()
{
  for (const auto &task : m_Tasks) task();
  m_Tasks.clear();

  m_VelocityPublisher->publish(m_TwistMessage);
}
