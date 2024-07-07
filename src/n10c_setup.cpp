#include <N10C/n10c.hpp>

using namespace std::chrono_literals;

void N10C::SetupWithImageTransport(image_transport::ImageTransport &it)
{
  rclcpp::Parameter image0, image1, image2, barcode, velocity, enable;

  declare_parameter("image0", "/n10/image0");
  declare_parameter("image1", "/n10/image1");
  declare_parameter("image2", "/n10/image2");
  declare_parameter("barcode", "/n10/barcode");
  declare_parameter("twist", "/n10/twist");
  declare_parameter("enable", "/n10/enable");

  get_parameter("image0", image0);
  get_parameter("image1", image1);
  get_parameter("image2", image2);
  get_parameter("barcode", barcode);
  get_parameter("twist", velocity);
  get_parameter("enable", enable);

  m_ImageSubscriber0 = it.subscribe(image0.as_string(), 10, &N10C::ImageCallback0, this);
  m_ImageSubscriber1 = it.subscribe(image1.as_string(), 10, &N10C::ImageCallback1, this);
  m_ImageSubscriber2 = it.subscribe(image2.as_string(), 10, &N10C::ImageCallback2, this);

  m_BarcodeSubscriber = create_subscription<std_msgs::msg::String>(barcode.as_string(), 10, std::bind(&N10C::BarcodeCallback, this, std::placeholders::_1));

  m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>(velocity.as_string(), 10);
  m_EnableMotorClient = create_client<std_srvs::srv::SetBool>(enable.as_string());

  m_Timer = create_wall_timer(20ms, std::bind(&N10C::TimerCallback, this));
}
