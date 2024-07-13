#include <N10C/n10c.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

using namespace std::chrono_literals;

void N10C::SetupWithImageTransport(image_transport::ImageTransport &it)
{
  rclcpp::Parameter image0, image1, image2, image3, image4, barcode, velocity, enable, gripper, image_gripper;

  declare_parameter("image0", "/n10/image0");
  declare_parameter("image1", "/n10/image1");
  declare_parameter("image2", "/n10/image2");
  declare_parameter("image3", "/n10/image3");
  declare_parameter("image4", "/n10/image4");
  declare_parameter("barcode", "/n10/barcode");
  declare_parameter("twist", "/n10/twist");
  declare_parameter("enable", "/n10/enable");
  declare_parameter("gripper", "/n10/gripper");
  declare_parameter("image_gripper","/n10/image_gripper");

  get_parameter("image0", image0);
  get_parameter("image1", image1);
  get_parameter("image2", image2);
  get_parameter("image3", image3);
  get_parameter("image4", image4);
  get_parameter("barcode", barcode);
  get_parameter("twist", velocity);
  get_parameter("enable", enable);
  get_parameter("gripper", gripper);
  get_parameter("image_gripper",image_gripper);

  m_ImageSubscriber0 = it.subscribe(image0.as_string(), 10, &N10C::ImageCallback0, this);
  m_ImageSubscriber1 = it.subscribe(image1.as_string(), 10, &N10C::ImageCallback1, this);
  m_ImageSubscriber2 = it.subscribe(image2.as_string(), 10, &N10C::ImageCallback2, this);
  m_ImageSubscriber3 = it.subscribe(image3.as_string(), 10, &N10C::ImageCallback3, this);
  m_ImageSubscriber4 = it.subscribe(image4.as_string(), 10, &N10C::ImageCallback4, this);

  m_ImageGripperSubsciber = it.subscribe(image_gripper.as_string(),10, &N10C::ImageGripperCallback,this);

  m_BarcodeSubscriber = create_subscription<std_msgs::msg::String>(barcode.as_string(), 10, std::bind(&N10C::BarcodeCallback, this, std::placeholders::_1));

  m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>(velocity.as_string(), 10);
  m_GripperPublisher = create_publisher<std_msgs::msg::Float64MultiArray>(gripper.as_string(), 10);

  m_EnableMotorClient = create_client<std_srvs::srv::SetBool>(enable.as_string());

  m_Timer = create_wall_timer(20ms, std::bind(&N10C::TimerCallback, this));
}
