#include <N10C/n10c.hpp>

using namespace std::chrono_literals;

#define MAKE_CALLBACK(INDEX) [this, INDEX](const ImageConstPtr &msg) { SetImage(INDEX, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

void N10C::SetupWithImageTransport(image_transport::ImageTransport &it)
{
	rclcpp::Parameter barcode, velocity, enable, gripper;

	declare_parameter("barcode", "/n10/barcode");
	declare_parameter("twist", "/n10/twist");
	declare_parameter("enable", "/n10/enable");
	declare_parameter("gripper", "/n10/gripper");

	get_parameter("barcode", barcode);
	get_parameter("twist", velocity);
	get_parameter("enable", enable);
	get_parameter("gripper", gripper);

	m_ImageSubscribers.resize(m_Images.size());
	for (size_t i = 0; i < m_ImageSubscribers.size(); ++i)
	{
		rclcpp::Parameter image;
		declare_parameter("image" + std::to_string(i), "/n10/image" + std::to_string(i));
		get_parameter("image" + std::to_string(i), image);
		m_ImageSubscribers[i] = it.subscribe(image.as_string(), 10, MAKE_CALLBACK(i));
	}

	m_BarcodeSubscriber = create_subscription<std_msgs::msg::String>(barcode.as_string(), 10, std::bind(&N10C::BarcodeCallback, this, std::placeholders::_1));

	m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>(velocity.as_string(), 10);
	m_GripperPublisher = create_publisher<std_msgs::msg::Float32MultiArray>(gripper.as_string(), 10);

	m_EnableMotorClient = create_client<std_srvs::srv::SetBool>(enable.as_string());

	m_Timer = create_wall_timer(20ms, std::bind(&N10C::TimerCallback, this));
}
