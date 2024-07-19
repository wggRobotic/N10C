#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <guitar/application.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>

using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using StringConstPtr = std_msgs::msg::String::ConstSharedPtr;

class N10C : public rclcpp::Node, public guitar::Application
{
public:
	N10C(int argc, const char **argv);

	void SetupWithImageTransport(image_transport::ImageTransport &);

protected:
	void OnStart() override;
	void OnFrame() override;
	void OnStop() override;

private:
	void BarcodeCallback(const StringConstPtr &);
	void TimerCallback();

	bool SetMotorStatus(bool status);
	float NoStickDrift(float x);

	void SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding);

private:
	// Images
	std::vector<guitar::Image> m_Images;
	std::vector<image_transport::Subscriber> m_ImageSubscribers;

	// Barcode
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_BarcodeSubscriber;
	std::map<std::string, size_t> m_Barcodes;

	// Twist
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
	geometry_msgs::msg::Twist m_TwistMessage;

	// Gripper
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperPublisher;
	std_msgs::msg::Float32MultiArray m_GripperMessage;
	bool m_ActivateGripper = false;
	bool m_ActivatedLine = false;

	// Enable
	rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_EnableMotorClient;
	bool m_SetMotorStatusLock = false;
	bool m_ShouldSetMotorStatusTrue = false;
	bool m_ShouldSetMotorStatusFalse = false;

	// Timer
	rclcpp::TimerBase::SharedPtr m_Timer;

	// Camera
	int m_SelectedCamera = 0;
	const std::vector<std::string> m_Cameras{ "Front", "Rear", "Motion", "Depth", "Thermal", "Gripper" };

	// Joystick
	int m_SelectedJoystick = -1;
	bool m_EnableButtonPressed = false;
	bool m_DisableButtonPressed = false;

	// Speed Modifiers
	float m_ModX = 1.0f, m_ModY = 1.0f, m_ModZ = 0.25f;
};
