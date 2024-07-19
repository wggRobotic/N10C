#include <N10C/n10c.hpp>

constexpr float PI = 3.1415926535897932384626433;
constexpr float PI2 = PI / 2;

static float add_value_range_cut(float min, float max, float delta, float value)
{
	if (delta + value > max) { return max; }
	else if (delta + value < min) { return min; }
	else { return value + delta; }
}

void N10C::OnFrame()
{
	if (!rclcpp::ok())
	{
		Close();
		return;
	}

	if (Input().GetKeyRelease(GLFW_KEY_L)) { m_ActivatedLine = true; }
	if (Input().GetKeyRelease(GLFW_KEY_O)) { m_ActivatedLine = false; }
	if (Input().GetKeyRelease(GLFW_KEY_I))
	{
		m_ModX = m_ModX + 0.2f;
		m_ModY = m_ModY + 0.2f;
	}
	if (Input().GetKeyRelease(GLFW_KEY_K))
	{
		m_ModX -= 0.2f;
		m_ModY -= 0.2f;
	}

	if (m_SelectedJoystick < 0)
	{
		m_EnableButtonPressed = false;
		m_DisableButtonPressed = false;

		if (Input().GetKeyRelease(GLFW_KEY_F)) { m_ShouldSetMotorStatusTrue = true; }
		if (Input().GetKeyRelease(GLFW_KEY_R)) { m_ShouldSetMotorStatusFalse = true; }
		if (Input().GetKeyRelease(GLFW_KEY_T)) { m_ActivateGripper = false; }
		if (Input().GetKeyRelease(GLFW_KEY_G)) { m_ActivateGripper = true; }

		if (Input().GetKeyRelease(GLFW_KEY_Z)) { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, -0.05f, m_GripperMessage.data[2]); }
		else if (Input().GetKeyRelease(GLFW_KEY_C)) { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, 0.05f, m_GripperMessage.data[2]); }
		else { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, 0.05f, m_GripperMessage.data[2]); }

		if (Input().GetKeyRelease(GLFW_KEY_B))
		{
			m_GripperMessage.data[0] = 0.11;
			m_GripperMessage.data[1] = 0.10;
			m_GripperMessage.data[2] = -PI2;
			m_GripperMessage.data[3] = 0;
		}
	}
	else
	{
		auto gamepad = Input().GetGamepad(m_SelectedJoystick);

		auto enable_pressed = gamepad.buttons[GLFW_GAMEPAD_BUTTON_A];
		auto disable_pressed = gamepad.buttons[GLFW_GAMEPAD_BUTTON_B];

		if (!m_EnableButtonPressed && enable_pressed) { m_ShouldSetMotorStatusTrue = true; }
		else if (!m_DisableButtonPressed && disable_pressed) { m_ShouldSetMotorStatusFalse = true; }

		m_EnableButtonPressed = enable_pressed;
		m_DisableButtonPressed = disable_pressed;

		if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_LEFT]) { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, -0.01f, m_GripperMessage.data[2]); }
		else if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_RIGHT]) { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, 0.01f, m_GripperMessage.data[2]); }
		else { m_GripperMessage.data[2] = add_value_range_cut(-PI2, PI2, 0.01f, m_GripperMessage.data[2]); }

		if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_Y]) { m_ActivateGripper = true; }
		if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_X]) { m_ActivateGripper = false; }
		if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN])
		{
			m_GripperMessage.data[0] = 0.06;
			m_GripperMessage.data[1] = 0.10;
			m_GripperMessage.data[2] = -PI2;
			m_GripperMessage.data[3] = 0;
		}
	}

	if (!m_ActivateGripper)
	{
		m_TwistMessage.linear.x = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * m_ModX;
		m_TwistMessage.linear.y = NoStickDrift(-Input().GetAxis(m_SelectedJoystick, "Horizontal")) * m_ModY;
		m_TwistMessage.angular.z = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Rotate")) * m_ModZ;
	}
	else
	{
		m_GripperMessage.data[0] = add_value_range_cut(0, 0.27f, (NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * 0.001f), m_GripperMessage.data[0]);
		m_GripperMessage.data[1] = add_value_range_cut(-0.14f, 0.27f, (NoStickDrift(Input().GetAxis(m_SelectedJoystick, "UpDown")) * 0.001f), m_GripperMessage.data[1]);
		m_GripperMessage.data[3] = add_value_range_cut(0, 1, (NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Gripper")) * 0.001f), m_GripperMessage.data[3]);
	}

	m_GripperPublisher->publish(m_GripperMessage);
	m_TwistPublisher->publish(m_TwistMessage);
}
