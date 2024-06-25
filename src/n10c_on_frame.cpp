#include <N10C/n10c.hpp>

void N10C::OnFrame()
{
  constexpr float multiX = 1.0f, multiY = 1.0f, multiZ = 0.25f;

  if (m_SelectedJoystick < 0)
  {
    m_EnableButtonPressed = false;
    m_DisableButtonPressed = false;

    if (Input().GetKeyRelease(GLFW_KEY_F)) { m_ShouldSetMotorStatusTrue = true; }
    if (Input().GetKeyRelease(GLFW_KEY_R)) { m_ShouldSetMotorStatusFalse = true; }
  }
  else
  {
    guitar::Joystick joystick = Input().GetJoystick(m_SelectedJoystick);

    auto enable_pressed = joystick.Buttons[GLFW_GAMEPAD_BUTTON_A];
    auto disable_pressed = joystick.Buttons[GLFW_GAMEPAD_BUTTON_B];

    if (!m_EnableButtonPressed && enable_pressed) { m_ShouldSetMotorStatusTrue = true; }
    else if (!m_DisableButtonPressed && disable_pressed) { m_ShouldSetMotorStatusFalse = true; }

    m_EnableButtonPressed = enable_pressed;
    m_DisableButtonPressed = disable_pressed;
  }

  m_TwistMessage.linear.y = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Horizontal")) * multiY;
  m_TwistMessage.linear.x = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * multiX;
  m_TwistMessage.angular.z = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Rotate")) * multiZ;

  if (!rclcpp::ok())
  {
    Close();
    return;
  }

  m_TwistPublisher->publish(m_TwistMessage);
}
