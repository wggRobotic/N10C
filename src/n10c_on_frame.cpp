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
    if (Input().GetKeyRelease(GLFW_KEY_T)) { m_ActivateGripper = false; }
    if (Input().GetKeyRelease(GLFW_KEY_G)) {m_ActivateGripper = true; }
    if (Input().GetKeyRelease(GLFW_KEY_B))
    { 
      m_GripperMessage.data.at(0) = 0;
      m_GripperMessage.data.at(1) = 0;
      m_GripperMessage.data.at(2) = 0;
    }
  }
  else
  {
    auto gamepad = Input().GetGamepad(m_SelectedJoystick);

    auto enable_pressed = gamepad.buttons[GLFW_GAMEPAD_BUTTON_A];
    auto disable_pressed = gamepad.buttons[GLFW_GAMEPAD_BUTTON_B];

    if (!m_EnableButtonPressed && enable_pressed) { m_ShouldSetMotorStatusTrue = true; }
    else if (!m_DisableButtonPressed && disable_pressed) { m_ShouldSetMotorStatusFalse = true; }

    if(gamepad.buttons[GLFW_GAMEPAD_BUTTON_Y]){m_ActivateGripper = true; }
    if(gamepad.buttons[GLFW_GAMEPAD_BUTTON_X]){m_ActivateGripper = false; }
    if (gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN])
    { 
      m_GripperMessage.data.at(0) = 0;
      m_GripperMessage.data.at(1) = 0;
      m_GripperMessage.data.at(2) = 0;
    }



    m_EnableButtonPressed = enable_pressed;
    m_DisableButtonPressed = disable_pressed;
  }

  if (!m_ActivateGripper)
  {
    m_TwistMessage.linear.x = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * multiX;
    m_TwistMessage.linear.y = NoStickDrift(-Input().GetAxis(m_SelectedJoystick, "Horizontal")) * multiY;
    m_TwistMessage.angular.z = NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Rotate")) * multiZ;
  }
  else
  {
    m_GripperMessage.data.at(0) += NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * 0.25;
    m_GripperMessage.data.at(1) += NoStickDrift(-Input().GetAxis(m_SelectedJoystick, "UpDown")) * 0.25;
    m_GripperMessage.data.at(2) += NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Gripper")) * 0.25;
  }

  if (!rclcpp::ok())
  {
    Close();
    return;
  }

  m_GripperPublisher->publish(m_GripperMessage);
  m_TwistPublisher->publish(m_TwistMessage);
}
