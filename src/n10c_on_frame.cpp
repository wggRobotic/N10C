#include <N10C/n10c.hpp>
static float addValueRangeCut(float min, float max, float delta, float& value ){
  if(delta + value > max){
    return max;
  }else if(delta + value < min){
    return min;
  }else{
    return value + delta;
  }
}

void N10C::OnFrame()
{
  constexpr float multiX = 1.0f, multiY = 1.0f, multiZ = 0.25f;

  if (Input().GetKeyRelease(GLFW_KEY_L)) { m_ActivatedLine = true; }
  if (Input().GetKeyRelease(GLFW_KEY_O)) { m_ActivatedLine = false; }


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
      m_GripperMessage.data.at(0) = 0.11;
      m_GripperMessage.data.at(1) = 0.10;
      m_GripperMessage.data.at(2) = -1;
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
      m_GripperMessage.data.at(0) = 0.11;
      m_GripperMessage.data.at(1) = 0.10;
      m_GripperMessage.data.at(2) = -1;
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
    m_GripperMessage.data.at(0) = addValueRangeCut(0, 0.27f,(NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Vertical")) * 0.001f), m_GripperMessage.data.at(0));
    m_GripperMessage.data.at(1) = addValueRangeCut(-0.14f, 0.27f,(NoStickDrift(Input().GetAxis(m_SelectedJoystick, "UpDown")) * 0.001f), m_GripperMessage.data.at(1));
    m_GripperMessage.data.at(2) = addValueRangeCut(0, 1, (NoStickDrift(Input().GetAxis(m_SelectedJoystick, "Gripper")) * 0.001f), m_GripperMessage.data.at(2));
  }

  if (!rclcpp::ok())
  {
    Close();
    return;
  }

  m_GripperPublisher->publish(m_GripperMessage);
  m_TwistPublisher->publish(m_TwistMessage);
}
