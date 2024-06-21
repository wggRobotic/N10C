#include <N10C/n10c.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

N10C::N10C(int argc, const char **argv) : rclcpp::Node("n10c"), guitar::Application(argc, argv), m_Images(3) {}

void N10C::SetupWithImageTransport(image_transport::ImageTransport &it)
{
  rclcpp::Parameter image0, image1, image2, barcode, velocity, enable;

  auto has_image0 = get_parameter("image0", image0);
  auto has_image1 = get_parameter("image1", image1);
  auto has_image2 = get_parameter("image2", image2);
  auto has_barcode = get_parameter("barcode", barcode);
  auto has_velocity = get_parameter("twist", velocity);
  auto has_enable = get_parameter("enable", enable);

  m_ImageSubscriber0 = it.subscribe(has_image0 ? image0.as_string() : "/n10/image0", 10, &N10C::ImageCallback0, this);
  m_ImageSubscriber1 = it.subscribe(has_image1 ? image1.as_string() : "/n10/image1", 10, &N10C::ImageCallback1, this);
  m_ImageSubscriber2 = it.subscribe(has_image2 ? image2.as_string() : "/n10/image2", 10, &N10C::ImageCallback2, this);

  m_BarcodeSubscriber = create_subscription<std_msgs::msg::String>(has_barcode ? barcode.as_string() : "/n10/barcode", 10, std::bind(&N10C::BarcodeCallback, this, std::placeholders::_1));

  m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>(has_velocity ? velocity.as_string() : "/n10/velocity", 10);
  m_EnableMotorClient = create_client<std_srvs::srv::SetBool>(has_enable ? enable.as_string() : "/n10/enable");

  m_Timer = create_wall_timer(20ms, std::bind(&N10C::TimerCallback, this));
}

void N10C::OnStart()
{
  Events().Register(
      "on_key", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::KeyPayload *)pPayload;
        if (payload.Key == GLFW_KEY_ESCAPE && payload.Action == GLFW_RELEASE)
        {
          Close();
          return true;
        }

        if (payload.Key == GLFW_KEY_F11 && payload.Action == GLFW_RELEASE)
        {
          Schedule([this] { ToggleFullscreen(); });
          return true;
        }

        return false;
      });

  Events().Register(
      "selected_camera", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::StringPayload *)pPayload;
        payload.Result = m_Cameras[m_SelectedCamera];
        return true;
      });
  Events().Register(
      "select_camera", this,
      [this](const guitar::EventPayload *) -> bool
      {
        for (int i = 0; i < static_cast<int>(m_Cameras.size()); ++i)
        {
          ImGui::PushID(i);
          const bool selected = i == m_SelectedCamera;
          if (ImGui::Selectable(m_Cameras[i].c_str(), selected)) m_SelectedCamera = i;
          if (selected) ImGui::SetItemDefaultFocus();
          ImGui::PopID();
        }
        return true;
      });

  Events().Register(
      "camera", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::ImagePayload *)pPayload;

        if (m_SelectedCamera >= 0 && m_SelectedCamera < static_cast<int>(m_Images.size()))
        {
          const auto &image = m_Images[m_SelectedCamera];
          payload.Width = image.Width;
          payload.Height = image.Height;
          payload.TextureID = (void *)(intptr_t)image.Texture;
          return true;
        }

        return false;
      });

  Events().Register(
      "selected_joystick", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::StringPayload *)pPayload;
        if (m_SelectedJoystick < 0)
        {
          payload.Result = "Keyboard";
          return true;
        }

        auto joysticks = Input().ListJoysticks();
        payload.Result = joysticks[m_SelectedJoystick];
        return true;
      });
  Events().Register(
      "select_joystick", this,
      [this](const guitar::EventPayload *) -> bool
      {
        auto joysticks = Input().ListJoysticks();
        if (joysticks.count(m_SelectedJoystick)) m_SelectedJoystick = -1;

        for (int i = -1; i < 16; ++i)
        {
          if (i >= 0 && joysticks[i].empty()) continue;

          ImGui::PushID(i);
          const bool selected = i == m_SelectedJoystick;
          if (ImGui::Selectable(i < 0 ? "Keyboard" : joysticks[i].c_str(), selected)) m_SelectedJoystick = i;
          if (selected) ImGui::SetItemDefaultFocus();
          ImGui::PopID();
        }
        return true;
      });
}

void N10C::OnFrame()
{
  constexpr float multiX = 1.0f, multiY = 1.0f, multiZ = 0.25f;

  if (m_SelectedJoystick < 0)
  {
    m_EnableButtonPressed = false;
    m_DisableButtonPressed = false;

    if (Input().GetKeyRelease(GLFW_KEY_F)) { m_ShouldSetMotorStatusTrue = true; }
    if (Input().GetKeyRelease(GLFW_KEY_R)) { m_ShouldSetMotorStatusFalse = true; }

    m_TwistMessage.linear.y = (Input().GetKey(GLFW_KEY_D) - Input().GetKey(GLFW_KEY_A)) * multiY;
    m_TwistMessage.linear.x = (Input().GetKey(GLFW_KEY_W) - Input().GetKey(GLFW_KEY_S)) * multiX;
    m_TwistMessage.angular.z = (Input().GetKey(GLFW_KEY_Q) - Input().GetKey(GLFW_KEY_E)) * multiZ;
  }
  else
  {
    guitar::Joystick joystick = Input().GetJoystick(m_SelectedJoystick);

    m_TwistMessage.linear.y = NoStickDrift(joystick.Axes[0]) * multiY;
    m_TwistMessage.linear.x = NoStickDrift(joystick.Axes[1]) * multiX;
    m_TwistMessage.angular.z = NoStickDrift(joystick.Axes[3]) * multiZ;

    if (!m_EnableButtonPressed && joystick.Buttons[0]) { m_ShouldSetMotorStatusTrue = true; }
    else if (!m_DisableButtonPressed && joystick.Buttons[1]) { m_ShouldSetMotorStatusFalse = true; }

    m_EnableButtonPressed = joystick.Buttons[0];
    m_DisableButtonPressed = joystick.Buttons[1];
  }

  if (!rclcpp::ok())
  {
    Close();
    return;
  }

  m_TwistPublisher->publish(m_TwistMessage);
}

void N10C::OnStop()
{
  if (rclcpp::ok()) rclcpp::shutdown();
}

void N10C::ImageCallback0(const ImageConstPtr &msg) { SetImage(0, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

void N10C::ImageCallback1(const ImageConstPtr &msg) { SetImage(1, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

void N10C::ImageCallback2(const ImageConstPtr &msg) { SetImage(2, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

void N10C::BarcodeCallback(const StringConstPtr &msg) { std::cout << msg->data << std::endl; }

void N10C::TimerCallback()
{
  if (m_ShouldSetMotorStatusTrue)
  {
    m_ShouldSetMotorStatusTrue = false;
    SetMotorStatus(true);
  }
  if (m_ShouldSetMotorStatusFalse)
  {
    m_ShouldSetMotorStatusFalse = false;
    SetMotorStatus(false);
  }
}

bool N10C::SetMotorStatus(bool state)
{
  if (m_SetMotorStatusLock)
  {
    std::cerr << "Motor Status request is locked" << std::endl;
    return false;
  }

  m_SetMotorStatusLock = true;
  for (size_t tries = 0; !m_EnableMotorClient->wait_for_service(1s); ++tries)
  {
    std::cout << "Attempt #" << tries + 1 << std::endl;
    if (!rclcpp::ok())
    {
      std::cerr << "Interrupted" << std::endl;
      m_SetMotorStatusLock = false;
      return false;
    }
    if (tries >= 4)
    {
      std::cerr << "Timeout" << std::endl;
      m_SetMotorStatusLock = false;
      return false;
    }
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  auto future = m_EnableMotorClient->async_send_request(request);

  bool success = false;
  switch (future.wait_for(1s))
  {
  case std::future_status::ready:
    std::cout << future.get()->message << std::endl;
    success = future.get()->success;
    break;
  case std::future_status::deferred: std::cerr << "Request deferred" << std::endl; break;
  case std::future_status::timeout: std::cerr << "Request timeout" << std::endl; break;
  }

  m_SetMotorStatusLock = false;
  return success;
}

float N10C::NoStickDrift(float x) { return std::fabs(x) > 0.1f ? x : 0.0f; }

static int convert_to_rgba(unsigned char *dst, const unsigned char *src, size_t count, uint8_t bpp, const std::string &encoding)
{
  auto has_alpha = sensor_msgs::image_encodings::hasAlpha(encoding);
  auto is_rgb = encoding == sensor_msgs::image_encodings::RGB8 || encoding == sensor_msgs::image_encodings::RGBA8;
  auto is_bgr = encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::BGRA8;

  if (!is_rgb && !is_bgr)
  {
    std::cerr << "Cannot convert unsupported encoding '" << encoding << "' to RGBA" << std::endl;
    return 1;
  }

  for (size_t i = 0; i < count; ++i)
  {
    auto pi = i * 4;
    auto di = i * bpp;
    dst[pi + 0] = is_rgb ? src[di + 0] : src[di + 2];
    dst[pi + 1] = is_rgb ? src[di + 1] : src[di + 1];
    dst[pi + 2] = is_rgb ? src[di + 2] : src[di + 0];
    dst[pi + 3] = has_alpha ? src[di + 3] : 0xff;
  }

  return 0;
}

void N10C::SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
  if (!IsActive()) return;

  auto bpp = step / width;
  auto pixels = new unsigned char[width * height * 4];

  if (convert_to_rgba(pixels, data.data(), width * height, bpp, encoding))
  {
    delete[] pixels;
    return;
  }

  Schedule(
      [this, index, width, height, pixels]()
      {
        m_Images[index].StorePixels(width, height, pixels);
        delete[] pixels;
      });
}
