#include <N10C/davinci.hpp>
#include <guitar/image.hpp>
#include <iostream>
#include <sensor_msgs/image_encodings.hpp>

Davinci::Davinci(int argc, const char **argv) : Application(argc, argv), m_Communicator(std::make_shared<Communicator>(*this))
{
  m_Thread = std::thread([this]() { rclcpp::spin(m_Communicator); });
}

static int convert_to_argb(unsigned char *dst, const unsigned char *src, size_t count, uint8_t bpp, const std::string &encoding)
{
  auto has_alpha = sensor_msgs::image_encodings::hasAlpha(encoding);
  auto is_rgb = encoding == sensor_msgs::image_encodings::RGB8 || encoding == sensor_msgs::image_encodings::RGBA8;
  auto is_bgr = encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::BGRA8;

  if (!is_rgb && !is_bgr)
  {
    std::cout << "Cannot convert unsupported encoding '" << encoding << "' to ARGB" << std::endl;
    return 1;
  }

  for (size_t i = 0; i < count; ++i)
  {
    auto pi = i * 4;
    auto di = i * bpp;
    dst[pi + 0] = has_alpha ? src[di + 3] : 0xff;
    dst[pi + 1] = is_rgb ? src[di + 0] : src[di + 2];
    dst[pi + 2] = is_rgb ? src[di + 1] : src[di + 1];
    dst[pi + 3] = is_rgb ? src[di + 2] : src[di + 0];
  }

  return 0;
}

void Davinci::SetPrimaryImage(const std::vector<unsigned char> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding, uint8_t is_bigendian)
{
  (void)is_bigendian;

  auto bpp = step / width;
  auto pixels = new unsigned char[width * height * 4];

  if (!convert_to_argb(pixels, data.data(), width * height, bpp, encoding)) m_PrimaryImage.StorePixels(width, height, pixels);

  delete[] pixels;
}

void Davinci::OnInit(guitar::AppConfig &config)
{
  image_transport::ImageTransport it(m_Communicator);
  m_Communicator->Init(it);
}

void Davinci::OnStart()
{
  Events().Register(
      "on_key", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::KeyPayload *)pPayload;
        if (payload.Key == GLFW_KEY_ESCAPE && payload.Action == GLFW_RELEASE)
        {
          this->Close();
          return true;
        }

        if (payload.Key == GLFW_KEY_F11 && payload.Action == GLFW_RELEASE)
        {
          this->Schedule([this]() { this->ToggleFullscreen(); });
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

        switch (m_SelectedCamera)
        {
        case 0: // Front
          payload.Width = m_PrimaryImage.Width;
          payload.Height = m_PrimaryImage.Height;
          payload.TextureID = (void *)(intptr_t)m_PrimaryImage.Texture;
          return true;

        case 1: // TODO: Rear
          payload.Width = 0;
          payload.Height = 0;
          payload.TextureID = (void *)(intptr_t)0;
          return true;

        case 2: // TODO: Thermal
          payload.Width = 0;
          payload.Height = 0;
          payload.TextureID = (void *)(intptr_t)0;
          return true;

        case 3: // TODO: Depth
          payload.Width = 0;
          payload.Height = 0;
          payload.TextureID = (void *)(intptr_t)0;
          return true;
        }

        return false;
      });

  Events().Register(
      "selected_joystick", this,
      [this](const guitar::EventPayload *pPayload) -> bool
      {
        const auto &payload = *(guitar::StringPayload *)pPayload;
        auto joysticks = Input().ListJoysticks();
        payload.Result = joysticks[m_SelectedJoystick];
        return true;
      });
  Events().Register(
      "select_joystick", this,
      [this](const guitar::EventPayload *) -> bool
      {
        auto joysticks = Input().ListJoysticks();
        for (int i = 0; i < 16; ++i)
        {
          if (joysticks[i].empty()) continue;

          ImGui::PushID(i);
          const bool selected = i == m_SelectedJoystick;
          if (ImGui::Selectable(joysticks[i].c_str(), selected)) m_SelectedJoystick = i;
          if (selected) ImGui::SetItemDefaultFocus();
          ImGui::PopID();
        }
        return true;
      });
}

void Davinci::OnFrame()
{
  guitar::Joystick joystick = Input().GetJoystick(m_SelectedJoystick);

  if (joystick.Name.empty())
  {
    // Keyboard
  }
  else
  {
    // Joystick
    joystick.Dump();
  }
}

void Davinci::OnImGui() { ImGui::ShowMetricsWindow(); }
