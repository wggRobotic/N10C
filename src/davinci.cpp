#include <N10C/davinci.hpp>
#include <guitar/image.hpp>
#include <iostream>

Davinci::Davinci(int argc, const char **argv) : Application(argc, argv), m_Communicator(std::make_shared<Communicator>())
{
  m_Thread = std::thread([this]() { rclcpp::spin(m_Communicator); });
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
}
