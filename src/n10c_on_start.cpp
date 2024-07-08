#include "imgui.h"
#include <N10C/n10c.hpp>
#include <string>

void N10C::OnStart()
{
  Events().Register(
      "on_key", this,
      [this](const guitar::EventBase *pEvent) -> bool
      {
        auto &event = *(guitar::KeyEvent *)pEvent;
        if (event.Payload.KeyCode == GLFW_KEY_F11 && event.Payload.Action == GLFW_RELEASE)
        {
          Schedule([this] { ToggleFullscreen(); });
          return true;
        }

        return false;
      });

  Events().Register(
      "get_selected_camera", this,
      [this](const guitar::EventBase *pEvent) -> bool
      {
        auto &event = *(guitar::MutableEvent<std::string> *)pEvent;
        event.Payload = m_Cameras[m_SelectedCamera];
        return true;
      });
  Events().Register(
      "select_camera", this,
      [this](const guitar::EventBase *) -> bool
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
      [this](const guitar::EventBase *pEvent) -> bool
      {
        auto &event = *(guitar::ImageEvent *)pEvent;

        if (m_SelectedCamera >= 0 && m_SelectedCamera < static_cast<int>(m_Images.size()))
        {
          const auto &image = m_Images[m_SelectedCamera];
          event.Payload.Width = image.Width;
          event.Payload.Height = image.Height;
          event.Payload.TextureID = (void *)(intptr_t)image.Texture;
          return true;
        }

        return false;
      });

  Events().Register(
      "get_selected_joystick", this,
      [this](const guitar::EventBase *pEvent) -> bool
      {
        const auto &payload = *(guitar::MutableEvent<std::string> *)pEvent;
        if (m_SelectedJoystick < 0)
        {
          payload.Payload = "Keyboard";
          return true;
        }

        auto joysticks = Input().ListJoysticks();
        payload.Payload = joysticks[m_SelectedJoystick];
        return true;
      });
  Events().Register(
      "select_joystick", this,
      [this](const guitar::EventBase *) -> bool
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

  Events().Register(
      "open_main", this,
      [this](const guitar::EventBase *) -> bool
      {
        Schedule([this] { UseLayout("main"); });
        return true;
      });
  Events().Register(
      "open_settings", this,
      [this](const guitar::EventBase *) -> bool
      {
        Schedule([this] { UseLayout("settings"); });
        return true;
      });

  Events().Register(
      "exit", this,
      [this](const guitar::EventBase *) -> bool
      {
        Close();
        return true;
      });
  Events().Register(
      "get_barcodes", this,
      [this](const guitar::EventBase *) -> bool
      {
        for (const auto &[code, count] : m_Barcodes) ImGui::Text("%s: %lu", code.c_str(), count);
        if (m_Barcodes.size() > 0)
        {
          if (ImGui::Button("Export"))
          {
            
	          std::string barcodes;
            for(const auto& pair : m_Barcodes)
            {
              std::string keyStr= pair.first;
              std::string valueStr =std::to_string(pair.second);
              std::string line = keyStr + ": " +valueStr;
              barcodes += line + "\n";
            }
	          std::ofstream ausgabeDatei("N10_Barcode.txt");
	          ausgabeDatei << barcodes;
	          
            ausgabeDatei.close();
          }
        }

        return true;
      });
  Input().CreateAxis(
      "Horizontal",
      {
          { guitar::AxisType_Key, GLFW_KEY_D, false },
          { guitar::AxisType_Key, GLFW_KEY_A, true },
          { guitar::AxisType_Axis, GLFW_GAMEPAD_AXIS_LEFT_X, false },
      });
  Input().CreateAxis(
      "Vertical",
      {
          { guitar::AxisType_Key, GLFW_KEY_W, false },
          { guitar::AxisType_Key, GLFW_KEY_S, true },
          { guitar::AxisType_Axis, GLFW_GAMEPAD_AXIS_LEFT_Y, false },
      });
  Input().CreateAxis(
      "Rotate",
      {
          { guitar::AxisType_Key, GLFW_KEY_Q, false },
          { guitar::AxisType_Key, GLFW_KEY_E, true },
          { guitar::AxisType_Axis, GLFW_GAMEPAD_AXIS_RIGHT_Y, false },
      });
}
