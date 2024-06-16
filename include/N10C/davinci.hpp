#pragma once

#include <N10C/communicator.hpp>
#include <guitar/application.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class Davinci : public guitar::Application
{
public:
  Davinci(int argc, const char **argv);

  void SetImage(size_t index, const std::vector<unsigned char> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding, uint8_t is_bigendian);

protected:
  void OnInit(guitar::AppConfig &config) override;
  void OnStart() override;
  void OnFrame() override;
  void OnImGui() override;

private:
  // ROS
  std::shared_ptr<Communicator> m_Communicator;
  std::thread m_Thread;

  // Cameras
  int m_SelectedCamera = 0;
  const std::vector<std::string> m_Cameras = { "Front", "Rear", "Motion" };

  // Joysticks
  int m_SelectedJoystick = -1;

  // Images
  std::vector<guitar::Image> m_Images;
};
