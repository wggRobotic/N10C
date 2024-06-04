#pragma once

#include <N10C/communicator.hpp>
#include <guitar/application.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class Davinci : public guitar::Application
{
public:
  Davinci(int argc, const char **argv);

protected:
  void OnStart() override;

private:
  // ROS
  std::shared_ptr<Communicator> m_Communicator;
  std::thread m_Thread;

  // Cameras
  int m_SelectedCamera = 0;
  const std::vector<std::string> m_Cameras = { "Vorne", "Hinten", "Thermal", "Tiefe" };
};
