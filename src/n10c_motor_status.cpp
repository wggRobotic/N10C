#include <N10C/n10c.hpp>

using namespace std::chrono_literals;

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
