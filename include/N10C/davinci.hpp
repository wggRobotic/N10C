#pragma once

#include <guitar/application.hpp>
#include <N10C/communicator.hpp>

class Davinci
    : public guitar::Application
{
public:
    Davinci(int argc, const char **argv);

protected:
    void OnFrame() override;

private:
    std::shared_ptr<Communicator> m_Communicator;
    rclcpp::executors::SingleThreadedExecutor m_Executor;
};
