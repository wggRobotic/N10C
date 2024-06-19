#pragma once

#include <guitar/application.hpp>

class GUI : public guitar::Application
{
public:
  GUI(int argc, const char **argv);

protected:
  void OnStart() override;
  void OnFrame() override;

private:
};
