#include <N10C/n10c.hpp>

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