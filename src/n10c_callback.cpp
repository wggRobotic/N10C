#include <N10C/n10c.hpp>

void N10C::BarcodeCallback(const StringConstPtr &msg) { ++m_Barcodes[msg->data]; }

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
