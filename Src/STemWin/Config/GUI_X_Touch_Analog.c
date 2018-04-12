#include "GUI.h"
#include "touch.h"

void GUI_TOUCH_X_ActivateX(void)
{
	TOUCH_Scan();
}
void GUI_TOUCH_X_ActivateY(void)
{
	TOUCH_Scan();
}
int GUI_TOUCH_X_MeasureX(void)
{
	TOUCH_Scan();
	return TouchData.lcdx;
}
int GUI_TOUCH_X_MeasureY(void)
{
	TOUCH_Scan();
	return TouchData.lcdy;
}
