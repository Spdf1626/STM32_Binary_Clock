#include <main.h>

int main(void)
{
	Init_All();
	Init_accelerometer();
	Init_RTC();
	Init_APDS9960();
	Update_brightness(DISPLAY_BRIGHTNESS);

	Load_to_slot(1, Display_manager, 1);
	Load_to_slot(2, Clock, 50*FPS);

	/*------------- Prepare WKUP Pin*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	PWR_WakeUpPinCmd(ENABLE);
	/*------------------------------*/

	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);       // Sleep after interrupt
	__WFI();        // Wait for interrupts

    while(1);
}
