
#include <Configuration.h>
#include <Task_manager.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_pwr.h>

LED_String LED_Display[5];

unsigned char Data_to_running_string[1024] = {"12:34"};
unsigned char String_to_run[1024] = {0};
extern unsigned char string_brightness;
extern unsigned char string_brightnessX2;
extern unsigned char string_brightnessX3;
extern unsigned char string_brightnessX4;
extern unsigned char string_brightnessX5;

unsigned char LED_current_string = 0;
unsigned char LED_brightness = 100;
unsigned char frame_counter = 0;
unsigned int string_position = 0;
unsigned char current_hours = 0;
unsigned char current_minutes = 0;
unsigned char current_seconds = 0;
unsigned char prepare_to_standby = 0;
unsigned char go_to_standby = 0;

int task_manager_reg1 = 0;
int Slots_prescaler[Slots][2] = {0};
void (*slot[Slots])();

void Task_Manager(void)
{
  for (unsigned char j=0; j<Slots; j++)
    {
	if(Slots_prescaler[j][0])
	{
		Slots_prescaler[j][1]++;
		if(Slots_prescaler[j][0] == Slots_prescaler[j][1])
		{
			Slots_prescaler[j][1] = 0;
			(*slot[j])();
		}
	}
    }
}

/*-------------------------------------------/
 * slot_number - number of slots
 * *func - function that will be added to slot
 * presc - frequency of function calling
/-------------------------------------------*/
void Load_to_slot(unsigned char slot_number, void (*func)(), int presc)
{
	slot[slot_number] = func;
	Slots_prescaler[slot_number][0]=presc;
}

void Update_Display(void)
{
	RAW_05_OFF;
	RAW_04_OFF;
	RAW_03_OFF;
	RAW_02_OFF;
	RAW_01_OFF;

	if(LED_Display[LED_current_string].LED1)
		COL_01_ON;
	else
		COL_01_OFF;
	if(LED_Display[LED_current_string].LED2)
		COL_02_ON;
	else
		COL_02_OFF;
	if(LED_Display[LED_current_string].LED3)
		COL_03_ON;
	else
		COL_03_OFF;
	if(LED_Display[LED_current_string].LED4)
		COL_04_ON;
	else
		COL_04_OFF;
	if(LED_Display[LED_current_string].LED5)
		COL_05_ON;
	else
		COL_05_OFF;
	if(LED_Display[LED_current_string].LED6)
		COL_06_ON;
	else
		COL_06_OFF;


	if(LED_current_string == 0)
		RAW_01_ON;
	if(LED_current_string == 1)
		RAW_02_ON;
	if(LED_current_string == 2)
		RAW_03_ON;
	if(LED_current_string == 3)
		RAW_04_ON;
	if(LED_current_string == 4)
		RAW_05_ON;
}

void Clear_Display(void)
{
	RAW_05_OFF;
	RAW_04_OFF;
	RAW_03_OFF;
	RAW_02_OFF;
	RAW_01_OFF;
}

void Prepare_LED_array(void)
{
	char temp = 0;
	if(LED_Display[0].LED1)
		temp = 1;
	for(int i=0; i<3; i++)
	{
		LED_Display[i].LED1 = 0;
		LED_Display[i].LED2 = 0;
		LED_Display[i].LED3 = 0;
		LED_Display[i].LED4 = 0;
		LED_Display[i].LED5 = 0;
		LED_Display[i].LED6 = 0;
	}
	if(temp)
		LED_Display[0].LED1 = 1;
	if(current_hours & 0b00000001)
		LED_Display[0].LED6 = 1;
	if(current_hours & 0b00000010)
		LED_Display[0].LED5 = 1;
	if(current_hours & 0b00000100)
		LED_Display[0].LED4 = 1;
	if(current_hours & 0b00001000)
		LED_Display[0].LED3 = 1;
	if(current_hours & 0b00010000)
		LED_Display[0].LED2 = 1;
	if(current_hours & 0b00100000)
		LED_Display[0].LED1 = 1;

	if(current_minutes & 0b00000001)
		LED_Display[1].LED6 = 1;
	if(current_minutes & 0b00000010)
		LED_Display[1].LED5 = 1;
	if(current_minutes & 0b00000100)
		LED_Display[1].LED4 = 1;
	if(current_minutes & 0b00001000)
		LED_Display[1].LED3 = 1;
	if(current_minutes & 0b00010000)
		LED_Display[1].LED2 = 1;
	if(current_minutes & 0b00100000)
		LED_Display[1].LED1 = 1;

	if(current_seconds & 0b00000001)
		LED_Display[2].LED6 = 1;
	if(current_seconds & 0b00000010)
		LED_Display[2].LED5 = 1;
	if(current_seconds & 0b00000100)
		LED_Display[2].LED4 = 1;
	if(current_seconds & 0b00001000)
		LED_Display[2].LED3 = 1;
	if(current_seconds & 0b00010000)
		LED_Display[2].LED2 = 1;
	if(current_seconds & 0b00100000)
		LED_Display[2].LED1 = 1;
}

void Display_manager(void)
{

	if(GPIOA->IDR & GPIO_Pin_0)
	{
		char a = Read_Byte_from_I2C(ACCELEROMETER_ADDR, 0x39);
		if(a & 0b00100000)
		{
			LED_Display[4].LED1 = 0;
			LED_Display[4].LED2 = 0;
			LED_Display[4].LED3 = 0;
			LED_Display[4].LED4 = 0;
			LED_Display[4].LED5 = 0;
			LED_Display[4].LED6 = 0;
			prepare_to_standby ++;
		}
		else if(a & 0b00010000)
			LED_Display[4].LED2 = 1;
		else if(a & 0b00001000)
		{
			LED_Display[4].LED3 = 1;
			if(string_brightness > 1)
				string_brightness--;
			Update_brightness(string_brightness);
		}
		else if(a & 0b00000100)
			LED_Display[4].LED4 = 1;
		else if(a & 0b00000010)
		{
			LED_Display[4].LED5 = 1;
			if(string_brightness < 10)
				string_brightness++;
			Update_brightness(string_brightness);
		}
		else if(a & 0b00000001)
			LED_Display[4].LED6 = 1;
	}

	if(frame_counter == 0)
	{
		LED_current_string = 0;
		Update_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter == string_brightness)
	{
		LED_current_string = 1;
		Update_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter == string_brightnessX2)
	{
		LED_current_string = 2;
		Update_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter == string_brightnessX3)
	{
		LED_current_string = 3;
		Update_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter == string_brightnessX4)
	{
		LED_current_string = 4;
		Update_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter == string_brightnessX5)
	{
		Clear_Display();
		frame_counter++;
		return;
	}
	else if(frame_counter >= 50)
	{
		Clear_Display();
		frame_counter = 0;
		return;
	}
	else
		frame_counter++;
}

void Print_accelerometer_data(void)
{
	print_byte(Read_Byte_from_I2C(ACCELEROMETER_ADDR, 0x2D));
}

void Clock(void)
{
  Read_time_from_RTC();
  Prepare_LED_array();
  //(void) Get_current_ambient_light();
  if(prepare_to_standby && (go_to_standby < 3))
    {
      prepare_to_standby = 0;
      go_to_standby++;
      if(go_to_standby == 3)
        PWR_EnterSTANDBYMode();
      return;
    }
  if(prepare_to_standby == 0)
    go_to_standby = 0;
}
