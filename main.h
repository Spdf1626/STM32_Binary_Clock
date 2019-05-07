
#include <stm32f10x_adc.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_pwr.h>
#include <misc.h>
#include <Configuration.h>
#include <stm32f105_functions.h>
#include <sensors.h>




extern unsigned int current_frequency;
extern LED_String LED_Display[5];
extern unsigned char Data_to_running_string[1024];
