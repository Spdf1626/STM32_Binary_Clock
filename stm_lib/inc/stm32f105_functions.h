
#include <Task_manager.h>

void Init_All(void);
void Init_HSE(void);
void Init_HSI(void);
void Init_GPIO(void);
void Init_USART3(void);
void Init_USART1(void);
void Init_UART5(void);
void Init_TIM4(void);
void Init_TIM5(void);
void Init_TIM6(void);
void Init_CAN1(char prescaler);
void Init_I2C1(void);
void Init_accelerometer(void);
void Init_RTC(void);
void Init_APDS9960(void);
void Init_PWD(void);
void Disable_SW(void);
void Disable_JTAG(void);
void Disable_SW(void);
void Read_time_from_RTC(void);
unsigned short Get_current_ambient_light(void);
void Write_time_to_RTC(void);
void Update_brightness(unsigned char br);
unsigned char Read_Byte_from_I2C(unsigned char addr, unsigned char reg);
void Write_Byte_to_I2C(unsigned char addr, unsigned char reg, unsigned char data);
void Flash_erase_page(unsigned int address);
void Write_to_flash(int addr, short data1, short data2);
unsigned int Read_from_flash(int addr);
void print_byte(char byte);
void print(char * string);
void printcr(char * string);
unsigned char Read_accelerometer_ID(void);
void Send_to_CAN1(unsigned int a,unsigned char b,unsigned char c);
void Usart_Handler(void);
void Button_Handler(void);
void CAN_Handler(void);
void Task_Manager(void);
void Deep_sleep(void);
void Init_SPI1(void);
void Test_LEDs(void);
void Delay(unsigned int tiq);
void Delay_TIM(unsigned int ms);




