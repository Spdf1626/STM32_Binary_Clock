#include <stm32f105_functions.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_pwr.h>
#include <misc.h>
#include <Configuration.h>

unsigned int UART_counter = 0;
char data_from_UART[256];
unsigned char UART_receive_complete = 0;
extern unsigned char current_hours;
extern unsigned char current_minutes;
extern unsigned char current_seconds;
unsigned char string_brightness;
unsigned char string_brightnessX2;
unsigned char string_brightnessX3;
unsigned char string_brightnessX4;
unsigned char string_brightnessX5;
unsigned int current_frequency = 0;
unsigned int delay_counter = 0;

unsigned char time[8] = {0};

extern LED_String LED_Display[5];
RCC_ClocksTypeDef RCC_Clocks;

void Init_All(void)
{
  Init_HSI();
  //Disable_JTAG();
  Init_GPIO();
  Init_USART1();
  Init_I2C1();
  Init_PWD();
  //Init_TIM4();
  //Init_TIM5();
  //Init_TIM6();
  __enable_irq();
  //Init_SPI1();
  printcr("Hello World!");
  printcr(" ");
}

void Init_HSI(void)		// 16 MHz
{
  RCC_DeInit();
  RCC_HSICmd(ENABLE);
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4);
  RCC_PLLCmd(ENABLE);

  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  	// Ждем включения PLL
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);              // Выбираем PLL как источник
  // системного тактирования

  while (RCC_GetSYSCLKSource() != 0x08);                	// Ждем, пока не установится PLL,
                                                          // как источник системного тактирования
  RCC_GetClocksFreq (&RCC_Clocks);
  current_frequency = RCC_Clocks.HCLK_Frequency;
  SysTick_Config(current_frequency / (50*FPS));
}

void Init_GPIO(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,	ENABLE);

  GPIO_InitTypeDef 			GPIO_InitStructure;


  // UART_5
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_10 ;		// RX
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_9 ;		// TX
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3  |
                                  GPIO_Pin_4  |
				  GPIO_Pin_5  |
				  GPIO_Pin_6  |
				  GPIO_Pin_7  |
				  GPIO_Pin_8  |
				  GPIO_Pin_11 |
				  GPIO_Pin_12 ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 ;		// Accelerometer INT1
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 |
				  GPIO_Pin_1 |
				  GPIO_Pin_2 ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // I2C1
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_I2C1(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  I2C_InitTypeDef  I2C_InitStructure;

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_Init(I2C1, &I2C_InitStructure);

  I2C_Cmd(I2C1, ENABLE);
}

void Init_USART1(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  printcr("UART_1 ready");
  printcr(" ");
}

void Init_TIM4(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  TIM_TimeBaseStructInit(&TIMER_InitStructure);
  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 32000-1;
  TIMER_InitStructure.TIM_Period = 1000;
  TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

  /* NVIC Configuration */
  /* Enable the TIM4_IRQn Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  printcr("TIM_4 ready");
}

void Init_TIM5(void)
{
  ;
}

void Init_TIM6(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;

  TIM_TimeBaseStructInit(&TIMER_InitStructure);
  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 16000-1;
  TIMER_InitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInit(TIM6, &TIMER_InitStructure);
  //TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM6, ENABLE);

  //printcr("TIM_6 ready");
}

void Init_PWD(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  PWR_PVDLevelConfig(PWR_PVDLevel_2V9);
  PWR_PVDCmd(ENABLE);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_ClearITPendingBit(EXTI_Line16);
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Update_brightness(unsigned char br)
{
  string_brightness = br;
  string_brightnessX2 = br*2;
  string_brightnessX3 = br*3;
  string_brightnessX4 = br*4;
  string_brightnessX5 = br*5;
}

void I2C_start(void)
{
  //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C1->CR1 |= I2C_CR1_START;	//Generate START
  while (!(I2C1->SR1 & I2C_SR1_SB));	// Wait EVENT5
  //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  (void) I2C1->SR1;
}

void I2C_stop(void)
{
  I2C1->CR1 |= I2C_CR1_STOP;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}
 
void I2C_address_direction(unsigned char address, unsigned char direction)
{
  // Send slave address
  I2C_Send7bitAddress(I2C1, address, direction);

  // Wait for I2C EV6
  // It means that a slave acknowledges his address
  if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(I2C1,
            I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
  else if (direction == I2C_Direction_Receiver)
    { 
        while (!I2C_CheckEvent(I2C1,
            I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}
 
void I2C_transmit(unsigned char byte)
{
  I2C1->DR = byte;
  //I2C_SendData(I2C1, byte);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
 
unsigned char I2C_receive_ack()
{
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  return I2C1->DR;
}
 
unsigned char I2C_receive_nack()
{
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  return I2C1->DR;	//I2C_ReceiveData(I2C1);
}

void Write_Byte_to_I2C(unsigned char addr, unsigned char reg, unsigned char data)
{
  I2C_start();
  I2C_address_direction(addr, I2C_Direction_Transmitter);
  I2C_transmit(reg);
  I2C_transmit(data);
  I2C_stop();
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

unsigned char Read_Byte_from_I2C(unsigned char addr, unsigned char reg)
{
  unsigned char data = 0;
  I2C_start();
  I2C_address_direction(addr, I2C_Direction_Transmitter);
  I2C_transmit(reg);
  I2C_start();
  I2C_address_direction(addr, I2C_Direction_Receiver);
  //I2C1->CR1 |= CR1_ACK_Set;
  while (!(I2C1->SR1 & I2C_SR1_RXNE));
  data = I2C_receive_nack();
  I2C_stop();
  return data;
}

void Deep_sleep(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;	// enable PWR
    SCB->SCR |= SCB_SCR_SLEEPDEEP; 		// sleepdeep enable
    PWR->CR |= PWR_CR_PDDS;				// select Power Down Deepsleep mode
    PWR->CR |= PWR_CR_CWUF ; 			// clear wakeup flag
    __WFE();							// go standby
}

void Disable_SW(void)
{
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

void Disable_JTAG(void)
{
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

void print_byte(char byte)
{
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TXE)));
  USART_SendData(UART_Interface, byte);
  UART_Interface->DR = (byte & (short)0x01FF);
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TC)));
}

void print(char * string)
{
  while(*string)
    {
      while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TXE)));
    	USART_SendData(UART_Interface, *string++);
    }
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TC)));
}

void printcr(char * string)
{
  while(*string)
    {
      while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TXE)));
    	USART_SendData(UART_Interface, *string++);
    }
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TXE)));
  USART_SendData(UART_Interface, 0x0D);
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TXE)));
  USART_SendData(UART_Interface, 0x0A);
  while(!(USART_GetFlagStatus(UART_Interface, USART_FLAG_TC)));
}

int Memcomp(const char *str1, const char *str2, int start)
{
  str1 += start;
  while(*str2)
    {
      if(*str2 == *str1)
        {
          str1++;
          str2++;
        }
      else
        return 0;
    }
  return 1;
}

void Delay(unsigned int tiq)
{
  tiq *= 128;
  for (; tiq!=0; tiq--);
}

void Delay_TIM(unsigned int ms)
{
  delay_counter = 0;
  while(delay_counter < ms);
}

void Set_time()
{
  current_hours   = (10*(time[0]-0x30) + (time[1]-0x30));
  current_minutes = (10*(time[3]-0x30) + (time[4]-0x30));
  current_seconds = (10*(time[6]-0x30) + (time[7]-0x30));
  Write_time_to_RTC();
}

unsigned char Read_data_from_RTC(unsigned char reg)
{
  return(Read_Byte_from_I2C(RTC_ADDR, reg));
}

void Write_time_to_RTC(void)
{
  I2C_start();
  I2C_address_direction(RTC_ADDR, I2C_Direction_Transmitter);
  I2C_transmit(0x00);
  I2C_transmit(((current_seconds/10)*0x10)+(current_seconds%10));
  I2C_transmit(((current_minutes/10)*0x10)+(current_minutes%10));
  I2C_transmit(((current_hours/10)*0x10)+(current_hours%10));
  I2C_stop();
}

void Reset_RTC(void)
{
  I2C_start();
  I2C_address_direction(RTC_ADDR, I2C_Direction_Transmitter);
  I2C_transmit(0x00);
  I2C_transmit(0x00);
  I2C_transmit(0x00);
  I2C_transmit(0x00);
  I2C_stop();
}

void Read_time_from_RTC(void)
{
  unsigned char temp_hours   = 0;
  unsigned char temp_minutes = 0;
  unsigned char temp_seconds = 0;

  I2C_start();
  I2C_address_direction(RTC_ADDR, I2C_Direction_Transmitter);
  I2C_transmit(0x00);
  I2C_start();
  I2C_address_direction(RTC_ADDR, I2C_Direction_Receiver);
  temp_seconds = I2C_receive_ack();
  temp_minutes = I2C_receive_ack();
  temp_hours = I2C_receive_nack();
  I2C_stop();
  current_hours   = 10*(temp_hours>>4) + temp_hours % 0x10;
  current_minutes = 10*(temp_minutes>>4) + temp_minutes % 0x10;
  current_seconds = 10*(temp_seconds>>4) + temp_seconds % 0x10;
}

unsigned char Read_accelerometer_ID(void)
{
	return(Read_Byte_from_I2C(ACCELEROMETER_ADDR, 0x0F));
}

void Init_accelerometer(void)
{
  if(Read_accelerometer_ID() == 0x3B)
    {
      printcr("Accelerometer OK");
      if(!(Read_Byte_from_I2C(ACCELEROMETER_ADDR, 0x20) & 0b01000000))
        Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG1, 0b01100111);// Turn ON accelerometer power
      (void)Read_Byte_from_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_SRC);	// Reset CLICK_SRC register
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG2, 0b00010100);	// Turn ON noise filter
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG3, 0b00000111);	// Interrupt1 by click
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_THSY_X, 0b11111111);	// Threshold for X and Y axes
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_THSZ, 0b00001111);	// Threshold for Z axis
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_CFG, 0b01101010);	// All-axes click and double click
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_TimeLimit, 0b00010000);	// Time_limit
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_Latency, 0b00010000);	// Latency
      Write_Byte_to_I2C(ACCELEROMETER_ADDR, ACCELEROMETER_CLICK_Window, 0b11111110);	// Click_window
    }
   else
     printcr("Accelerometer or I2C not ok");
}

void Init_APDS9960(void)
{
  Write_Byte_to_I2C(APDS9960_ADDR, APDS9960_ATIME, DEFAULT_ATIME);
  Write_Byte_to_I2C(APDS9960_ADDR, APDS9960_CONTROL, DEFAULT_AGAIN);
  Write_Byte_to_I2C(APDS9960_ADDR, APDS9960_ENABLE, (APDS9960_PON | APDS9960_AEN));
}

void Init_RTC(void)
{
  Write_Byte_to_I2C(RTC_ADDR, 0x07, (Read_Byte_from_I2C(RTC_ADDR, 0x07) ^ 0b00100000));
  Delay_TIM(5000);
  if(Read_Byte_from_I2C(RTC_ADDR, 0x07) & 0b00100000)
    printcr("RTC not ok");
  else
    {
      printcr("RTC OK");
      if(Read_Byte_from_I2C(RTC_ADDR, 0x00) & 0b10000000)
        Write_Byte_to_I2C(RTC_ADDR, 0x00, (Read_Byte_from_I2C(RTC_ADDR, 0x00) ^ 0b10000000));
      if(Read_Byte_from_I2C(RTC_ADDR, 0x02) & 0b01000000)
        {
          Write_Byte_to_I2C(RTC_ADDR, 0x02, 0x00);
          printcr("AM/PM error");
        }
      if(Read_Byte_from_I2C(RTC_ADDR, 0x07) & 0b00100000)
        printcr("32 kHz oscillator error");
      else
        {
          printcr("32 kHz oscillator OK");
          Read_time_from_RTC();
          printcr("Time restored");
        }
    }
}

unsigned short Get_current_ambient_light(void)
{
  unsigned char Color_tmpL = Read_Byte_from_I2C(APDS9960_ADDR, APDS9960_CDATAL);
  unsigned char Color_tmpH = Read_Byte_from_I2C(APDS9960_ADDR, APDS9960_CDATAH);

  print_byte(Read_Byte_from_I2C(APDS9960_ADDR, APDS9960_CDATAL));
  print_byte(Read_Byte_from_I2C(APDS9960_ADDR, APDS9960_CDATAH));

  unsigned char Color_Clear = (Color_tmpH << 8) + Color_tmpL;
  return Color_Clear;
}

void Usart_Handler(void)
{
  UART_receive_complete = 0;

  if(Memcomp(data_from_UART, "Help", 0))
    {
      printcr(" ");
      printcr("Set time to hh:mm:ss");
      printcr(" ");
      printcr("Timer blink disable");
      printcr(" ");
      printcr("Echo");
      printcr(" ");
      printcr("Shutdown");
      printcr(" ");
      printcr("Reset");
      printcr(" ");
      printcr("Uptime");
      printcr(" ");
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Read accelerometer ID", 0))
    {
      print_byte(Read_accelerometer_ID());
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Read data from RTC", 0))
    {
      print_byte(Read_data_from_RTC(0x00));
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Notification 1 ON", 0))
    {
      LED_Display[0].LED1 = 1;
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Notification 1 OFF", 0))
    {
      LED_Display[0].LED1 = 0;
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Notification 2 ON", 0))
    {
      LED_Display[4].LED1 = 1;
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Notification 2 OFF", 0))
    {
      LED_Display[4].LED1 = 0;
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Set time to ", 0))
    {
      for(int i=0; i<8; i++)
        time[i] = data_from_UART[i+12];
      Set_time();
      printcr("Time updated");
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Reset RTC", 0))
    {
      Reset_RTC();
      printcr("RTC was reset");
      goto M1;
    }

  else if(Memcomp(data_from_UART, "Shutdown", 0))
    {
      printcr("Power off . . .");
      Deep_sleep();
      goto M1;
    }

  else if(Memcomp(data_from_UART, "Echo", 0))
    {
      char data_to_UART[250];
      for(int i = 0; i < 250; i++)
        data_to_UART[i] = data_from_UART[i+5];
      printcr(data_to_UART);
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Author", 0))
    {
      printcr(" ");
      printcr("----------------------------------------");
      printcr("   Created by ");
      printcr("   Egor B.    ");
      printcr("   spdf1626@gmail.com    ");
      printcr("----------------------------------------");
      printcr(" ");
      goto M1;
    }
  else if(Memcomp(data_from_UART, "Reset", 0))
    {
      printcr("Reset MCU . . .");
      NVIC_SystemReset();
      goto M1;
    }
  else
    {
      printcr("Unknown command");
      printcr("----------------------------------------");
    }

  M1: for(int i = 0; i < 256; i++)
    data_from_UART[i] = 0;
}

void USART3_IRQHandler(void)
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
      data_from_UART[UART_counter] = USART_ReceiveData(USART3);
      UART_counter++;
    }

  if((data_from_UART[UART_counter-2] == 0x0D) && (data_from_UART[UART_counter-1] == 0x0A))
    {
      data_from_UART[UART_counter-2] = 0;
      data_from_UART[UART_counter-1] = 0;
      UART_counter = 0;
      UART_receive_complete = 1;
      Usart_Handler();
    }
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      data_from_UART[UART_counter] = USART1->DR;
      UART_counter++;
    }

  if((data_from_UART[UART_counter-2] == 0x0D) && (data_from_UART[UART_counter-1] == 0x0A))
    {
      data_from_UART[UART_counter-2] = 0;
      data_from_UART[UART_counter-1] = 0;
      UART_counter = 0;
      UART_receive_complete = 1;
      Usart_Handler();
    }
  else if(UART_counter > 255)
    UART_counter = 0;
}

void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

void SysTick_Handler (void)
{
  if (delay_counter < 0xFFFFFFFF)
    delay_counter++;
  else
    delay_counter = 0;

  Task_Manager();
}

void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void PVD_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line16) != RESET)
  {
      if (PWR_GetFlagStatus(PWR_FLAG_PVDO) == RESET)
        PWR_EnterSTANDBYMode();
      else ;

      EXTI_ClearITPendingBit(EXTI_Line16);
  }
}
