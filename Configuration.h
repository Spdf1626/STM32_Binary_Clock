
/*-----------Common definitions----------*/
#define FPS 100
#define UART_Interface USART1
//#define UART_Interface USART3
#define DISPLAY_BRIGHTNESS 1	// 1>=DISPLAY_BRIGHTNESS<=10
#define Slots 8


#define FLASH_KEY1		0x45670123
#define FLASH_KEY2		0xCDEF89AB
/*---------------------------------------*/

/*--------Accelerometer------------------*/
#define ACCELEROMETER_ADDR 0x38

#define ACCELEROMETER_Who_Am_I        0x0F
#define ACCELEROMETER_CTRL_REG1       0x20
#define ACCELEROMETER_CTRL_REG2       0x21
#define ACCELEROMETER_CTRL_REG3       0x22
#define ACCELEROMETER_STATUS_REG      0x27
#define ACCELEROMETER_FF_WU_CFG_1     0x30
#define ACCELEROMETER_FF_WU_SRC_1     0x31
#define ACCELEROMETER_FF_WU_THS_1     0x32
#define ACCELEROMETER_FF_WU_CFG_2     0x34
#define ACCELEROMETER_FF_WU_SRC_2     0x35
#define ACCELEROMETER_FF_WU_THS_2     0x36
#define ACCELEROMETER_CLICK_CFG       0x38
#define ACCELEROMETER_CLICK_SRC       0x39
#define ACCELEROMETER_CLICK_THSY_X    0x3B
#define ACCELEROMETER_CLICK_THSZ      0x3C
#define ACCELEROMETER_CLICK_TimeLimit 0x3D
#define ACCELEROMETER_CLICK_Latency   0x3E
#define ACCELEROMETER_CLICK_Window    0x3F
/*---------------------------------------*/

/*-----------------RTC-------------------*/
#define RTC_ADDR 0xD0
/*---------------------------------------*/

/*-----------------GPIOs-----------------*/
#define COL_01_ON GPIOA->BSRR = GPIO_Pin_3
#define COL_02_ON GPIOA->BSRR = GPIO_Pin_4
#define COL_03_ON GPIOA->BSRR = GPIO_Pin_5
#define COL_04_ON GPIOA->BSRR = GPIO_Pin_6
#define COL_05_ON GPIOA->BSRR = GPIO_Pin_7
#define COL_06_ON GPIOB->BSRR = GPIO_Pin_0

#define COL_01_OFF GPIOA->BRR = GPIO_Pin_3
#define COL_02_OFF GPIOA->BRR = GPIO_Pin_4
#define COL_03_OFF GPIOA->BRR = GPIO_Pin_5
#define COL_04_OFF GPIOA->BRR = GPIO_Pin_6
#define COL_05_OFF GPIOA->BRR = GPIO_Pin_7
#define COL_06_OFF GPIOB->BRR = GPIO_Pin_0

#define RAW_05_ON GPIOA->BSRR = GPIO_Pin_12
#define RAW_04_ON GPIOA->BSRR = GPIO_Pin_11
#define RAW_03_ON GPIOA->BSRR = GPIO_Pin_8
#define RAW_02_ON GPIOB->BSRR = GPIO_Pin_2
#define RAW_01_ON GPIOB->BSRR = GPIO_Pin_1

#define RAW_05_OFF GPIOA->BRR = GPIO_Pin_12
#define RAW_04_OFF GPIOA->BRR = GPIO_Pin_11
#define RAW_03_OFF GPIOA->BRR = GPIO_Pin_8
#define RAW_02_OFF GPIOB->BRR = GPIO_Pin_2
#define RAW_01_OFF GPIOB->BRR = GPIO_Pin_1
/*----------------------------------------*/

/*-----------------APDS9960---------------*/
#define APDS9960_ADDR       0x72
#define APDS9960_ATIME          0x81
#define APDS9960_CONTROL        0x8F
#define APDS9960_ENABLE         0x80

#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B

#define APDS9960_PON            0x01
#define APDS9960_AEN            0x02
#define APDS9960_PEN            0x04
#define APDS9960_WEN            0x08
#define APSD9960_AIEN           0x10
#define APDS9960_PIEN           0x20
#define APDS9960_GEN            0x40
#define APDS9960_GVALID         0x01

#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

#define DEFAULT_ATIME           219
#define DEFAULT_AGAIN           AGAIN_4X
/*----------------------------------------*/

