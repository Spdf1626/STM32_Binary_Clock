

void Task_Manager(void);
void Display_manager(void);
void Running_string(void);
void Clock(void);
void Load_to_slot(unsigned char slot_number, void (*func)(), int presc);
void Running_String(char * string);
void Print_accelerometer_data(void);
extern unsigned char Read_Byte_from_I2C(unsigned char addr, unsigned char reg);
extern void print_byte(char byte);
extern void printcr(char * string);
extern void Read_time_from_RTC(void);
extern void Update_brightness(unsigned char br);
extern unsigned short Get_current_ambient_light(void);


typedef struct
{
	unsigned LED1 : 1;
	unsigned LED2 : 1;
	unsigned LED3 : 1;
	unsigned LED4 : 1;
	unsigned LED5 : 1;
	unsigned LED6 : 1;
	unsigned Brightness : 3;
} LED_String;






