Please analyse the below code and report any issues

#include <avr/io.h>
#define F_CPU 8000000
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/cpufunc.h> 

#ifndef Global_Declaration_H
#define Global_Declaration_H

#define MAJOR_REVISION						4
#define MINOR_REVISION						0
#define PATCH_REVISION						1
#define EEPROM_MAX_PAGE                     236					//index of 237th page

#define EEPROM_HIGHEST_VALUE_PER_PAGE		900000
#define MAX_RX_BUFFER_SIZE					150
#define MAX_MODBUS_BUFFER_SIZE				150
#define Modbus_Stop_Interval				29
#define Four_Crore 							40000000

//Modbus Commands
#define READ_HOLDING_REGISTERS				0x03
#define WRITE_SINGLE_REGISTER				0x06
#define WRITE_MULTIPLE_REGISTERS			0x10

//NETWORK ADDRESS
#define NETWORK_ID_EEPROM_ADDRESS 			0x00

//Leakage LPM
#define Leakage_LPM							0.3

//SENSOR SIZES
#define DN25								25
#define DN32								32
#define DN100								100
#define DN0									0					// Testing Purpose with no calibration curve

//GP30 Constants
//#define STD_AM_VAL							400
#define Delay_Counter						200
#define US_AM_UPD							0
#define US_AMC_UPD							1

//Duct Module
//#define DM_Network_Address 				0x01

//Master ID
//#define Master_Network_Address 			0x02

//Factory_Unique_Number
#define Factory_Unique_Number_Address		0x04		//12 Bytes

//Pulse/Litre Value ,in EEPROM Table
#define Pulse_Per_Litre_Address 			0x10

//Pulse/LPM Value ,in EEPROM Table
//#define Pulse_Per_LPM_Address 			0x11

//Calibration Address
#define Sensor_Calibration_Value_Address	0x12

//Leakage Cutoff Value ,in EEPROM Table
//#define Leakeage_Cutoff_Value_Address 	0x13

//OpenTap Cutoff Value ,in EEPROM Table
//#define OpenTap_Cutoff_Value_Address 		0x14

//Leakage Time in min
#define Leakage_Time_Address 				0x15 

//Leakage Time in min
#define Open_Tap_Time_Address 				0x17

//Calibration Volume
//#define Calibration_Volume_Address		0x1A

//Total_Sensor_Read_Address
//#define Total_Sensor_Read_Address 		0x1C

//EEPROM_Page_Number_Address
#define EEPROM_Page_Number_Address			0x1D

#define Error_Pulse_Threshold_Address		0x20

// Min Valid Consumption
#define Minimum_Valid_Consumption_Address 	0x2C

// Minimum flow pulse
#define Minimum_Flow_Pulse_Address 			0x2D

//Alarm mode address
#define Alarm_State_Address 				0x2E

//Dust_Flow_Disable_Address
#define Dust_Flow_Disable_Address 			0x2F

//CUMLATIVE STARTING EEPROM ADDRESS
#define CUMULATIVE_VALUE_START_LOC_Address 	(0x4C)

#define Sensor_Size_Address					0x21

#define AM_Tolerance_Address				0x22

#define Validation_Flow_Secs_Count_Address	0x23

#define Low_LPM_Pulse_Count_Address			0x24

#define Neg_Flow_Threshold_Address			0x25

#define Maximum_Flow_Pulse_Address			0x26					// 2 Bytes

#define LPM_RANGE_START_LOC_ADDRESS			0x28					// 4 Bytes

#define EVENT_TIME_RANGE_START_LOC_ADDRESS	0x30					// 4 Bytes

#define LPM_LOG_START_LOC_ADDRESS			0x34					// 10 Bytes

#define EVENT_LOG_START_LOC_ADDRESS			0x3E					// 10 Bytes

#define AM_BREACH_START_LOC_ADDRESS			0x48					// 2 Bytes


//LPM array should always be increasing and should not have equal or same value
const float Calibration_Curve_25mm[] PROGMEM		= {0.8934f, 0.8965f, 0.922f, 0.937f, 0.949f, 0.964f, 0.977f, 0.9813f, 0.9924f, 0.992f};
const float Calibration_LPM_25mm[] PROGMEM	    	= {0.93f, 1.49f, 3.25f, 5.34f, 10.54f, 24.7f, 49.03f, 67.94f, 83.98f, 90.74f};

const float Calibration_Curve_32mm[] PROGMEM		= {1.47f, 1.45f, 1.475f, 1.485f, 1.587f, 1.766f, 1.843f};
const float Calibration_LPM_32mm[] PROGMEM	    	= {1.08f, 2.3f, 40.3f, 80.71f, 105.01f, 117.94f, 120.0f};

//const float Calibration_Curve_Default[] PROGMEM		= {0.92f, 0.93f, 0.95f, 0.97f, 0.98f, 1.0f, 1.04f, 1.05f, 1.07f, 1.08f};
//const float Calibration_LPM_Default[] PROGMEM	    = {1.5f, 3.0f, 10.5f, 30.0f, 60.0f, 120.0f, 217.5f, 322.5f, 375.0f, 431.25f};

unsigned char Sensor_Network_ID = 0;
unsigned char Sensor_Calibration_Value = 0;
unsigned char EEPROM_Page_Number = 0;

unsigned char Alarm = 0;
unsigned char Alarm_Mode_Flag = 1;
unsigned char Minimum_Flow_Pulse = 0;
unsigned int  Maximum_Flow_Pulse = 0;
unsigned char Dust_Flow_Disable = 0;

unsigned char Error_Pin_State = 0;
unsigned char Flow_Pin_State = 1;
unsigned char Short_Term_Error = 0;
unsigned char Long_Term_Error = 0;
unsigned char Long_Term_Error_Counter = 0;

unsigned char Error_Pulse_Threshold = 0;

volatile unsigned char RX_Complete = 0;
unsigned char USART0_Modbus_Timer = 0;
unsigned char USART0_RX_Count = 0;
unsigned char Modbus_Request_Buffer_Count = 0;

unsigned char Leakage_Time_In_Min = 0;
unsigned char Open_Tap_Time_In_Min = 0;

unsigned char One_Min_Counter = 0;
unsigned char Minimumn_Valid_Consumption = 0;

unsigned char Modbus_Network_ID;
unsigned char Modbus_Function_Code;
unsigned char Modbus_Register_AddressH;
unsigned char Modbus_Register_AddressL;
unsigned char Modbus_Register_CountH;
unsigned char Modbus_Register_CountL;

unsigned char USART0_Receive_Buffer [MAX_RX_BUFFER_SIZE];
unsigned char Modbus_Response_Buffer [MAX_MODBUS_BUFFER_SIZE];
unsigned char Modbus_Request_Buffer [MAX_MODBUS_BUFFER_SIZE];

unsigned char Factory_Unique_Number[13];

unsigned char Device_ID[3];
unsigned char Unique_ID[4];

volatile unsigned char UNIO_READ_ERROR = 0;

unsigned int  One_Sec_Timer = 0;
unsigned int  Ten_Sec_Timer = 0;

unsigned int  Calibration_Range[] = {575, 500, 430, 290, 160, 80, 40, 14, 4, 2};
unsigned int  Calibration_Values[10];
unsigned int  Error_Pulse_Queue[3];

unsigned long int Flow_Pulse_Count = 0;
unsigned long int Raw_Pulse_Count = 0;
unsigned long int Raw_Pulse_Count_Processed = 0;
unsigned long int Raw_Pulse_Holder = 0;
unsigned long int Error_Pulse_Count = 0;
unsigned long int Raw_Error_Pulse_Count = 0;

unsigned long int Cumulative_Raw_Pulse_Count = 0;
unsigned long int Cumulative_Error_Pulse_Count = 0;
unsigned long int Cumulative_Flow_Pulse_Count = 0;

unsigned long int Cumulative_Consumption = 0;
unsigned long int Previous_Consumption = 0;

unsigned long int Flow_Pulse_Held = 0;
unsigned long int Flow_Pulse_On_Error = 0;
unsigned long int Flow_Pulse_On_Clear = 0;

unsigned long int Leakage_Timer = 0;
unsigned long int Open_Tap_Timer = 0;

unsigned char Sensor_Size = 0;
unsigned char Neg_Flow_Threshold = 0;

float  Pulse_Per_Litre = 0;
double Consumption_Double = 0;
double Consumption_Last_Min = 0;

float AMC_High     = 0;
float AMC_Low      = 0;
float AM_Up        = 0;
float AMVup        = 0;
float Vcal         = 350;
float Min_AM	   = 700;

unsigned char AM_Tolerance = 0;

#define INDEX_CMD_READ_SRR_ERR_FLAG	0
#define INDEX_CMD_CLEAR_FLAGS		46
#define INDEX_RES_SRR_FEP_STF_0		8
#define INDEX_RES_AM_UP				22
#define INDEX_RES_AMC_HIGH			26
#define INDEX_RES_AMC_LOW			42
#define INDEX_CMD_RAM_R_FW_ERR_FLAG 52
#define INDEX_CMD_READ_RAM		    58
#define INDEX_DATA_READ_RAM		    60


unsigned char SPI1_Request_Buffer[] = {0x7A, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x7A, 0xE2, 0xFF, 0xFF, 0xFF, 0xFF, 0x7A, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x5A, 0xDD, 0x00, 0x00, 0x00, 0x07, 0x7A, 0x27, 0xFF, 0xFF, 0xFF, 0xFF, 0x7A, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x8D, 0x5A, 0x27, 0x00, 0x00, 0x00, 0x07};
unsigned char SPI1_SSN_BUFFER[]     = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};

unsigned char SPI1_Response_Buffer[sizeof (SPI1_Request_Buffer)];

unsigned char SPI1_Buffer_Index = 0;
unsigned char SPI_TXN_Complete  = 0;

unsigned int GP30_INT_Count     = 0;
unsigned int Neg_Flow_Reg_Fraction = 0;

unsigned char Flow_State 					= 0;
unsigned char Valid_Flow_Counts 			= 0;
unsigned char Validation_Flow_Secs_Count 	= 0;
unsigned char Low_LPM_Pulse_Count 			= 0;
unsigned char High_LPM_Instant				= 0;

unsigned char USL_BOARD						= 0;

unsigned char Test_Variable					= 0;

//unsigned char GP30_Data_Array[5];

unsigned long int Negative_Increment_Count	= 0;
unsigned long int Positive_Increment_Count	= 0;


unsigned char LPM_Ranges[] 			= {1, 5, 15, 50};			//In 8 secs cycle
unsigned char Event_Time_Ranges[] 	= {1, 8, 40, 120};			//Multiples of 8 secs

unsigned int LPM_Log_Count[5];
unsigned int Event_Log_Count[5];

unsigned long int LPM_Log_Temp_Holder[5];
unsigned long int Event_Log_Temp_Holder[5];

unsigned int AM_Breach_Count = 0;
unsigned long int AM_Breach_Holder = 0;



//Size Constants
typedef struct{    
	unsigned char pulse_per_liter;    
	unsigned char am_threshold;
	unsigned char minimum_flow_pulse;
	unsigned char negative_flow_threshold;
	unsigned char validation_flow_secs;
} Sensor;

Sensor Sensor_Configs[5] = {{40, 150, 0, 20, 3}, {10, 125, 0, 30, 3}, {10, 100, 2, 40, 3}, {1, 125, 0, 50, 3}, {40, 1, 0, 240, 1}};


#endif


//MCU -> ATMEGA32M1AU
#ifndef Support_Functions_C
#define Support_Functions_C


void Write_EEPROM (unsigned int eprom_address, unsigned char data2write)
{
	while (EECR & (1 << EEMWE));				//Wait for previous EEPROM Operation to complete

	EEAR = eprom_address;						//Location to write
	EEDR = data2write;							//Data to be written

	cli();										//Clear Global Interrupt
	EECR |= (1 << EEMWE);						//EEPROM Master Write Enable
	EECR |= (1 << EEWE);						//EEPROM Write Enable
	while (EECR & (1 << EEMWE));
	sei();										//Enable Global Interrupt
}


unsigned char Read_EEPROM (unsigned int eprom_address)
{
	while (EECR & (1 << EEMWE));				//Wait for previous EEPROM Operation to complete
	EEAR  = eprom_address;						//Location to read from
	EECR |= (1 << EERE);						//Set Read enable bit to start read operation
	return EEDR;
}


void Save_Cumulative_Consumption (void)
{
	Write_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4), (Cumulative_Consumption >> 24));
	Write_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 1, (Cumulative_Consumption >> 16));
	Write_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 2, (Cumulative_Consumption >> 8));
	Write_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 3, (Cumulative_Consumption));
}


void Check_EEPROM_Page_Save_Cumulative_Consumption (void)
{
	unsigned long highest_value_for_page;
	
	highest_value_for_page = (EEPROM_HIGHEST_VALUE_PER_PAGE + (EEPROM_Page_Number * EEPROM_HIGHEST_VALUE_PER_PAGE));
	
	if (Cumulative_Consumption >= highest_value_for_page)
	{
		if (EEPROM_Page_Number < EEPROM_MAX_PAGE)											//Total 243 pages available in Attiny1634 ((1024 bytes - 48 (Start Add))/4)
		{
			EEPROM_Page_Number++;
			Write_EEPROM (EEPROM_Page_Number_Address, EEPROM_Page_Number);
		}
	}

	Save_Cumulative_Consumption ();
}


unsigned char Find_Sensor_Config_Index (unsigned char size)
{
	switch (size)
	{
		case (0):
			return (4);
		case (25):
			return (0);
		case (32):
			return (1);
		case (40):
		case (50):
			return (2);
		case (100):
			return (3);
		default:
			return (0);
	}
}


void Initialize(void)
{
	unsigned char tmpval, tmpval2;
	unsigned long tmplong;
	unsigned int tmpint;
	unsigned char sensor_config_index;
	
	Sensor_Network_ID = Read_EEPROM (NETWORK_ID_EEPROM_ADDRESS);
	if ((Sensor_Network_ID < 1) || (Sensor_Network_ID > 254)) Sensor_Network_ID = 200;
	
	//for (tmpval = 0; tmpval < 12; tmpval++)
	//{
	//	Factory_Unique_Number[tmpval] = Read_EEPROM (Factory_Unique_Number_Address + tmpval);
	//}
	
	Sensor_Calibration_Value = Read_EEPROM (Sensor_Calibration_Value_Address);
	if ((Sensor_Calibration_Value < 10) || (Sensor_Calibration_Value > 240)) Sensor_Calibration_Value = 100;
	
	Leakage_Time_In_Min =  Read_EEPROM (Leakage_Time_Address);
	if ((Leakage_Time_In_Min < 1) || (Leakage_Time_In_Min > 240)) Leakage_Time_In_Min = 30;
	
	Open_Tap_Time_In_Min = Read_EEPROM (Open_Tap_Time_Address);
	if ((Open_Tap_Time_In_Min < 1) || (Open_Tap_Time_In_Min > 240)) Open_Tap_Time_In_Min = 30;
	
	Sensor_Size = Read_EEPROM (Sensor_Size_Address);
	if (Sensor_Size > 240) Sensor_Size = 25;
	
	sensor_config_index = Find_Sensor_Config_Index (Sensor_Size);
	
	tmpval = Read_EEPROM (Pulse_Per_Litre_Address);
	if ((tmpval < 1) || (tmpval > 240)) tmpval = Sensor_Configs[sensor_config_index].pulse_per_liter;
	Pulse_Per_Litre = tmpval;

	Minimum_Flow_Pulse = Read_EEPROM (Minimum_Flow_Pulse_Address);
	if ((Minimum_Flow_Pulse < 1) || (Minimum_Flow_Pulse > 240)) Minimum_Flow_Pulse = Sensor_Configs[sensor_config_index].minimum_flow_pulse;
	
	tmpint = Read_EEPROM (Maximum_Flow_Pulse_Address);
	tmpint <<= 8;
	tmpint |= Read_EEPROM (Maximum_Flow_Pulse_Address + 1);
	
	Maximum_Flow_Pulse = tmpint;
	if ((Maximum_Flow_Pulse < 1) || (Maximum_Flow_Pulse > 1050)) Maximum_Flow_Pulse = 1040;

	Alarm_Mode_Flag = Read_EEPROM (Alarm_State_Address);
	if (Alarm_Mode_Flag != 2) Alarm_Mode_Flag = 1;

	Dust_Flow_Disable = Read_EEPROM (Dust_Flow_Disable_Address);
	if (Dust_Flow_Disable != 1) Dust_Flow_Disable = 0;

	Minimumn_Valid_Consumption = Read_EEPROM (Minimum_Valid_Consumption_Address);
	if ((Minimumn_Valid_Consumption < 1) || (Minimumn_Valid_Consumption > 240)) Minimumn_Valid_Consumption = 0;

	EEPROM_Page_Number = Read_EEPROM (EEPROM_Page_Number_Address);
	if (EEPROM_Page_Number > EEPROM_MAX_PAGE)											//Total 52 pages available in Attiny1634 ((256 bytes - 48 (Start Add))/4)
	{
		EEPROM_Page_Number = 0;
		Write_EEPROM (EEPROM_Page_Number_Address, EEPROM_Page_Number);
	}
	
	tmplong = Read_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4));
	tmplong <<= 8;
	tmplong |= Read_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 1);
	tmplong <<= 8;
	tmplong |= Read_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 2);
	tmplong <<= 8;
	tmplong |= Read_EEPROM (CUMULATIVE_VALUE_START_LOC_Address + (EEPROM_Page_Number * 4) + 3);
	
	if ((tmplong >= EEPROM_HIGHEST_VALUE_PER_PAGE) && (EEPROM_Page_Number == 0))
	{
		tmplong = 0;
	}

	Cumulative_Consumption 	= tmplong;
	Previous_Consumption    = tmplong;
	Consumption_Double  	= (double) Cumulative_Consumption;
	
	Error_Pulse_Threshold = Read_EEPROM (Error_Pulse_Threshold_Address);
	if ((Error_Pulse_Threshold > 240) || (Error_Pulse_Threshold < 5)) Error_Pulse_Threshold = 40;
	
	
	AM_Tolerance = Read_EEPROM (AM_Tolerance_Address);
	if (AM_Tolerance > 240) AM_Tolerance = Sensor_Configs[sensor_config_index].am_threshold;
	
	Validation_Flow_Secs_Count = Read_EEPROM (Validation_Flow_Secs_Count_Address);
	if (Validation_Flow_Secs_Count > 7) Validation_Flow_Secs_Count = Sensor_Configs[sensor_config_index].validation_flow_secs;
	
	Low_LPM_Pulse_Count = Read_EEPROM (Low_LPM_Pulse_Count_Address);
	if (Low_LPM_Pulse_Count > 5) 	Low_LPM_Pulse_Count = 2;
	
	Neg_Flow_Threshold = Read_EEPROM (Neg_Flow_Threshold_Address);
	if ((Neg_Flow_Threshold > 240) || (Neg_Flow_Threshold < 2)) Neg_Flow_Threshold = Sensor_Configs[sensor_config_index].negative_flow_threshold;
	
	tmpint = Read_EEPROM (AM_BREACH_START_LOC_ADDRESS);
	tmpint <<= 8;
	tmpint |= Read_EEPROM (AM_BREACH_START_LOC_ADDRESS + 1);
	if (tmpint > 65001) tmpint = 0;
	AM_Breach_Count = tmpint;
	
	if (tmpint) AM_Breach_Holder = (tmpint - 1) * 100;
	else AM_Breach_Holder = 0;
	
	
	for (tmpval = 0; tmpval < 4; tmpval++)
	{
		tmpval2 = Read_EEPROM (LPM_RANGE_START_LOC_ADDRESS + tmpval);
		if ((tmpval2 < 240) && (tmpval2 > 0)) LPM_Ranges[tmpval] = tmpval2;
		
		tmpval2 = Read_EEPROM (EVENT_TIME_RANGE_START_LOC_ADDRESS + tmpval);
		if ((tmpval2 < 240) && (tmpval2 > 0)) Event_Time_Ranges[tmpval] = tmpval2;
	}
	
	for (tmpval = 0; tmpval < 5; tmpval++)
	{
		tmpint = Read_EEPROM (LPM_LOG_START_LOC_ADDRESS + (tmpval * 2));
		tmpint <<= 8;
		tmpint |= Read_EEPROM (LPM_LOG_START_LOC_ADDRESS + (tmpval * 2) + 1);
		if (tmpint > 65001) tmpint = 0;
		
		LPM_Log_Count[tmpval] = tmpint;
		if (tmpint) LPM_Log_Temp_Holder[tmpval] = (tmpint - 1) * 100;
		else LPM_Log_Temp_Holder[tmpval] = 0;
		
		tmpint = Read_EEPROM (EVENT_LOG_START_LOC_ADDRESS + (tmpval * 2));
		tmpint <<= 8;
		tmpint |= Read_EEPROM (EVENT_LOG_START_LOC_ADDRESS + (tmpval * 2) + 1);
		if (tmpint > 65001) tmpint = 0;
		
		Event_Log_Count[tmpval] = tmpint;
		if (tmpint) Event_Log_Temp_Holder[tmpval] = (tmpint - 1) * 100;
		else Event_Log_Temp_Holder[tmpval] = 0;
	}
}



void PortB_Init (void)
{
	//Configuring pin as input and setting that pin as high, enables the internal pullup resistor
	/* Port B Configuration
	Port B.0 - Input 	- MISO1 (not used yet)
	Port B.1 - Output 	- MOSI1 (not used yet)
	Port B.2 - Input  	- GPIO_1	Error Pin from GP30
	Port B.3 - Input 	- Test Point 6
	Port B.4 - Input 	- Test Point 12
	Port B.5 - Input 	- GPIO_2 SSN
	Port B.6 - Input  	- INTN
	Port B.7 - Output 	- SCK1 (not used yet)
	*/
	
	DDRB  |= (1 << 1);							// MOSI is set Output
	DDRB  |= (1 << 7);							// SCK is set output
	
	
	PORTB |= (1 << 2);							//Error Pin is set high
	
}

void PortC_Init (void)
{
	//Configuring pin as input and setting that pin as high, enables the internal pullup resistor
	/* Port C Configuration
	Port C.0 - Output	- Manual Load Resistor On/Off
	Port C.1 - Output	- DE (Direction Enable RS485)
	Port C.2 - Input 	- Test Point 2
	Port C.3 - Input	- PC3 (unused)
	Port C.4 - Input	- ADC_IP
	Port C.5 - Input	- Test Point 7
	Port C.6 - Input	- Test Point 8
	Port C.7 - Input	- NC
	*/
	
	DDRC  |= (1 << 0);							//Manual Load Resistor On/Off
	DDRC  |= (1 << 1);							//Direction Enable for RS485
	DDRC  |= (1 << 3);							//SSN_GPIO2 Chip selection is set as Output
	
	PORTC &= ~(1 << 0); 						//Manual Load is set Low
	PORTC &= ~(1 << 1); 						//Direction Enable is set Low (RX)
	PORTC |= (1 << 3);							//SSN_GPIO2 is set high initially
	PORTC |= (1 << 6);							//INTN_DIR
}


void PortD_Init (void)
{
	//Configuring pin as input and setting that pin as high, enables the internal pullup resistor
	/* Port D Configuration
	Port D.0 - Input	- Test Point 10
	Port D.1 - Input	- Test Point 9
	Port D.2 - Input 	- MISO (unused)
	Port D.3 - Output	- TXD_MOSI
	Port D.4 - Output	- RXD_SCK
	Port D.5 - Input	- Test Point 11
	Port D.6 - Input	- GPIO_0 Flow Pin from GP30
	Port D.7 - Input/Output	- UNIO SCIO Pin (unique ID)
	*/
	
	DDRD  |= (1 << 3);							// SS as Output pin
	
	
	PORTD |= (1 << 6);							// Flow Pin is set high
	//PORTD |= (1 << 2);						// INTN_DIR
}


void USART_Init (void)
{
	//USART 0 Enabled for RS485 Communication
	/*----USART 0 Configuration----*/
	/* Baud Rate :9600  */
	
	LINCR 	 = 0x0F;							//Enabled LIN/UART Controller for Full Duplex UART 8 Data Bits, No Parity and Listening Mode OFF
	LINENIR  = 0x01;							//Rx Performed Interrupt enabled 
	LINENIR |= (1 << 3);						//UART Err Interrupt enable
	
	//LINBTR |= (1 << 7);						//Bit Timing Re synchronization is Disabled.
	//LINBTR	= 0x90;							//Bit Timing Re synchronization is Disabled. Bit Timing is 16
	LINBRRH	= 0x00;
	LINBRRL = 0x19;								//8Mhz, 9600 Baud rate

}


void SPI1_Init (void)
{
	/* Enabling SPI Hardware, Master Mode, SPI Interrupts*/
	/* Setting the Clock fclkio/128 */
	
	SPCR  = 0xD7;								//(1 << SPE) | (1 << MSTR) | (1 << SPIE) | (0 << CPOL) | (1 << CPHA) | (1 << SPR1) | (1 << SPR0)
	
}


void USART0_Enable_TX (void)
{
	
	PORTC |= (1 <<  1); 						//Direction Enable is set High (TX)		
}


void USART0_Enable_RX (void)
{
	
	PORTC &= ~(1 <<  1); 						//Direction Enable is set Low (RX)				   
}


void TIMER1_Init (void)
{
	//Timer 1 Enabled for 1ms Compare match Interrupt
	TCCR1A  = 0x00;
	TCCR1B  = 0x01;								//Pre-scalar Value 1 (No prescaling, 8MHZ)
	TCCR1C  = 0x00;
	
	TCNT1H  = 0x00;
	TCNT1L  = 0x00;

	OCR1AH  = 0x1F;								//0x1F40=8000 -> 1ms
	OCR1AL  = 0x40;
	
	OCR1BH  = 0x00;
	OCR1BL  = 0x00;

	TIMSK1   = 0x00;
	TIMSK1  |= (1 << OCIE1A);					//Compare Match Interrupt Enabled for Timer 1 at channel A for 1ms.
}


void Power_Reduction_Init (void)
{
	//Power Reduction Register - Writing 1 to PRR bit shuts down the corresponding peripheral

	PRR &= ~(1 << PRADC);						//No Power Reduction for ADC
	PRR &= ~(1 << PRLIN);						//No Power Reduction for LIN/UART
	PRR &= ~(1 << PRTIM1);						//No Power Reduction for TIMER1
	PRR &= ~(1 << PRSPI);						//No Power Reduction for SPI

	PRR |= (1 << PRTIM0);						//Shut Down Timer/Counter 0
	PRR |= (1 << PRCAN);						//Shut Down CAN Periphereal
	PRR |= (1 << PRPSC);						//Shut Down PSC Peripheral
}


void External_Interrupt_Init (void)
{
	//Global External Interrupt Enabling
	EICRA = 0x0E;								//Leading edge of INT1(error) and Falling edge of Flow (INT0) generates intrrupts
	EIMSK = 0x03;								//INT1 and INT0 interrupts are enabled
	
	//Pin change Interrupt for INTN_DIR
	PCICR  = (1 << PCIE1);						// Enabling PCIE1 for the PCINT14 pin INTN_DIR
	PCMSK1 = (1 << PCINT14);					// Enabling PCINT14 Vector
}


unsigned int Read_ADC (void)
{
	ADMUX 	= 0xC8;								//ADC8 - PC4, External Vref on AREF pin
	//ADCSRB |= (1 << AREFEN);					// External VREF Enable
	_delay_ms(5);
	
	ADCSRA  = (1 << ADEN);						//ADC Enable
	ADCSRA |= 0x06;								//Setting clock prescalar to 64
	ADCSRA |= (1 << ADSC);						//ADC Start Conversion
	
	while ((ADCSRA & (1 << ADIF)) == 0);		//Wait for the AD conversion to complete
	ADCSRA |= (1 << ADIF);						//Writing 1 to ADIF clears this flag (ADIF)

	ADCSRA  = 0;								//ADC Disable
	return ADCW;								//Return the ADC Value (10 bits)
}


void USART0_Transmit (unsigned char data2send)
{
	LINDAT = data2send;
	while ((LINSIR & (1 << LTXOK)) == 0);
}


unsigned int Modbuscrc16(unsigned char* modbusframe,unsigned char Length)
{
	unsigned int crc_register=0xFFFF,crc_temp;//new
	unsigned char ival=0;
	unsigned char right_shift_count=0;//new
	
	ival=0;
	
	while(ival<Length)
	{
		crc_register=(crc_register^modbusframe[ival]);
		do
		{
			crc_temp=crc_register;
			crc_register=crc_register>>1;
			if((crc_temp&0x0001)==0x0001)
		{    crc_register=(crc_register^0xA001);   }
			right_shift_count++;
		}while(right_shift_count<8);
		right_shift_count=0;
		ival++;
	}
	
	crc_temp=crc_register;
	crc_register=crc_register>>8;
	crc_temp=crc_temp<<8;
	crc_register=(crc_register|crc_temp);
	return crc_register;
}


unsigned char Check_CRC (void)
{
	unsigned int  tmp_crc, tmp_received_crc;
	
	if (Modbus_Request_Buffer_Count < 5) return (0);

	tmp_received_crc  = Modbus_Request_Buffer[Modbus_Request_Buffer_Count - 2] << 8;
	tmp_received_crc |= Modbus_Request_Buffer[Modbus_Request_Buffer_Count - 1];
	
	if ((tmp_received_crc == 0xFFFF) || (tmp_received_crc == 0x0000)) return (0);
	
	tmp_crc = Modbuscrc16 (&Modbus_Request_Buffer[0], (Modbus_Request_Buffer_Count - 2));
	if (tmp_crc == tmp_received_crc) return (1); else return (0);
}


void Send_Modbus_Response (unsigned char noofbytes)
{
	unsigned char tmpc;

	USART0_Enable_TX ();							//Tx is enabled; RX is disabled
	_delay_ms (1);

	for (tmpc = 0; tmpc < noofbytes; tmpc++)
	{
		USART0_Transmit (Modbus_Response_Buffer [tmpc]);
	}

	_delay_ms (1);
	USART0_Enable_RX ();							//Rx is enabled; Tx is disabled
}


void Send_Consumption_Data (void)
{
	unsigned long tmplong;
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID;							//ID
	Modbus_Response_Buffer[1] = 0x03;										//Function code
	Modbus_Response_Buffer[2] = 0x08;										//Byte count

	//Consumption
	if ((unsigned long) Consumption_Double <= Four_Crore)
	{
		tmplong = (Consumption_Double * 100);
	}
	else
	{
		tmplong = Consumption_Double;
	}

	Modbus_Response_Buffer[3] = tmplong >> 24;
	Modbus_Response_Buffer[4] = tmplong >> 16;
	Modbus_Response_Buffer[5] = tmplong >> 8;
	Modbus_Response_Buffer[6] = tmplong;

	Modbus_Response_Buffer[7] = 0;
	Modbus_Response_Buffer[8] = Alarm;

	Modbus_Response_Buffer[9]  = 0;
	Modbus_Response_Buffer[10] = Short_Term_Error;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 11);
	
	Modbus_Response_Buffer[11] = tmp_crc >> 8;
	Modbus_Response_Buffer[12] = tmp_crc;
	
	Send_Modbus_Response (13);
}


void Send_Consumption_Configuration_Data (void)
{
	unsigned long tmplong;
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID; 						//ID
	Modbus_Response_Buffer[1] = 0x03; 									//Function code
	Modbus_Response_Buffer[2] = 0x1E; 									//Byte count

	tmplong = Consumption_Double;
	Modbus_Response_Buffer[3] = tmplong >> 24;
	Modbus_Response_Buffer[4] = tmplong >> 16;
	Modbus_Response_Buffer[5] = tmplong >> 8;
	Modbus_Response_Buffer[6] = tmplong;

	memcpy (&Modbus_Response_Buffer[7], &Factory_Unique_Number[0], 12);	//Factory_Unique_Number

	Modbus_Response_Buffer[19] = Pulse_Per_Litre;
	Modbus_Response_Buffer[20] = 0;

	Modbus_Response_Buffer[21] = 0;
	Modbus_Response_Buffer[22] = Sensor_Calibration_Value;

	Modbus_Response_Buffer[23] = 0;
	Modbus_Response_Buffer[24] = Alarm;

	Modbus_Response_Buffer[25] = 0;										//Leakage_Time_In_Min >> 8;
	Modbus_Response_Buffer[26] = Leakage_Time_In_Min;

	Modbus_Response_Buffer[27] = 0;										//Open_Tap_Time_In_Min >> 8;
	Modbus_Response_Buffer[28] = Open_Tap_Time_In_Min;

	Modbus_Response_Buffer[29] = 0;
	Modbus_Response_Buffer[30] = 0;

	Modbus_Response_Buffer[31] = 0;
	Modbus_Response_Buffer[32] = 0;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 33);
	
	Modbus_Response_Buffer[33] = tmp_crc >> 8;
	Modbus_Response_Buffer[34] = tmp_crc;
	
	Send_Modbus_Response (35);
}


void Send_Configuration_Data (void)
{
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID; 				//ID
	Modbus_Response_Buffer[1] = 0x03; 							//Function code
	Modbus_Response_Buffer[2] = 12; 							//Byte count
	
	Modbus_Response_Buffer[3] = Dust_Flow_Disable;
	Modbus_Response_Buffer[4] = 0;

	Modbus_Response_Buffer[5] = Pulse_Per_Litre;
	Modbus_Response_Buffer[6] = Sensor_Calibration_Value;

	Modbus_Response_Buffer[7] = Leakage_Time_In_Min;
	Modbus_Response_Buffer[8] = 0;
	
	Modbus_Response_Buffer[9]  = Open_Tap_Time_In_Min;
	Modbus_Response_Buffer[10] = 0;
	
	Modbus_Response_Buffer[11] = 0;
	Modbus_Response_Buffer[12] = 0;
	
	Modbus_Response_Buffer[13] = Flow_Pulse_Count;
	Modbus_Response_Buffer[14] = 0;
		
	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0],15);
	
	Modbus_Response_Buffer[15] = tmp_crc >> 8;
	Modbus_Response_Buffer[16] = tmp_crc;
	
	Send_Modbus_Response (17);
}


void Send_Configuration2_Data (void)
{
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID; 				//ID
	Modbus_Response_Buffer[1] = 0x03; 							//Function code
	Modbus_Response_Buffer[2] = 12; 							//Byte count
	
	Modbus_Response_Buffer[3] = Minimum_Flow_Pulse;
	Modbus_Response_Buffer[4] = Alarm_Mode_Flag;

	Modbus_Response_Buffer[5] = Dust_Flow_Disable;
	Modbus_Response_Buffer[6] = Minimumn_Valid_Consumption;

	Modbus_Response_Buffer[7] = 0;
	Modbus_Response_Buffer[8] = 0;
	
	Modbus_Response_Buffer[9]  = 0;
	Modbus_Response_Buffer[10] = 0;
	
	Modbus_Response_Buffer[11] = 0;
	Modbus_Response_Buffer[12] = 0;
	
	Modbus_Response_Buffer[13] = 0;
	Modbus_Response_Buffer[14] = 0;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0],15);
	
	Modbus_Response_Buffer[15] = tmp_crc >> 8;
	Modbus_Response_Buffer[16] = tmp_crc;
	
	Send_Modbus_Response (17);
}

void Send_Cumulative_Pulse_Info (void)
{
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID; 				//ID
	Modbus_Response_Buffer[1] = 0x03; 							//Function code
	Modbus_Response_Buffer[2] = 12; 								//Byte count
	
	Modbus_Response_Buffer[3] = Cumulative_Raw_Pulse_Count >> 24;
	Modbus_Response_Buffer[4] = Cumulative_Raw_Pulse_Count >> 16;
	Modbus_Response_Buffer[5] = Cumulative_Raw_Pulse_Count >> 8;
	Modbus_Response_Buffer[6] = Cumulative_Raw_Pulse_Count;

	Modbus_Response_Buffer[7]  = Cumulative_Error_Pulse_Count >> 24;
	Modbus_Response_Buffer[8]  = Cumulative_Error_Pulse_Count >> 16;
	Modbus_Response_Buffer[9]  = Cumulative_Error_Pulse_Count >> 8;
	Modbus_Response_Buffer[10] = Cumulative_Error_Pulse_Count;
	
	Modbus_Response_Buffer[11]  = Cumulative_Flow_Pulse_Count >> 24;
	Modbus_Response_Buffer[12]  = Cumulative_Flow_Pulse_Count >> 16;
	Modbus_Response_Buffer[13]  = Cumulative_Flow_Pulse_Count >> 8;
	Modbus_Response_Buffer[14]  = Cumulative_Flow_Pulse_Count;
	
	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0],15);
	
	Modbus_Response_Buffer[15] = tmp_crc >> 8;
	Modbus_Response_Buffer[16] = tmp_crc;
	
	Send_Modbus_Response (17);
}


void Send_Debug_Info (void)
{
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Sensor_Network_ID; 				//ID
	Modbus_Response_Buffer[1] = 0x03; 							//Function code
	Modbus_Response_Buffer[2] = 16; 							//Byte count
	
	Modbus_Response_Buffer[3] = Flow_Pulse_Count >> 24;
	Modbus_Response_Buffer[4] = Flow_Pulse_Count >> 16;
	Modbus_Response_Buffer[5] = Flow_Pulse_Count >> 8;
	Modbus_Response_Buffer[6] = Flow_Pulse_Count;

	Modbus_Response_Buffer[7]  = Error_Pulse_Count >> 24;
	Modbus_Response_Buffer[8]  = Error_Pulse_Count >> 16;
	Modbus_Response_Buffer[9]  = Error_Pulse_Count >> 8;
	Modbus_Response_Buffer[10] = Error_Pulse_Count;
	
	Modbus_Response_Buffer[11] = Raw_Pulse_Count >> 24;
	Modbus_Response_Buffer[12] = Raw_Pulse_Count >> 16;
	Modbus_Response_Buffer[13] = Raw_Pulse_Count >> 8;
	Modbus_Response_Buffer[14] = Raw_Pulse_Count;

	Modbus_Response_Buffer[15] = Raw_Error_Pulse_Count >> 24;
	Modbus_Response_Buffer[16] = Raw_Error_Pulse_Count >> 16;
	Modbus_Response_Buffer[17] = Raw_Error_Pulse_Count >> 8;
	Modbus_Response_Buffer[18] = Raw_Error_Pulse_Count;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0],19);
	
	Modbus_Response_Buffer[19] = tmp_crc >> 8;
	Modbus_Response_Buffer[20] = tmp_crc;
	
	Send_Modbus_Response (21);
}


void Send_Revision_No (void)
{
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;							//ID
	Modbus_Response_Buffer[1] = 0x03;										//Function code
	Modbus_Response_Buffer[2] = 0x04;										//Byte count

	Modbus_Response_Buffer[3] = MAJOR_REVISION;
	Modbus_Response_Buffer[4] = MINOR_REVISION;
	Modbus_Response_Buffer[5] = PATCH_REVISION;
	Modbus_Response_Buffer[6] = 0;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 7);
	
	Modbus_Response_Buffer[7] = tmp_crc >> 8;
	Modbus_Response_Buffer[8] = tmp_crc;
	
	Send_Modbus_Response (9);
}


void Send_Response_NetworkID (void)
{
	unsigned int  tmp_crc = 0;

	Modbus_Response_Buffer[0] = Modbus_Request_Buffer[0];
	Modbus_Response_Buffer[1] = 0x10; 							//Function code
	Modbus_Response_Buffer[2] = 0x31;
	Modbus_Response_Buffer[3] = 0x01;
	Modbus_Response_Buffer[4] = 0x00;
	Modbus_Response_Buffer[5] = 0x07;

	Modbus_Response_Buffer[6] = Modbus_Request_Buffer[19];		//As per protocol this should not be sent
	Modbus_Response_Buffer[7] = Sensor_Network_ID;				//As per protocol this should not be sent
	
	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 8);
	
	Modbus_Response_Buffer[8] = tmp_crc >> 8;
	Modbus_Response_Buffer[9] = tmp_crc;

	Send_Modbus_Response (10);
}


void Send_Response_Calibration (void)
{
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x10;
	Modbus_Response_Buffer[2] = 0x51;
	Modbus_Response_Buffer[3] = 0x01;
	Modbus_Response_Buffer[4] = 0x00;
	Modbus_Response_Buffer[5] = 0x03;
	Modbus_Response_Buffer[6] = 0x01;								//As per protocol this should not be sent

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 7);
	
	Modbus_Response_Buffer[7] = tmp_crc >> 8;
	Modbus_Response_Buffer[8] = tmp_crc;

	Send_Modbus_Response (9);
}


void Send_Single_Register (uint16_t value)
{
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x03;
	Modbus_Response_Buffer[2] = 0x02;
	Modbus_Response_Buffer[3] = value >> 8;
	Modbus_Response_Buffer[4] = value;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 5);
	
	Modbus_Response_Buffer[5] = tmp_crc >> 8;
	Modbus_Response_Buffer[6] = tmp_crc;

	Send_Modbus_Response (7);
}


void Send_Flow_Variations ()
{
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x03;
	Modbus_Response_Buffer[2] = 0x08;
	
	
	Modbus_Response_Buffer[3] = Positive_Increment_Count >> 24;
	Modbus_Response_Buffer[4] = Positive_Increment_Count >> 16;
	Modbus_Response_Buffer[5] = Positive_Increment_Count >> 8;
	Modbus_Response_Buffer[6] = Positive_Increment_Count;

	Modbus_Response_Buffer[7] = Negative_Increment_Count >> 24;
	Modbus_Response_Buffer[8] = Negative_Increment_Count >> 16;
	Modbus_Response_Buffer[9] = Negative_Increment_Count >> 8;
	Modbus_Response_Buffer[10] = Negative_Increment_Count;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 11);
	
	Modbus_Response_Buffer[11] = tmp_crc >> 8;
	Modbus_Response_Buffer[12] = tmp_crc;

	Send_Modbus_Response (13);
}


void Send_Single_Register_Direct (unsigned char val)
{
	Modbus_Response_Buffer[0] = val;
	Send_Modbus_Response (1);
}

void Send_LPM_Holder_Array ()
{
	unsigned char tmpval = 0;
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x03;
	Modbus_Response_Buffer[2] = 20;

	for (tmpval = 0; tmpval < 5; tmpval++)
	{
		Modbus_Response_Buffer[(tmpval * 4) + 3] = LPM_Log_Temp_Holder[tmpval] >> 24;
		Modbus_Response_Buffer[(tmpval * 4) + 4] = LPM_Log_Temp_Holder[tmpval] >> 16;
		Modbus_Response_Buffer[(tmpval * 4) + 5] = LPM_Log_Temp_Holder[tmpval] >> 8;
		Modbus_Response_Buffer[(tmpval * 4) + 6] = LPM_Log_Temp_Holder[tmpval];
	}
	
	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 23);
	
	Modbus_Response_Buffer[23] = tmp_crc >> 8;
	Modbus_Response_Buffer[24] = tmp_crc;

	Send_Modbus_Response (25);
}

void Send_Event_Holder_Array ()
{
	unsigned char tmpval = 0;
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x03;
	Modbus_Response_Buffer[2] = 20;

	for (tmpval = 0; tmpval < 5; tmpval++)
	{
		Modbus_Response_Buffer[(tmpval * 4) + 3] = Event_Log_Temp_Holder[tmpval] >> 24;
		Modbus_Response_Buffer[(tmpval * 4) + 4] = Event_Log_Temp_Holder[tmpval] >> 16;
		Modbus_Response_Buffer[(tmpval * 4) + 5] = Event_Log_Temp_Holder[tmpval] >> 8;
		Modbus_Response_Buffer[(tmpval * 4) + 6] = Event_Log_Temp_Holder[tmpval];
	}
	
	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 23);
	
	Modbus_Response_Buffer[23] = tmp_crc >> 8;
	Modbus_Response_Buffer[24] = tmp_crc;

	Send_Modbus_Response (25);
}

void Send_AM_Breach_Holder_Array ()
{
	unsigned int  tmp_crc = 0;
	
	Modbus_Response_Buffer[0] = Sensor_Network_ID;
	Modbus_Response_Buffer[1] = 0x03;
	Modbus_Response_Buffer[2] = 4;
	
	Modbus_Response_Buffer[3] = AM_Breach_Holder >> 24;
	Modbus_Response_Buffer[4] = AM_Breach_Holder >> 16;
	Modbus_Response_Buffer[5] = AM_Breach_Holder >> 8;
	Modbus_Response_Buffer[6] = AM_Breach_Holder;

	tmp_crc = Modbuscrc16 (&Modbus_Response_Buffer[0], 7);
	
	Modbus_Response_Buffer[7] = tmp_crc >> 8;
	Modbus_Response_Buffer[8] = tmp_crc;

	Send_Modbus_Response (9);
}

/**
void Send_GP30_Values_Direct (void)
{
	Modbus_Response_Buffer[0] = 0x89;
	Modbus_Response_Buffer[1] = 0xAB;
	Modbus_Response_Buffer[2] = 0xCD;
	Modbus_Response_Buffer[3] = SPI1_Buffer_Index;
	Modbus_Response_Buffer[4] = SPCR;
	Modbus_Response_Buffer[5] = GP30_INT_Count;
	Modbus_Response_Buffer[6] = Test_Variable;
	Modbus_Response_Buffer[7] = DDRB;
	Modbus_Response_Buffer[8] = DDRC;
	Modbus_Response_Buffer[9] = DDRD;
	
	//memcpy (&Modbus_Response_Buffer[6], &SPI1_Response_Buffer[0], sizeof (SPI1_Request_Buffer));
	Send_Modbus_Response ((10));
}
**/
/**
unsigned char SPI1_Transceive (unsigned char data2send)
{
	unsigned char temp;
	SPDR = data2send;
	while(!(SPSR & (1<<SPIF)));
	temp = SPDR;
	return (temp);
}
**/

void SCIO_Input_Mode (void)
{
	//Port D.7 - UNIO SCIO Pin (unique ID)
	
	DDRD  &= ~(1 << 7); 		//Port D.7 input
	PORTD |= (1 << 7); 			//SCIO is set High, Pullup enabled
}


void SCIO_Output_Mode (void)
{
	//Port D.7 - UNIO SCIO Pin (unique ID)
	
	DDRD  |= (1 << 7); 			//Port D.7 output
}


void SCIO_Output (unsigned char outval)
{
	//Port D.7 - UNIO SCIO Pin (unique ID)
	
	if (outval)
	{
		PORTD |= (1 << 7); 			//SCK Pin is set High
	}
	else
	{
		PORTD &= ~(1 << 7); 		//SCK Pin is set Low
	}
}


unsigned char SCIO_Get_State (void)
{
	//Port D.7 - UNIO SCIO Pin (unique ID)
	
	if ((PIND & (1 << 7)) == 0) return (0); else return (1);
}


void UNIO_POR_Header (void)
{
	SCIO_Output_Mode ();
	
	SCIO_Output (0);
	_delay_us (20);
	
	SCIO_Output (1);
	_delay_ms (1);
	SCIO_Output (0);
	_delay_us (20);
}


void UNIO_Header (void)
{
	SCIO_Output_Mode ();

	SCIO_Output (1);
	_delay_us (40);
	SCIO_Output (0);
	_delay_us (20);
}


void UNIO_Master_ACK (void)
{
	SCIO_Output_Mode ();

	SCIO_Output (0);
	_delay_us (32);

	SCIO_Output (1);
	_delay_us (32);
}

void UNIO_Master_NACK (void)
{
	SCIO_Output_Mode ();

	SCIO_Output (1);
	_delay_us (32);

	SCIO_Output (0);
	_delay_us (32);
}


unsigned char UNIO_Slave_ACK (void)
{
	unsigned char tmpa;
	
	SCIO_Input_Mode ();

	tmpa = 30;
	while ((SCIO_Get_State () == 1) &&  (tmpa > 0))		//Wait SCIO to go Low
	{
		tmpa--;
	}
	
	if (tmpa == 0) return (0);

	_delay_us (64);										//Check next state after 1 bit time
	
	if (SCIO_Get_State ()) return (1); else return (0);
}

unsigned char UNIO_Slave_NACK (void)
{
	unsigned char tmpa;
	
	SCIO_Input_Mode ();

	tmpa = 30;
	while ((SCIO_Get_State () == 0) &&  (tmpa > 0))		//Wait SCIO to go High
	{
		tmpa--;
	}
	
	if (tmpa == 0) return (0);

	_delay_us (64);										//Check next state after 1 bit time
	
	if (SCIO_Get_State ()) return (1); else return (0);
}


void UNIO_Write_Bit (unsigned char val2write)
{
	if (val2write > 0)					//1 = 0 -> 1
	{
		SCIO_Output (0);
		_delay_us (32);

		SCIO_Output (1);
		_delay_us (32);
	}
	else								//0 = 1 -> 0
	{
		SCIO_Output (1);
		_delay_us (32);

		SCIO_Output (0);
		_delay_us (32);
	}
}


void UNIO_Write_Byte (unsigned char val2write)
{
	unsigned char tmpa;
	
	SCIO_Output_Mode ();
	for (tmpa = 0; tmpa < 8; tmpa++)
	{
		UNIO_Write_Bit (val2write & (1 << 7));
		val2write <<= 1;
	}
}


unsigned char UNIO_Read_Byte (void)
{
	unsigned char tmpa, tmpb, tmpc;
	
	//UNIO_READ_ERROR = 0;
	tmpc = 0;
	
	SCIO_Input_Mode ();
	for (tmpa = 0; tmpa < 8; tmpa++)
	{
		tmpc <<= 1;
		
		_delay_us (16);
		if (SCIO_Get_State () == 0)
		{
			_delay_us (32);
			if (SCIO_Get_State () == 0)
			{
				UNIO_READ_ERROR = 1;
				return (0);					//Error
			}
			else
			{
				tmpb = 1;					//1 = 0 -> 1
			}
		}
		else
		{
			_delay_us (32);
			if (SCIO_Get_State () == 1)
			{
				UNIO_READ_ERROR = 1;
				return (0);					//Error
			}
			else
			{
				tmpb = 0;					//0 = 1 -> 0
			}
		}
		
		if (tmpb > 0) tmpc |= 0x01;
		_delay_us (16);
	}
	return (tmpc);
}


unsigned char UNIO_Read_Status_Register (void)
{
	unsigned char tmpa;
	
	cli ();
	UNIO_POR_Header ();
	
	UNIO_Write_Byte (0x55);
	UNIO_Master_ACK ();
	UNIO_Slave_NACK ();

	UNIO_Write_Byte (0xA0);
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();

	UNIO_Write_Byte (0x05);
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();
	
	tmpa = UNIO_Read_Byte ();
	UNIO_Master_NACK ();
	UNIO_Slave_ACK ();
	sei ();
	
	return (tmpa);
}


unsigned char UNIO_Read_EEPROM (unsigned int addr)
{
	unsigned char tmpa;
	
	cli ();
	UNIO_Header ();
	
	UNIO_Write_Byte (0x55);
	UNIO_Master_ACK ();
	UNIO_Slave_NACK ();

	UNIO_Write_Byte (0xA0);
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();

	UNIO_Write_Byte (0x03);
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();

	UNIO_Write_Byte (((addr >> 8) & 0xFF));
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();

	UNIO_Write_Byte ((addr & 0xFF));
	UNIO_Master_ACK ();
	UNIO_Slave_ACK ();
	
	tmpa = UNIO_Read_Byte ();
	UNIO_Master_NACK ();
	UNIO_Slave_ACK ();
	sei ();
	
	return (tmpa);
	
}


void UNIO_Read_Unique_ID (void)
{
	UNIO_READ_ERROR = 0;
	
	Device_ID[0] = UNIO_Read_Status_Register ();
	Device_ID[1] = UNIO_Read_EEPROM (0xFA);
	Device_ID[2] = UNIO_Read_EEPROM (0xFB);
	
	Unique_ID[0] = UNIO_Read_EEPROM (0xFC);
	Unique_ID[1] = UNIO_Read_EEPROM (0xFD);
	Unique_ID[2] = UNIO_Read_EEPROM (0xFE);
	Unique_ID[3] = UNIO_Read_EEPROM (0xFF);
	
	if (UNIO_READ_ERROR) memset (Unique_ID, 0xFF, 4);
}


float Get_LPM (unsigned long int pulses)
{
	float lpm = 0;												//Maximum Possible accuracy is less than 6 digits in case of float. So using double.
	
	lpm = (60.0 * pulses) / (8 * Pulse_Per_Litre);
	return lpm;
}


float GetCalibrationValue (float lpm)
{
	unsigned char tmp = 0;
	float lpm_diff, prev_calib , prev_lpm, curr_calib, curr_lpm;
	
	
	prev_calib 	= 0;
	prev_lpm 	= 0;
	curr_calib	= 1.0f;
	
	
	switch (Sensor_Size)
	{
		case DN25:
			for (tmp = 0; tmp < (sizeof(Calibration_LPM_25mm) / sizeof(Calibration_LPM_25mm[0])); tmp++)
			{
				curr_lpm 	= pgm_read_float(&Calibration_LPM_25mm[tmp]);
				curr_calib	= pgm_read_float(&Calibration_Curve_25mm[tmp]);
				
				if (tmp == 0) prev_calib = curr_calib;
				
				if (lpm <= curr_lpm)
				{
					lpm_diff = (lpm - prev_lpm) / (curr_lpm - prev_lpm);
					lpm_diff = (lpm_diff * fabs(curr_calib - prev_calib));
					
					if (curr_calib > prev_calib)	return (prev_calib + lpm_diff); else return (prev_calib - lpm_diff);
				}
				prev_calib 	= curr_calib;
				prev_lpm 	= curr_lpm;
			}
			break;
			
		case DN32:
			for (tmp = 0; tmp < (sizeof(Calibration_LPM_32mm) / sizeof(Calibration_LPM_32mm[0])); tmp++)
			{
				curr_lpm 	= pgm_read_float(&Calibration_LPM_32mm[tmp]);
				curr_calib	= pgm_read_float(&Calibration_Curve_32mm[tmp]);
				
				if (tmp == 0) prev_calib = curr_calib;
				
				if (lpm <= curr_lpm)
				{
					lpm_diff = (lpm - prev_lpm) / (curr_lpm - prev_lpm);
					lpm_diff = (lpm_diff * fabs(curr_calib - prev_calib));
					
					if (curr_calib > prev_calib)	return (prev_calib + lpm_diff); else return (prev_calib - lpm_diff);
				}
				prev_calib 	= curr_calib;
				prev_lpm 	= curr_lpm;
			}
			break;
		
		case DN100:
			return 0.76f;
			break;
		
		case DN0:
			return 1.0f;
			break;
			
		default:
			return 1.0f;
			break;

	}
	return curr_calib;																									//Return the last array element if out of range
}


void Enable_GP30 (void)
{
	PORTC &= ~(1 <<  PORTC3);
	_NOP ();
	_NOP ();
	_NOP ();
	_NOP ();
}


void Disable_GP30 (void)
{
	PORTC |= (1 <<  PORTC3);
	_NOP ();
	_NOP ();
	_NOP ();
	_NOP ();
}

/**
void Clear_Flags_GP30 (void)
{
	
	Enable_GP30 ();
	
	SPI1_Transceive (0x5A);
	SPI1_Transceive (0xDD);																		//SRR_IRQ_FLAG (Interrupt Flags)
	SPI1_Transceive (0x00);
	SPI1_Transceive (0x00);
	SPI1_Transceive (0x00);
	SPI1_Transceive (0x07);
	
	Disable_GP30 ();

}
**/

void Clear_Flags_GP30 (void)
{
	
	/** 
	The last command in the SPI1_Request_Buffer is Clear Flags.
	Initiating SPI transfer with corresponding index.
	Rest are handled in the SPI1 Interrupt
	**/
	
	Enable_GP30 ();
	
	SPI1_Buffer_Index = INDEX_CMD_CLEAR_FLAGS;
	SPDR 			  = SPI1_Request_Buffer[SPI1_Buffer_Index];	
}


float Fixed_Point2Float (unsigned char tmparr[], int intbits)
{
	float tmpresult;
	unsigned int tmpc;

	tmpc   = tmparr[2];
	tmpc <<= 8;
	tmpc  |= tmparr[3];
	
	tmpresult  = tmpc;
	tmpresult /= pow(2, intbits);

	tmpc   = tmparr[0];
	tmpc <<= 8;
	tmpc  |= tmparr[1];
	tmpresult += tmpc;

	return (tmpresult);
}


unsigned char Get_GP30_Values (void)
{
	
	unsigned int temp_neg_reg_fraction = 0;
	
	if ((SPI1_Response_Buffer[INDEX_RES_SRR_FEP_STF_0] & (1 << US_AM_UPD)) > 0)
	{
		AM_Up  = Fixed_Point2Float (&SPI1_Response_Buffer[INDEX_RES_AM_UP], 16);
		AM_Up *= 250;
	}
	else
	{
		return (0);
	}
	
	if ((SPI1_Response_Buffer[INDEX_RES_SRR_FEP_STF_0] & (1 << US_AMC_UPD)) > 0)
	{
		AMC_High  = Fixed_Point2Float (&SPI1_Response_Buffer[INDEX_RES_AMC_HIGH], 16);
		AMC_High *= 250;
		AMC_Low  = Fixed_Point2Float (&SPI1_Response_Buffer[INDEX_RES_AMC_LOW], 16);
		AMC_Low *= 250;
	}
	
	temp_neg_reg_fraction   = SPI1_Response_Buffer[INDEX_DATA_READ_RAM];
	temp_neg_reg_fraction <<= 8;
	temp_neg_reg_fraction  += SPI1_Response_Buffer[(INDEX_DATA_READ_RAM + 1)];
	
	if (temp_neg_reg_fraction < Neg_Flow_Reg_Fraction)	Negative_Increment_Count += (Neg_Flow_Reg_Fraction - temp_neg_reg_fraction);
	else if (temp_neg_reg_fraction > Neg_Flow_Reg_Fraction) Positive_Increment_Count += (temp_neg_reg_fraction - Neg_Flow_Reg_Fraction);
	
	//Updating the latest in the Neg_Flow_Reg_Fraction after calculation
	Neg_Flow_Reg_Fraction	= temp_neg_reg_fraction;
	
	memset(SPI1_Response_Buffer, 0, sizeof(SPI1_Response_Buffer));	
	return (1);
}



void Check_And_Save_LPM_Log (unsigned char lpm_index)
{
	if (LPM_Log_Temp_Holder[lpm_index - 1] < (6500000)) LPM_Log_Temp_Holder[lpm_index - 1]++;
	
	if (((LPM_Log_Temp_Holder[lpm_index - 1] / 100) + 1) > LPM_Log_Count[lpm_index - 1])
	{
		LPM_Log_Count[lpm_index - 1] = (LPM_Log_Temp_Holder[lpm_index - 1] / 100) + 1;
		
		Write_EEPROM (LPM_LOG_START_LOC_ADDRESS + ((lpm_index - 1) * 2), (LPM_Log_Count[lpm_index - 1] >> 8));
		Write_EEPROM (LPM_LOG_START_LOC_ADDRESS + ((lpm_index - 1) * 2) + 1, LPM_Log_Count[lpm_index - 1]);
	}
	
}


void Check_And_Save_Event_Log (unsigned char event_counter)
{
	unsigned char tmpval, curr_event_range;
	
	for (tmpval = 0; tmpval < 4; tmpval++)
	{
		if (event_counter <= Event_Time_Ranges[tmpval])
		{
			curr_event_range = tmpval;
			break;
		}
		if (tmpval == 3)
		{
			curr_event_range = 4;
		}
	}
	
	if (Event_Log_Temp_Holder[curr_event_range] < 6500000) Event_Log_Temp_Holder[curr_event_range]++;
	
	
	if (((Event_Log_Temp_Holder[curr_event_range] / 100) + 1) > Event_Log_Count[curr_event_range])
	{
		Event_Log_Count[curr_event_range] = (Event_Log_Temp_Holder[curr_event_range] / 100) + 1;
		
		Write_EEPROM (EVENT_LOG_START_LOC_ADDRESS + ((curr_event_range) * 2), (Event_Log_Count[curr_event_range] >> 8));
		Write_EEPROM (EVENT_LOG_START_LOC_ADDRESS + ((curr_event_range) * 2) + 1, Event_Log_Count[curr_event_range]);
	}	
}


void Log_LPM_Event_Count (float lpm)
{
	static unsigned char LPM_Index = 0, Event_Counter_8_sec = 0;
	unsigned char tmpval, curr_lpm_range;
	
	if (Flow_Pulse_Count > 0)
	{
		Event_Counter_8_sec++;
		if (Event_Counter_8_sec > 240)	Event_Counter_8_sec = 240;

		for (tmpval = 0; tmpval < 4; tmpval++)
		{
			if (lpm <= ((float) LPM_Ranges[tmpval]))
			{
				curr_lpm_range = tmpval + 1;
				break;
			}
			if (tmpval == 3)
			{
				curr_lpm_range = 5;
			}
		}
		
		if (curr_lpm_range != LPM_Index)
		{
			Check_And_Save_LPM_Log (LPM_Index);
		}
		
		LPM_Index = curr_lpm_range;
	}
	else
	{
		if (LPM_Index != 0) Check_And_Save_LPM_Log (LPM_Index);
		
		if (Event_Counter_8_sec != 0) Check_And_Save_Event_Log (Event_Counter_8_sec);
		
		//Resetting Event counter and previous LPM holder after processing
		Event_Counter_8_sec = 0;
		LPM_Index 		= 0;
	}
}




/**
unsigned char Get_GP30_Values (void)
{
	
	Enable_GP30 ();
	
	SPI1_Transceive (0x7A);
	SPI1_Transceive (0xE1);																		//SRR_IRQ_FLAG (Interrupt Flags)
	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);
	
	SPI1_Transceive (0x7A);
	SPI1_Transceive (0xE2);																		//SRR_IRQ_FLAG (Interrupt Flags)
	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);
	
	Disable_GP30 ();
	
	if ((GP30_Data_Array[0] & (1 << US_AM_UPD)) == 0)
	{
		return (0);
	}
	
	Enable_GP30 ();
	
	SPI1_Transceive (0x7A);
	SPI1_Transceive (0x80);
	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);
	AM_Up  = Fixed_Point2Float (GP30_Data_Array, 16);
	AM_Up *= 250;

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);
	AMC_High  = Fixed_Point2Float (GP30_Data_Array, 16);
	AMC_High *= 250;

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);

	GP30_Data_Array[0] = SPI1_Transceive (0xFF);
	GP30_Data_Array[1] = SPI1_Transceive (0xFF);
	GP30_Data_Array[2] = SPI1_Transceive (0xFF);
	GP30_Data_Array[3] = SPI1_Transceive (0xFF);
	AMC_Low  = Fixed_Point2Float (GP30_Data_Array, 16);
	AMC_Low *= 250;
	
	Disable_GP30 ();
	return (1);
}
**/
#endif


const volatile char __attribute__ ((section (".fini1"))) FW_INFO[] = "WFS101";		//Place this info at the (almost) end of the program

ISR (TIMER1_COMPA_vect)												//Timer 1 Enabled for 1ms
{
	TCNT1H = 0x00;
	TCNT1L = 0x00;

	//Check if UART receiption is complete
	if (USART0_Modbus_Timer > 0) USART0_Modbus_Timer++; 			//Counting only if the modbus reception is started

	if (USART0_Modbus_Timer >= Modbus_Stop_Interval)
	{
		USART0_Modbus_Timer = 0;									//Resetting the timer on receive complete
		RX_Complete         = 1;
	}

	One_Sec_Timer++;
	
	return;
}


ISR (LIN_TC_vect)
{
	unsigned char tmpdata;
	
	tmpdata = LINDAT;												//Must Read LINDAT irrespective of buffer availability
	
	if (USART0_RX_Count < MAX_RX_BUFFER_SIZE)
	{
		USART0_Receive_Buffer[USART0_RX_Count++] = tmpdata;
	}
	
	USART0_Modbus_Timer = 1;										//Reset Modbus timer if data is received
	return;
}


ISR (LIN_ERR_vect)
{
	//err_counter++;
	LINSIR	|= (1 << 3);
}


ISR (INT0_vect)
{
	Raw_Pulse_Count++;
}


ISR (INT1_vect)
{
	Raw_Error_Pulse_Count++;
}


ISR (PCINT1_vect)
{
	if((PINC & (1 << 6)) == 0)
	{
		SPI1_Buffer_Index 	= INDEX_CMD_READ_SRR_ERR_FLAG;
		SPI_TXN_Complete  	= 0;
		
		Enable_GP30 ();
		
		SPDR 				= SPI1_Request_Buffer[SPI1_Buffer_Index];
		GP30_INT_Count++;														//Debug Purpose
	}
}

ISR (SPI_STC_vect)
{
	//SPI Transmission Complete
	unsigned char temp;
	
	temp = SPDR;
	
	if ((++SPI1_Buffer_Index) < sizeof(SPI1_Request_Buffer))
	{
		if (SPI1_SSN_BUFFER[SPI1_Buffer_Index] == 1)							//If this byte is 1, Chip selection should be applied new (New Txn)
		{
			Disable_GP30 ();
			Enable_GP30 ();
		}
		SPDR = SPI1_Request_Buffer[SPI1_Buffer_Index];
	}
	else
	{
		Disable_GP30 ();
		SPI_TXN_Complete = 1;
	}
	
	SPI1_Response_Buffer [(SPI1_Buffer_Index - 1)] = temp;
}

/**
void Check_Flow_Error (void)
{
	unsigned int  total_error_pulses = 0;
	unsigned char tmpa;
	
	
	//Check Long Term Error
	if (((PINA & (1 << 2)) == 0) && (Flow_Pulse_Count <= 2))				//if Error signal is low and no flow pulse
	{
		Long_Term_Error_Counter++;
		if (Long_Term_Error_Counter >= 6)
		{
			Long_Term_Error_Counter = 0;
			Short_Term_Error        = 1;							//This is long term error but using same variable
			Alarm            	   |= (1 << 5);
		}
	}
	else
	{
		Long_Term_Error_Counter = 0;
	}
	
	//Check Short Term Error	
	Error_Pulse_Queue[0] = Error_Pulse_Queue[1];
	Error_Pulse_Queue[1] = Error_Pulse_Queue[2];
	Error_Pulse_Queue[2] = Error_Pulse_Count;

	//total_error_pulses = Error_Pulse_Queue[0] + Error_Pulse_Queue[1] + Error_Pulse_Queue[2];
	
	total_error_pulses = 0;
	for (tmpa = 0; tmpa < 3; tmpa++)
	{
		if (Error_Pulse_Queue[tmpa] > 1) total_error_pulses += Error_Pulse_Queue[tmpa];
	}

	if ((Error_Pulse_Queue[0] > 1) && (Error_Pulse_Queue[1] > 1) && (Error_Pulse_Queue[2] > 1))		//At least 2 error pulses in each queue
	{
		//Detected error pulses for three successive cycle
			
		if (total_error_pulses >= 6)
		{
			Short_Term_Error  = 1;
			Alarm            |= (1 << 3);
				
			if (Dust_Flow_Disable == 1)
			{
				Flow_Pulse_On_Error = 0;
				Flow_Pulse_Count	= 0;
			}
		}
		else
		{
			if (Short_Term_Error == 0)
			{
				if (Dust_Flow_Disable == 1)
				{
					Flow_Pulse_On_Error += Flow_Pulse_Count;
					Flow_Pulse_Count     = 0;
				}
			}
			else
			{
				if (Dust_Flow_Disable == 1) Flow_Pulse_Count = 0;
			}
		}
	}
	else
	{
		if (total_error_pulses > 1)
		{
			//Detected error pulses in one/two cycles (but not in all three cycles)
			if (Short_Term_Error == 0)
			{
				if (Error_Pulse_Count == 0)									//Error pulses in the current cycle
				{
					Flow_Pulse_Held      = Flow_Pulse_On_Error;
					Flow_Pulse_On_Error  = 0;
					Error_Pulse_Queue[0] = 0;
					Error_Pulse_Queue[1] = 0;
					Error_Pulse_Queue[2] = 0;
				}
				else
				{
					if (Dust_Flow_Disable == 1)
					{
						Flow_Pulse_On_Error += Flow_Pulse_Count;
						Flow_Pulse_Count     = 0;
					}
				}
			}
			else
			{
				if ((Error_Pulse_Count == 0) && (Flow_Pulse_Count > 0))		//Error pulses in the current cycle
				{
					if (Dust_Flow_Disable == 1)
					{
						Flow_Pulse_On_Clear += Flow_Pulse_Count;
						Flow_Pulse_Count     = 0;
					}	
				}
				else
				{
					Error_Pulse_Queue[2] = 2;								//manually add error pulse in queue if no flow
					if (Dust_Flow_Disable == 1)
					{
						Flow_Pulse_On_Clear = 0;
						Flow_Pulse_Count    = 0;
					}	
				}
			}
		}
		else
		{
			if (Short_Term_Error != 0) 										//Do not clear during normal operation (clear only if short term error is set)
			{
				if (Flow_Pulse_Count > 0)
				{
					Short_Term_Error    = 0;
					Alarm              &= ~(1 << 3);
					Alarm              &= ~(1 << 5);
						
					Flow_Pulse_Held     = Flow_Pulse_On_Clear;
					Flow_Pulse_On_Clear = 0;
				}
				else
				{
					Error_Pulse_Queue[2] = 2;								//manually add error pulse in queue if no flow
					if (Dust_Flow_Disable == 1)
					{
						Flow_Pulse_On_Clear = 0;
						Flow_Pulse_Count    = 0;
					}	
				}
			}
		}
	}
}
**/



unsigned char Calculate_AM_Value (void)
{
	float AMCgradient;
	float AMCoffset;

	if (Get_GP30_Values () == 0) return (0);
	
	AMCgradient = Vcal / (AMC_High - AMC_Low);
	AMCoffset   = ((2 * AMC_Low) - AMC_High) * AMCgradient;
	AMVup       = (AMCgradient * AM_Up) - AMCoffset;
	return (1);
}


void Check_AM_Val (void)
{
	if (Min_AM < (2.0f * ((float) AM_Tolerance)))
	{
		Short_Term_Error = 1;
		Alarm           |= (1 << 3);
		Flow_Pulse_Count = 0;
	}
	else
	{
		Short_Term_Error = 0;
		Alarm           &= ~(1 << 3);
	}
}


void Check_Error (void)
{
	if(Error_Pulse_Count > Error_Pulse_Threshold)
	{
		Long_Term_Error  = 1;
		Alarm           |= (1 << 5);
		Flow_Pulse_Count = 0;
	}
	else
	{
		Long_Term_Error  = 0;
		Alarm           &= ~(1 << 5);
	}
}


void Check_Consumption (void)
{
	double tmp_consumption;
	float lpm;
	
	Cumulative_Error_Pulse_Count += Raw_Error_Pulse_Count;
	
	Flow_Pulse_Count      			= Raw_Pulse_Count_Processed;
	Raw_Pulse_Count_Processed       = 0;
	
	Error_Pulse_Count     = Raw_Error_Pulse_Count;
	Raw_Error_Pulse_Count = 0;
	
	Check_Error ();
	Check_AM_Val ();
	
	if ((Minimum_Flow_Pulse > 0) && (Flow_Pulse_Count < Minimum_Flow_Pulse)) Flow_Pulse_Count = 0;		//ignore flow pulses if less then minimum valid pulses (for 100mm)
	

	tmp_consumption = 0;
	lpm = Get_LPM (Flow_Pulse_Count);
	
    if ((Flow_Pulse_Count < 1040) && (Flow_Pulse_Count > 0))
	{
		
		tmp_consumption = (((double) Flow_Pulse_Count *  GetCalibrationValue (lpm) * (Sensor_Calibration_Value / 100.0f)) / Pulse_Per_Litre);
		
		if (lpm > Leakage_LPM)
		{
			Open_Tap_Timer = Open_Tap_Timer + Leakage_Timer + 8;
			Leakage_Timer  = 0;
		}
		else
		{
			Leakage_Timer  = Open_Tap_Timer + Leakage_Timer + 8;
			Open_Tap_Timer = 0;
		}
	}
	
	//-------------------------------------------------------------
	if (Minimumn_Valid_Consumption > 0)										//If "Min Valid Consumption" Check is enabled
	{
		Consumption_Last_Min += tmp_consumption;

		if (Consumption_Last_Min >= (Minimumn_Valid_Consumption * 0.05f))	//1 unit in "Min Valid Consumption" is 50ml; EX: Min Valid Consumption = 5 is 250ml (5 * 50ml = 250ml)
		{
			Consumption_Double  += Consumption_Last_Min;
			Consumption_Last_Min = 0;
			One_Min_Counter		 = 0;
		}
		else
		{
			if (Consumption_Last_Min > 0)
			{
				One_Min_Counter++;
				if (One_Min_Counter >= 7)									//Check Consumption function is called once in 8 sec; 7 * 8 = 56 (~1 min)
				{
					One_Min_Counter 	 = 0;
					Consumption_Last_Min = 0;								//Discard the flow if < Min Valid Consumption for a minute
					Leakage_Timer		 = 0;
					Open_Tap_Timer		 = 0;
				}
			}
			else
			{
				One_Min_Counter = 0;
			}
		}
	}
	else
	{
		Consumption_Double  += tmp_consumption;
		Consumption_Last_Min = 0;
	}
	//-------------------------------------------------------------

	Cumulative_Consumption = (unsigned long int) (Consumption_Double);
	
	Log_LPM_Event_Count (lpm);

	if(Flow_Pulse_Count <= 2)
	{
		Leakage_Timer  = 0;
	    Open_Tap_Timer = 0;
	}

    if (Alarm_Mode_Flag == 1)
	{
		if (Leakage_Timer >= (Leakage_Time_In_Min * 60))
		{
			Alarm |= (1 << 0);	
		}
		else
		{
			Alarm &= ~(1 << 0);
		}
		
		//Open Tap Alarm
		if (Open_Tap_Timer >= (Open_Tap_Time_In_Min * 60))
		{
			Alarm |= (1 << 1);
		}
		else
		{
			Alarm &= ~(1 << 1);
		}
	}
	else
	{
		Alarm &= ~(1 << 0);
		Alarm &= ~(1 << 1);
	}
	
	Cumulative_Flow_Pulse_Count += Flow_Pulse_Count;
	//Error_Pulse_Count=0;			//no need to clear; required when requested from modbus
	//Flow_Pulse_Count=0;			//no need to clear; required when requested from modbus
}


void Get_Modbus_Header (void)
{
	Modbus_Network_ID        = Modbus_Request_Buffer[0];
	Modbus_Function_Code     = Modbus_Request_Buffer[1];
	Modbus_Register_AddressH = Modbus_Request_Buffer[2];
	Modbus_Register_AddressL = Modbus_Request_Buffer[3];
	Modbus_Register_CountH   = Modbus_Request_Buffer[4];
	Modbus_Register_CountL   = Modbus_Request_Buffer[5];
}


void Check_Modbus_Command (void)
{
	unsigned int  register_address;
	unsigned char tmpa;

	if (Check_CRC () != 1) return;
	
	Get_Modbus_Header ();
	
	register_address = ((Modbus_Register_AddressH << 8) + Modbus_Register_AddressL);
	
	if (!((Modbus_Network_ID == Sensor_Network_ID) || (Modbus_Network_ID == 0) || ((Modbus_Function_Code == WRITE_MULTIPLE_REGISTERS) && (register_address == 0x3101) && Modbus_Register_CountL == 0x07))) return;
	
	switch (Modbus_Function_Code)
	{
		case READ_HOLDING_REGISTERS:
			switch (register_address)
			{
				case 0x0001:
					if (Modbus_Register_CountL == 0x04)
					{
						Send_Consumption_Data ();
					}
					break;
				
				case 0x7001:
					if (Modbus_Register_CountL == 0x0F)
					{
						Send_Consumption_Configuration_Data ();
					}
					break;
				
				case 0x5001:
					if (Modbus_Register_CountL == 0x06)
					{
						Send_Configuration_Data ();
					}
					break;

				case 0x6001:
					if (Modbus_Register_CountL == 0x02)
					{
						Send_Revision_No ();
					}
					break;

				case 0xA001:
					if (Modbus_Register_CountL == 0x06)
					{
						Send_Configuration2_Data ();
					}
					break;

				case 0xF000:
					if (Modbus_Register_CountL == 0x08)
					{
						Send_Debug_Info ();
					}
					break;
				
				case 0xB000:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Error_Pulse_Threshold << 8);
					}
					break;

				case 0xB001:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Read_ADC ());
					}
					break;
				case 0xB002:														//Sensor Size
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Sensor_Size << 8);
					}
					break;
				
				case 0xB003:														//AM Value read
					if (Modbus_Register_CountL == 0x01)
					{
						if (AMVup > 0)	Send_Single_Register ((unsigned int) AMVup); else Send_Single_Register (0);
					}
					break;
				
				case 0xB004:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (AM_Tolerance << 8);
					}
					break;
				
				case 0xB005:
					if (Modbus_Register_CountL == 0x01)
					{
						if (Min_AM > 0)	Send_Single_Register ((unsigned int) Min_AM); else Send_Single_Register (0);
					}
					break;
					
				case 0xB006:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (GP30_INT_Count);
					}
					break;
					
				case 0xB007:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Validation_Flow_Secs_Count << 8);
					}
					break;
					
				case 0xB008:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Low_LPM_Pulse_Count << 8);
					}
					break;
				case 0xB009:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Neg_Flow_Reg_Fraction);
					}
					break;
					
				case 0xB00A:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Neg_Flow_Threshold << 8);
					}
					break;
					
				case 0xB00B:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register ((PORTC & (1 << 0)) << 8);
					}
					break;
					
				case 0xB00C:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (Maximum_Flow_Pulse);
					}
					break;
					
				case 0xB00D:
				case 0xB00E:
				case 0xB00F:
				case 0xB010:
					if (Modbus_Register_CountL == 0x01)
					{
						tmpa = Modbus_Register_AddressL - 0x0D;								//Find offset for LPM Range
						Send_Single_Register (LPM_Ranges[tmpa] << 8);
					}
					break;
					
				case 0xB011:
				case 0xB012:
				case 0xB013:
				case 0xB014:
					if (Modbus_Register_CountL == 0x01)
					{
						tmpa = Modbus_Register_AddressL - 0x11;								//Find offset for Event Range
						Send_Single_Register (Event_Time_Ranges[tmpa] << 8);
					}
					break;
					
				case 0xB015:
				case 0xB016:
				case 0xB017:
				case 0xB018:
					if (Modbus_Register_CountL == 0x01)
					{
						tmpa = Modbus_Register_AddressL - 0x15;								//Find offset for LPM Log
						Send_Single_Register (LPM_Log_Count[tmpa]);
					}
					break;
					
				case 0xB019:
				case 0xB01A:
				case 0xB01B:
				case 0xB01C:
					if (Modbus_Register_CountL == 0x01)
					{
						tmpa = Modbus_Register_AddressL - 0x19;								//Find offset for Event Log
						Send_Single_Register (Event_Log_Count[tmpa]);
					}
					break;
					
				case 0xB01D:
					if (Modbus_Register_CountL == 0x01)
					{
						Send_Single_Register (AM_Breach_Count);
					}
					break;
					
				case 0xF009:
					if (Modbus_Register_CountL == 0x06)
					{
						Send_Cumulative_Pulse_Info ();
					}
					break;
					
				case 0xF011:
					if (Modbus_Register_CountL == 0x04)
					{
						Send_Flow_Variations ();
					}
				case 0xF00A:
					if (Modbus_Register_CountL == 0x0A)
					{
						Send_LPM_Holder_Array ();
					}
					break;
				case 0xF00B:
					if (Modbus_Register_CountL == 0x0A)
					{
						Send_Event_Holder_Array ();
					}
					break;
				case 0xF00C:
					if (Modbus_Register_CountL == 0x02)
					{
						Send_AM_Breach_Holder_Array();
					}
					break;
			}
			break;


		case WRITE_SINGLE_REGISTER:
			switch (register_address)
			{
				case 0x7004:
				case 0x7005:
				case 0x7006:
				case 0x7007:
				case 0x7008:
					//tmpa = (((Modbus_Register_AddressL - 0x04) * 2) + 2);					//Set Unique ID is not allowed as it is read only from unique ID chip	//Find offset for Factory_Unique_Number
					
					//Factory_Unique_Number[tmpa] = Modbus_Register_CountH;
					//Write_EEPROM ((Factory_Unique_Number_Address + tmpa), Modbus_Register_CountH);
					
					//tmpa += 1;
					
					//Factory_Unique_Number[tmpa] = Modbus_Register_CountL;
					//Write_EEPROM ((Factory_Unique_Number_Address + tmpa), Modbus_Register_CountL);
					break;
					
				case 0x5001:
					Dust_Flow_Disable = Modbus_Register_CountH;
					Write_EEPROM (Dust_Flow_Disable_Address, Dust_Flow_Disable);
					break;

				case 0x5002:
					Sensor_Calibration_Value = Modbus_Register_CountL;
					Write_EEPROM (Sensor_Calibration_Value_Address, Sensor_Calibration_Value);
					break;

				case 0x5003:
					Leakage_Time_In_Min = Modbus_Register_CountH;
					Write_EEPROM (Leakage_Time_Address, Leakage_Time_In_Min);
					
					//Leakage_Cutoff_Value = Modbus_Register_CountL;
					//Write_Eeprom (Leakeage_Cutoff_Value_Address, Leakage_Cutoff_Value);				
					break;

				case 0x5004:
					Open_Tap_Time_In_Min = Modbus_Register_CountH;
					Write_EEPROM (Open_Tap_Time_Address, Open_Tap_Time_In_Min);
					
					//Open_Tap_CutOff_Value = Modbus_Register_CountL;
					//Write_Eeprom (OpenTap_Cutoff_Value_Address, Open_Tap_CutOff_Value);
					break;

				case 0x5005:
					//if (Modbus_Register_CountH == 1) Restore_Factory_Flag = 1;
					//if (Modbus_Register_CountL == 1) Reset_Value_Flag = 1;
					break;

				case 0x5006:
					//Pulse_Per_LPM = Modbus_Register_CountL;
					//Write_Eeprom (Pulse_Per_LPM_Address, Pulse_Per_LPM);
					break;

				case 0x5009:
					Alarm_Mode_Flag = Modbus_Register_CountH;
					Write_EEPROM (Alarm_State_Address, Alarm_Mode_Flag);
					break;

				case 0x500A:
					Pulse_Per_Litre = Modbus_Register_CountH;
					Write_EEPROM (Pulse_Per_Litre_Address, Modbus_Register_CountH);
					break;

				case 0x500B:
					Minimumn_Valid_Consumption = Modbus_Register_CountH;
					Write_EEPROM (Minimum_Valid_Consumption_Address, Minimumn_Valid_Consumption);
					break;

				case 0x500C:
					Minimum_Flow_Pulse = Modbus_Register_CountH;
					Write_EEPROM (Minimum_Flow_Pulse_Address, Minimum_Flow_Pulse);
					break;

				case 0xF001:
					tmpa = 0x0250 - Read_ADC ();				//Consider only 8 bits
					Sensor_Network_ID = tmpa;
					break;
				
				case 0xB000:
					Error_Pulse_Threshold = Modbus_Register_CountH;
					Write_EEPROM (Error_Pulse_Threshold_Address, Error_Pulse_Threshold);
					break;

				case 0xB001:			//Do not use B0001, as it is used for ADC, ADC wtite is not possible
					break;
				case 0xB002:
					Sensor_Size = Modbus_Register_CountH;
					Write_EEPROM (Sensor_Size_Address, Sensor_Size);
					break;
					
				case 0xB004:
					AM_Tolerance = Modbus_Register_CountH;
					Write_EEPROM (AM_Tolerance_Address, AM_Tolerance);
					break;
				
				case 0xB007:
					Validation_Flow_Secs_Count = Modbus_Register_CountH;
					Write_EEPROM (Validation_Flow_Secs_Count_Address, Validation_Flow_Secs_Count);
					break;
				
				case 0xB008:
					Low_LPM_Pulse_Count = Modbus_Register_CountH;
					Write_EEPROM (Low_LPM_Pulse_Count_Address, Low_LPM_Pulse_Count);
					break;
				
				case 0xB009:
					//Should not use
					break;
					
				case 0xB00A:
					Neg_Flow_Threshold = Modbus_Register_CountH;
					Write_EEPROM (Neg_Flow_Threshold_Address, Neg_Flow_Threshold);
					break;
					
				case 0xB00B:
					if (Modbus_Register_CountH)
					{
						PORTC |= (1 <<  0);
					}
					else
					{
						PORTC &= ~(1 <<  0);
					}
					break;
					
				case 0xB00C:
					Maximum_Flow_Pulse = Modbus_Register_CountH;
					Maximum_Flow_Pulse <<= 8;
					Maximum_Flow_Pulse |= Modbus_Register_CountL;
					Write_EEPROM (Maximum_Flow_Pulse_Address, Modbus_Register_CountH);
					Write_EEPROM (Maximum_Flow_Pulse_Address + 1, Modbus_Register_CountL);
					break;
					
				case 0xB00D:
				case 0xB00E:
				case 0xB00F:
				case 0xB010:
					tmpa = Modbus_Register_AddressL - 0x0D;								//Find offset for LPM Range
					LPM_Ranges[tmpa] = Modbus_Register_CountH;
					Write_EEPROM (LPM_RANGE_START_LOC_ADDRESS + tmpa, Modbus_Register_CountH);
					break;
					
				case 0xB011:
				case 0xB012:
				case 0xB013:
				case 0xB014:
					tmpa = Modbus_Register_AddressL - 0x11;								//Find offset for Event Range
					Event_Time_Ranges[tmpa] = Modbus_Register_CountH;
					Write_EEPROM (EVENT_TIME_RANGE_START_LOC_ADDRESS + tmpa, Modbus_Register_CountH);
					break;
				
				case 0xB015:
				case 0xB016:
				case 0xB017:
				case 0xB018:
					tmpa = Modbus_Register_AddressL - 0x15;								//Find offset for LPM Log
					LPM_Log_Count[tmpa] = Modbus_Register_CountH;
					LPM_Log_Count[tmpa] <<= 8;
					LPM_Log_Count[tmpa] |= Modbus_Register_CountL;
					Write_EEPROM (LPM_LOG_START_LOC_ADDRESS + (tmpa * 2), Modbus_Register_CountH);
					Write_EEPROM (LPM_LOG_START_LOC_ADDRESS + (tmpa * 2) + 1, Modbus_Register_CountL);
					break;
					
				case 0xB019:
				case 0xB01A:
				case 0xB01B:
				case 0xB01C:
					tmpa = Modbus_Register_AddressL - 0x19;								//Find offset for Event Log
					Event_Log_Count[tmpa] = Modbus_Register_CountH;
					Event_Log_Count[tmpa] <<= 8;
					Event_Log_Count[tmpa] |= Modbus_Register_CountL;
					Write_EEPROM (EVENT_LOG_START_LOC_ADDRESS + (tmpa * 2), Modbus_Register_CountH);
					Write_EEPROM (EVENT_LOG_START_LOC_ADDRESS + (tmpa * 2) + 1, Modbus_Register_CountL);
					break;
					
				case 0xB01D:
					AM_Breach_Count = Modbus_Register_CountH;
					AM_Breach_Count <<= 8;
					AM_Breach_Count |= Modbus_Register_CountL;
					Write_EEPROM (AM_BREACH_START_LOC_ADDRESS, Modbus_Register_CountH);
					Write_EEPROM (AM_BREACH_START_LOC_ADDRESS + 1, Modbus_Register_CountL);
					break;
			}	

			memcpy (&Modbus_Response_Buffer[0], &Modbus_Request_Buffer[0], 8);
			Send_Modbus_Response (8);													//for WRITE_SINGLE_REGISTER, return the command pkt received
			break;

		case WRITE_MULTIPLE_REGISTERS:
			switch (register_address)
			{
				case 0x3101:
					if (Modbus_Register_CountL == 0x07)
					{
						if (memcmp (&Modbus_Request_Buffer[7], &Factory_Unique_Number[0], 12) == 0)
						{
							//Number_Of_Sensors = Modbus_Request_Buffer[19];
							//Write_Eeprom (Total_Sensor_Read_Address, Number_Of_Sensors);

							Sensor_Network_ID = Modbus_Request_Buffer[20];
							Write_EEPROM (NETWORK_ID_EEPROM_ADDRESS, Sensor_Network_ID);
							
							Send_Response_NetworkID ();
						}
					}
					break;
				
				case 0x4101:
					if (Modbus_Register_CountL == 0x04)
					{
						//Not used
					}
					break;
				
				case 0x5101:
					if (Modbus_Register_CountL == 0x01)
					{
						Sensor_Calibration_Value = Modbus_Request_Buffer[8];
						Write_EEPROM (Sensor_Calibration_Value_Address, Sensor_Calibration_Value);
						Send_Response_Calibration ();
					}
					break;
			}
			break;		
	}
}


void Check_and_Send_Modbus_Data (void)
{
	if (RX_Complete)																				//If modbus request is received
	{
		RX_Complete = 0;
		
		memcpy (&Modbus_Request_Buffer[0], &USART0_Receive_Buffer[0], USART0_RX_Count);				//Copy to Modbus request buffer	
		Modbus_Request_Buffer_Count = USART0_RX_Count;												//No of bytes receive in Modbus request
		USART0_RX_Count 			= 0;															//reset receive buffer count
		
		Check_Modbus_Command ();
	}
}

void Set_Factory_Unique_Number (void)
{
	Factory_Unique_Number[0] = 'A';
	Factory_Unique_Number[1] = 'S';
	Factory_Unique_Number[2] = 'E';
	Factory_Unique_Number[3] = 0x00;

	Factory_Unique_Number[4] = ((Unique_ID[0] >> 4) & 0x0F);
	Factory_Unique_Number[5] = (Unique_ID[0] & 0x0F);

	Factory_Unique_Number[6] = ((Unique_ID[1] >> 4) & 0x0F);
	Factory_Unique_Number[7] = (Unique_ID[1] & 0x0F);
	
	Factory_Unique_Number[8] = ((Unique_ID[2] >> 4) & 0x0F);
	Factory_Unique_Number[9] = (Unique_ID[2] & 0x0F);
	
	Factory_Unique_Number[10] = ((Unique_ID[3] >> 4) & 0x0F);
	Factory_Unique_Number[11] = (Unique_ID[3] & 0x0F);
}


void Check_Reverse_Connection (void)
{
	static unsigned int Prev_Neg_Flow_Reg_Fraction = 0;
	static unsigned char Neg_Count = 0, Pos_Count = 0;
	long tmpval;
	
	if ((Neg_Flow_Reg_Fraction != 0xFFFF) && (Neg_Flow_Reg_Fraction != 0x0000))			//just in case if not read properly
	{
		tmpval  = Prev_Neg_Flow_Reg_Fraction;											//Convert to
		tmpval -= Neg_Flow_Reg_Fraction;
		if (tmpval > Neg_Flow_Threshold) 
		{
			if (Neg_Count < 200) Neg_Count++;											//Max count is 200
			Pos_Count = 0;
		}
		else if (tmpval < 0)															//Flow in forward direction
		{
			if (Pos_Count < 200) Pos_Count++;											//Max count is 200
			Neg_Count = 0;
		}
		else																			//No Flow
		{
			Pos_Count = 0;
			Neg_Count = 0;
		}
		
		if (Neg_Count > 7)																//flow in reverse direction for > 1 min
		{
			Alarm |= (1 << 6);
		}

		if (Pos_Count > 7)																//flow in reverse direction for > 1 min
		{
			Alarm &= ~(1 << 6);
		}
				
		Prev_Neg_Flow_Reg_Fraction = Neg_Flow_Reg_Fraction;
	}
	
}

int main(void)
{
	wdt_reset();				//Watchdog Reset
	MCUSR &= ~(1 << WDRF);		//Clear Watchdog reset flag before disable
	wdt_disable();				//Disable Watchdog

	PortB_Init ();
	PortC_Init ();
	PortD_Init ();
	USART_Init ();	
	SPI1_Init ();
	TIMER1_Init ();
	Power_Reduction_Init ();
	External_Interrupt_Init ();

	Initialize();
	
	cli();	
	wdt_enable(WDTO_2S);		//Enable Watchdog reset for 2 Secs
	sei();

	UNIO_Read_Unique_ID ();
	Set_Factory_Unique_Number ();		//Set unique id as factory unique number
	
	Clear_Flags_GP30 ();

	/**
	while (1)
	{
		unsigned char tmp;
		tmp = SPI1_Transceive (0x12);
		//Send_Single_Register_Direct (tmp);
		if (tmp == 0)
		{
			tmp = 0;
		}
	}
	**/
	


	while(1)
    {			
		wdt_reset();																					//Reinitializing WDT
		
		if (Cumulative_Consumption  >= (Previous_Consumption + 10))
		{
			Previous_Consumption = Cumulative_Consumption;
			
			Check_EEPROM_Page_Save_Cumulative_Consumption ();
		}
		
		if (One_Sec_Timer >= 1000)	
		{
			Ten_Sec_Timer += One_Sec_Timer;
			One_Sec_Timer = 0;
			Cumulative_Raw_Pulse_Count += Raw_Pulse_Count;
			
			if (Raw_Pulse_Count == 0)																	//No Flow or Low LPM
			{
				if (High_LPM_Instant == 0)	Raw_Pulse_Count_Processed += Raw_Pulse_Holder;
				Flow_State 			= 0;
				Raw_Pulse_Holder 	= 0;
				Valid_Flow_Counts	= 0;
				High_LPM_Instant 	= 0;
			}
			else if (Flow_State == 0)																	//Flow Initializing
			{
				Valid_Flow_Counts++;
				if (Raw_Pulse_Count > Low_LPM_Pulse_Count)	High_LPM_Instant = 1;
				if (Valid_Flow_Counts >= Validation_Flow_Secs_Count)
				{
					Valid_Flow_Counts 	= 0;
					Flow_State 			= 1;
					Raw_Pulse_Count_Processed += (Raw_Pulse_Holder + Raw_Pulse_Count);
					Raw_Pulse_Holder 	= 0;
				}
				else
				{
					Raw_Pulse_Holder	+= Raw_Pulse_Count;
				}
			}
			else if (Flow_State)																		//Currently water is flowing
			{
				Raw_Pulse_Count_Processed += Raw_Pulse_Count;
			}
			
			Raw_Pulse_Count = 0;
			//Send_Debug_Info ();
			//Send_GP30_Values_Direct ();
		}

		if (Ten_Sec_Timer >= 8000)																		//8 Secs
		{
			Ten_Sec_Timer = 0;
			Check_Consumption ();																		//Consumption and Alarm calculation
			
			Min_AM = 700;
			if (GP30_INT_Count == 0)																	//If no int received in a cycle, clear the int flags to restart interrupts
			{
				Clear_Flags_GP30 ();
			}
			GP30_INT_Count = 0;
			Check_Reverse_Connection ();
		}

		Check_and_Send_Modbus_Data ();
		
		if (SPI_TXN_Complete)
		{
			SPI_TXN_Complete = 0;
			if (Calculate_AM_Value ())
			{
				if (AMVup < Min_AM)
				{
					Min_AM = AMVup;
				}
			}
			//Send_GP30_Values_Direct ();
		}
	}
	return 0;
}