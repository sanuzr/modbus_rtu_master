/*
 * Project:Otto Fuel Vapour Detector
 * PCB: EC_IN_OTTO_01, V-0.1
 * Version: V-0.1
 * Created: 29-01-2018 09:28:42
 * Author : Sujit Kr Nayak
 * Configurations:
				CPU: ATmega328PU
				CPU Speed: 8.0MHz
				TIMER0 Interrupt (8-bit) is used for periodic scanning of events
				TIMER1 is used for PWM Generation for driving the Sampling Pump
				UART0 Rx and Tx Interrupts are used to serve MODBUS Interface (Configured as Master)
				
* H/W Specifications:

1. 16-bit (TIMER1) PWM to Drive Sampling Pump (by MOSFET), but effective byte using now is 12 bit to increase the frequency to 2kHz
2. RS485 Driver(MAX485) on UART (for MODBUS Master Implementation)
3. I2C RTC (DS1307) and EEPROM (AT24C256) for Future Use
4. 20x4 OLED Display Interfacing 	
5. 4 Keys for User Interface
6. 2 I/O for LED Indicators		
				
Revision History:

V-0.1:			29-Jan-2018: Started the Firmware Writing (Imported from SenseROD Firmware)
 */ 

#define F_CPU		8000000UL   //CPU Speed
#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "lib\EEPROM_RTC\eeprom.h"
#include "lib\EEPROM_RTC\eeprom.c"
#include "lib\uart\uart.h"
#include "lib\uart\uart.c"
#include "lib\io.c"
#include "lib\LCD\lcd.h"
#include "lib\LCD\lcd.c"
#include "lib\modbus_rtu_master.c"

//------------------------------------------ Constants ----------------------------------------


//LED_Bits
#define LED_Bit_Pump		0X00
#define LED_Bit_Heater		0X01

//Key Assignments
#define SET		4			// PORTD.4
#define ENT		3			// PORTD.3
#define UP		7			// PORTB.7
#define DN		6			// PORTB.6

//Pump PWM Fixed Counts
//Ideal Counts for 2kHz PWM Frequency
#define mLPM_100				166		//Ideal Counts for 100 ml/min
#define mLPM_400				3999	//Ideal Counts for 400 ml/min
#define mLPM_Factor			((float) (mLPM_High - mLPM_Low)/(float) 300)
#define DAC_Counts			OCR1B		//DAC Register

char  PREV_PIND = 0xFF, PREV_PINB = 0xFF;//lcd_buffer[20];
//Global variables
uint8_t LED_Freq, LED_Flag;
char lcd_buffer[20];
EEMEM uint16_t m_mLPM_Low = 4, m_mLPM_High = 6;	//EEPROM Address for mA_Low & mA_High
EEMEM uint8_t First_Run_Flag = 8;		//EEPROM Address for Flag to Check First Run of the device
uint16_t  mLPM_Low, mLPM_High;
bool	Cold_Start, LED_Tick_1, Sec_Tik; 

//Function Definitions
void ServeRequests(void);
void UpdateStatus(void);
void Build_Coil(void);
uint8_t GetKeyStatus(uint8_t key);
uint8_t GetPrevKeyStatus(uint8_t key);
void Welcome(void);
//void MainMenu(void);
void StartSampling(void);
void SetFlow(void);
//void Settings(void);
//void SetTime(void);
//void SetDate(void);
uint8_t GetKeyStatus(uint8_t key)
	{
	if (key == UP || key == DN)
	return (!CHECKBIT(PINB,key));
	else
	return (!CHECKBIT(PIND,key));
	}
uint8_t GetPrevKeyStatus(uint8_t key)
	{
	if (key == UP || key== DN)
	return (!CHECKBIT(PREV_PINB,key));
	else
	return (!CHECKBIT(PREV_PIND,key));
	}
// Timer0 Overflow interrupt service routine
ISR(TIMER0_OVF_vect) //Running @2000Hz
	{
	static uint8_t ticks,led_ticks;
	TCNT0=0xF0;
	ticks++;
	led_ticks++;
	if(ticks>=2000)
		{
		ticks =0;
		Sec_Tik = true;
		//LED_Tick_1 = true;
		}
	if(led_ticks>=(200*LED_Freq)) //
		{
		led_ticks =0;
		//Sec_Tik = true;
		LED_Tick_1 = true;
		}
	//MODBUS Functions
	housekeeping();
	//decode_slave();
	//ServeRequests();
	UpdateStatus();
	//Build_Coil();
	}

void Set_mLPM(uint16_t PWM_Value)
	{
	static uint16_t XTR_Cnt;	
		XTR_Cnt = (mLPM_Low+(mLPM_Factor*PWM_Value));
		if(XTR_Cnt > mLPM_High)
			XTR_Cnt = mLPM_High;
		DAC_Counts = XTR_Cnt;	
	}
void Flow_Adjustment (uint8_t type, uint16_t current_value)
	{
	static uint16_t new_count;;
	//New_Count = Currrent_Count/Current_I_Out(in uA)*Desired_Output(in uA)
	switch(type)
		{
		case 1:
		new_count = (uint16_t) mLPM_100*((float)100/(float)current_value);
		//EepromPageWrite(EEPROM_ADD, ADDR_MA_4, (unsigned char *) &new_count, 2);
		mLPM_Low = new_count;
		eeprom_write_word(&m_mLPM_Low, mLPM_Low);
		_delay_ms(20);
		break;
		case 4:
		new_count = (uint16_t) mLPM_400*((float)400/(float)current_value);
		//EepromPageWrite(EEPROM_ADD, ADDR_MA_20, (unsigned char *) &new_count, 2);
		mLPM_High = new_count;
		eeprom_write_word(&m_mLPM_High, mLPM_High);
		_delay_ms(20);
		break;	
		}
	}
void HwInit()				// Initialize MCU Pins as I/P, O/P or Internal-pull
	{	
	uint8_t first_run_status=0;
	
	//Keys
	C_CLEARBIT(DIR_KEY_UP);	//Configure as I/P
	C_SETBIT(PORT_KEY_UP);		//Enable Pull-up
	C_CLEARBIT(DIR_KEY_DN);
	C_SETBIT(PORT_KEY_DN);
	C_CLEARBIT(DIR_KEY_ENT);
	C_SETBIT(PORT_KEY_ENT);
	C_CLEARBIT(DIR_KEY_SET);
	C_SETBIT(PORT_KEY_SET);
	
	//I/O Setting for Indication LED
	C_SETBIT(DIR_LED_HEATER);		//Configure as O/P
	C_CLEARBIT(LED_HEATER);			//Set O/P to Low Logic
	
	C_SETBIT(DIR_LED_PUMP);			//Configure as O/P
	C_CLEARBIT(LED_PUMP);			//Set O/P to Low Logic
	
	
	// I/O Setting for DIOs
	
	// I/O Setting forDirection PWM
	C_SETBIT(DIR_PWM);		//Configure as O/P
	C_SETBIT(PWM);		//Set O/P to HIGH Logic
	
	// I/O Setting for Transmit Enable for MAX487 (RS485 Driver)
	C_SETBIT(DIR_TX_EN);		//Configure as O/P
	C_CLEARBIT(TX_EN);			//Set O/P to Low Logic
	
	
	
	// Timer/Counter 0 initialization, Used for Periodic Scanning 
	// Clock source: System Clock
	// Clock value: 31.250 kHz
	
	TCCR0B=(1<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0xF0;
	
	/* For PWM Frequency 122 Hz, Bit Resolution Obtained = 16 Bit
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 8000.000 kHz
	// Mode: Fast PWM top=ICR1
	// OC1A output: Non-Inverted PWM
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 8.192 ms
	// Output Pulse(s):
	// OC1A Period: 8.192 ms Width: 2.0479 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0xFF;
	ICR1L=0xFF;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	*/
	//For PWM Frequency of 2kHz, Bit Resolution Obtained = 12 Bit
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 8000.000 kHz
	// Mode: Fast PWM top=ICR1
	// OC1A output: Disconnected
	// OC1B output: Non-Inverted PWM
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 0.5 ms
	// Output Pulse(s):
	// OC1A Period: 0.5 ms Width: 0.0 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x0F;
	ICR1L=0x9F;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;
	
	
	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK0= (0<<OCIE0A) | (0<<OCIE0B) | (1<<TOIE0);
		
	Uart0Init();						// USART0 Initialize
	
	first_run_status = eeprom_read_byte(&First_Run_Flag);
	if(first_run_status==6)			//System is Already Configured Earlier, Copy the Essential Variables
		{
		mLPM_Low = eeprom_read_word(&m_mLPM_Low);
		_delay_ms(10);
		mLPM_High = eeprom_read_word(&m_mLPM_High);
		_delay_ms(10);
		}
	else							//System is Running for first time, Set default esseential parameters
		{
		mLPM_Low = mLPM_100;;				//Default PWM Counts for 4mA O/P
		mLPM_High = mLPM_400;			//Default PWM Counts for 20mA O/P
		first_run_status =6;		//Set First Run Bit
		eeprom_write_word(&m_mLPM_Low, mLPM_Low);
		_delay_ms(20);
		eeprom_write_word(&m_mLPM_High, mLPM_High);
		_delay_ms(20);
		 eeprom_write_byte(&First_Run_Flag, first_run_status);
		_delay_ms(20);
		}
	
	sei();
	
	//C_SETBIT(LED_HEATER);
	//C_SETBIT(LED_PUMP);
	//_delay_ms(1000);
	//LCD Initialization
	lcd_init(LCD_DISP_ON);				// LCD Initialize
	_delay_ms(50);
	/*
	SETONLY(LED_Flag, LED_Bit_Heater);
	CLEARBIT(LED_Flag, LED_Bit_Pump);
	_delay_ms(2000);
	CLEARBIT(LED_Flag, LED_Bit_Heater);
	SETONLY(LED_Flag, LED_Bit_Pump);
	_delay_ms(2000);
	CLEARBIT(LED_Flag, LED_Bit_Heater);
	CLEARBIT(LED_Flag, LED_Bit_Pump);
	*/
	}
void UpdateStatus()
	{
		if(LED_Tick_1)
			{
			LED_Tick_1 = false;
			switch(LED_Flag)
				{
				case 0x00:
					C_CLEARBIT(LED_PUMP);
					C_CLEARBIT(LED_HEATER);
					break;
				case 0x01:
					C_FLIPBIT(LED_PUMP);
					C_CLEARBIT(LED_HEATER);
					break; 
				case 0x02:
					C_CLEARBIT(LED_PUMP);
					C_FLIPBIT(LED_HEATER);
					break;
				
				default:
					C_CLEARBIT(LED_PUMP);
					C_CLEARBIT(LED_HEATER);
					break;
				}
			}	
	}
void Welcome()				// Display Welcome Text on 16x2 LCD/OLED
{
	
	lcd_clrscr();
	lcd_gotoxy(0,0);	lcd_puts("       ecTech       ");
	_delay_ms(5000);
	/*
	lcd_clrscr();
	lcd_gotoxy(0,0);	lcd_puts("     Otto Fuel      ");
	_delay_ms(10);
	lcd_gotoxy(0,1);	lcd_puts("  Vapour Detector   ");
	_delay_ms(4000);
	
	lcd_clrscr();
	lcd_gotoxy(0,0);	lcd_puts(" Serial No:         ");
	_delay_ms(10);
	lcd_gotoxy(0,1);	lcd_puts("   OTTO-VD-031702   ");
	_delay_ms(4000);
	lcd_clrscr();
	lcd_gotoxy(0,0);	lcd_puts(" Software Ver:      ");
	_delay_ms(10);
	lcd_gotoxy(0,1);	lcd_puts("       V-0.0.1      ");
	_delay_ms(4000);
	*/
	lcd_clrscr();
}
int main(void)
	{
	
	uint16_t temp_u16, Reg_Address,Reg_Length ;
	uint8_t Slave_ID, Function_Code;
	HwInit();
	Welcome();
	LED_Freq = 2;		//1000 mS delay
	SETBIT(LED_Flag, LED_Bit_Heater); 
	DAC_Counts = 3000;
    while (1) 
		{
		Slave_ID=247;
		Function_Code=4;
		Reg_Address=4096;
		Reg_Length=2;
		lcd_clrscr();
		lcd_gotoxy(0,1);
		_delay_ms(500);
		
		MBQuery(Slave_ID, Function_Code, Reg_Address,0,Reg_Length);
		_delay_ms(1000);
		
		temp_u16 = decode_slave();
		_delay_ms(100);
		//C_CLEARBIT(LED_HEATER);
		if (temp_u16 == 100)
		temp_u16 = ((Rx_Value[0]<<8) | Rx_Value[1]);
		sprintf(lcd_buffer,"Ret_Value:%02d", temp_u16);
		lcd_gotoxy(0,1);	lcd_puts(lcd_buffer);
		_delay_ms(1000);
		}
		
	}

