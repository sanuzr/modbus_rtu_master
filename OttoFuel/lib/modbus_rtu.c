/*****************************************************************************************************************
Project : MODBUS Library fro AVR Controllers (for SenseROD)
Version : 1.2
Date    : 23/03/2015
Author  : Sujit Kr Nayak
Company : Uniphos Envirotronic Pvt Ltd

Chip type               : ATmega8
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024 


Rev. History:

V- 1.0  - 23 March 2015:    First Version Created
V- 1.1  - 07 April 2016:    Bug in Hot Pluggable (CRC Error) & ID Mismatch Error Handling rectified
		- 08 April 2016:    Bug in displaying year was rectified
V- 1.2  - 21 June  2016:    Added Input Register Functionality and Ported all Holding Registers to Input Registers
							Added Support for Function Code 0x10 (Write Multiple Holding Register)
		- 24 Oct   2016:    Two Registers (36, 37) Added for Zero Counts and Span Counts
		- 01 Dec   2016:	I/P Register were re-arranged and total of 40 registers are added with reserved register
							Provision to write Single Register (Fn Code: 6) was added
							Provision to set the Slave ID from MODBUS (by Writing to Holding Register 100) was added

		
*/


#include <avr/io.h>
#include "io.c"

//Serial Bus Status
#define      SERIAL_ERROR    0x01
#define      SERIAL_BUSY     0x02
#define      TIMER_RUN		 0x04
#define      NO_RESPONSE     0x08

//Register Numbers
#define		MAX_IP_REGISTER 40			//Maximum Number of I/P Register used
#define		MAX_HL_REGISTER 20			//Maximum Number of Holding Register used

//Incoming Request Codes
#define      Set_Time_Request	0x00
#define      Set_Date_Request	0x01
#define      Set_TLV_Request	0x02
#define      Set_STLV_Request	0x03
#define      Cal_Zero_Request	0x04
#define      Cal_Span_Request	0x05
#define      Set_4mA_Request	0x06
#define      Set_20mA_Request	0x07

unsigned char	system_flags;// sensor_error;
extern EEMEM uint8_t m_Device_ID=10;		//EEPROM Address for Device ID
//MODBUS VARIABLES
volatile unsigned char  comm_flags,timeout;
 uint8_t rx_data[256],tx_data[256];
volatile unsigned int   start_msg,length_msg,tx_length,rx_cntr,tx_cntr;
unsigned int Status_Coils, Incoming_Requests;
volatile bool req_cal;     
//Structure for Device Parameters
struct inst
    {
    unsigned char id;           // MODBUS Device Address
    unsigned char mode;         // Idle(0), Measure(1), Zero(2), Span(3), Maintainance(4)
   } device;
 struct req
 {	unsigned int zero_counts;
	 unsigned int span_counts;
	 unsigned int tlv;           // Incoming Value for TLV
	 unsigned int stlv;         // Incoming Value for STLV
	 unsigned int calgas_val;         // Incoming Value for Calibration Gas Concentration
	 unsigned int mA_High;
	 unsigned int mA_Low;
 } ;
 
struct smart_sens
	{
	signed int  rngl;         //Low Range
	signed int	rngh;         //High Range
	signed int  tlv;          //TLV
	signed int	stlv;         //STLV
	signed int	calgas_val;	  //Standard Gas Value 
	unsigned int caloff_count;
	unsigned int calgain_count;
	float sensor_slope;			//Stores the Slope in RAM
	unsigned char gas_name[8];
	unsigned char gas_unit[4];
	signed int lap_days;
	signed int cal_attempt;
	unsigned int gas_unit_int;
	unsigned char gas_logo;
	unsigned char no_of_dig;
	unsigned char dp;
	unsigned char tolerance;
	unsigned char cal_date;    //Last Calibration Date
	unsigned char cal_month;   //Last Calibration Month
	unsigned char cal_year;    //Last Calibration Year
	unsigned char mfg_date;    //Manufacturing Date
	unsigned char mfg_month;   //Manufacturing Month
	unsigned char mfg_year;    //Manufacturing Year
	unsigned int ss_serial_no_ch12;    //Smart Sensor Serial Number First two Digits
	unsigned int ss_serial_no_ch23;    //EEPROM Address for Smart Sensor Serial Number Second two Digits	
	};
   
struct rtc
    {       
    unsigned char dd,mm, yy;
    unsigned char h, m, s, wd,dummy ; 
    };

//Structure for Measurements
struct parameter 
{
  unsigned int Raw;
  unsigned int mV;
  unsigned int Reading;
  unsigned int Ambient;
  unsigned int Supply_vltg;
}measured;

//Structure for MODBUS Incoming Requests
struct rtc rtc_rd,rtc_wr;
static struct smart_sens sensor;
static struct req requests;
// USART0 Receiver interrupt service routine
ISR (USART_RX_vect)
{
	char readStatus,readData;
	readStatus = UCSR0A;
	//C_SETBIT(LED_RED);
	//C_CLEARBIT(LED_BLUE);
	while(readStatus & RX_COMPLETE)
	{
		readData =  UDR0;
		
		if (!(comm_flags & SERIAL_BUSY))
		{
			if (timeout > 100)                   //Set Serial Error for Broken frame after 50ms (In 1ms, 2 counts @2000Hz)
			{
				comm_flags |= SERIAL_ERROR;
			}
			else
			{
				comm_flags |= TIMER_RUN;
				timeout = 0;
			}

			if(readStatus & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))
			{
				comm_flags |= SERIAL_ERROR;
				timeout = 0;
			}
			else
			{
				//C_SETBIT(LED_BLUE);
				//C_CLEARBIT(LED_GREEN);
				rx_data[rx_cntr++] = readData;
				if (rx_cntr > 250)
				rx_cntr = 250;
			}
		}
		readStatus = UCSR0A;
	}
}
// USART0 Transmitter interrupt service routine
ISR(USART_TX_vect)
{
	if(UCSR0A & DATA_REGISTER_EMPTY)
	{
		if(tx_cntr <= tx_length)
		{
			UDR0 = tx_data[tx_cntr++];
		}
		else
		{
			tx_length = 0;
			tx_cntr = 0;
			//C_SETBIT(LED_BLUE);
			//C_CLEARBIT(LED_GREEN);
			//Disable RS485 Transmission and Enable Reception
			C_CLEARBIT(TX_EN);
		}
	}
}
uint16_t SwapByte(uint16_t data_int)
		{
		uint8_t byte_lo, byte_hi;	
		byte_hi = (uint8_t) (data_int & 0x00FF);
		byte_lo = (uint8_t) (data_int >> 8);
		return ((byte_hi<<8)|byte_lo);
		}
uint16_t modbusCRC16(uint8_t * data, uint8_t size)
	{
	uint8_t i,n;
	uint16_t crc = 0xffff;
	for(i = 0; i <= size; i++)
		{
		crc ^= data[i];
		for(n = 0; n < 8; n++)
		crc = (crc & 0x0001)?((crc >> 1)^0xa001):(crc >> 1);
		}
	return SwapByte(crc); //Return the Swapped Bytes (MSB<>LSB - MODBUS Format)
	}
void address_error(void)
{
  unsigned int tempi;
  
  tx_data[0] = rx_data[0];
  tx_data[1] = rx_data[1];
  
  tx_data[1] += 0x80;
  tx_data[2] = 0x02;               //signal address error
 //tempi = crc16(2, 1);
   tempi = modbusCRC16 (tx_data,2);
  tx_data[3] = tempi >> 8;        //msb of crc
  tx_data[4] = (char)tempi;       //lsb of crc
  
  tx_cntr = 0;
  tx_length = 4;
  
  UDR0 = tx_data[tx_cntr++];
}
void send_echo(void)
{
  unsigned char i;
  unsigned int tempi;
  
  tx_cntr = rx_cntr;
  for (i = 0; i <= rx_cntr - 3; i++)
    tx_data[i] = rx_data[i];
  
  //tempi = crc16(tx_cntr - 3, 1);
  tempi = modbusCRC16 (tx_data,tx_cntr - 3);
  tx_data[tx_cntr - 2] = tempi >> 8;    //msb of crc
  tx_data[tx_cntr - 1] = (char)tempi;   //lsb of crc
  
  
  tx_length = tx_cntr - 1;
  tx_cntr = 0;
  UDR0 = tx_data[tx_cntr++];
}
void function_error(void)
{
  unsigned int tempi;
   
  tx_data[0] = rx_data[0];
  tx_data[1] = rx_data[1];
   
  tx_data[1] += 0x80;
  tx_data[2] = 0x01;    //signal function error
 // tempi = crc16(2, 1);
  tempi = modbusCRC16 (tx_data,2);
  tx_data[3] = tempi >> 8;    //msb of crc
  tx_data[4] = (char)tempi;   //lsb of crc
  
  tx_cntr = 0;
  tx_length = 4;
  UDR0 = tx_data[tx_cntr++];
}
void holding_registers(void)
    {
      unsigned char  tempc,i,j,k;
      unsigned int tempi;
	  
      //Device MODBUS Address 
      tx_data[0] = device.id;		//Slave ID
      tx_data[1] = 0x03;			//Function Code for Holding Reg

      tempc = (char)length_msg;		//Data Bytes 
      tx_data[2] = tempc << 1;			 
      
      //Update Register 1			//Hour from Device RTC
      tx_data[3] = 0;
      tx_data[4] = rtc_rd.h;
      
      //Update Register 2			//Minutes from Device Registers
      tx_data[5] = 0;
      tx_data[6] = rtc_rd.m;
      
      //Update Register 3			//Seconds from Device RTC
      tx_data[7] = 0;
      tx_data[8] = rtc_rd.s;	
	  
	  //Update Register 4			//Date from Device RTC
	  tx_data[9] = 0;
	  tx_data[10] = rtc_rd.dd;
	  
	  //Update Register 5			//Month from Device Registers
	  tx_data[11] = 0;
	  tx_data[12] = rtc_rd.mm;
	  
	  //Update Register 6			//Year from Device RTC
	  tx_data[13] = 0;
	  tx_data[14] = rtc_rd.yy;					
      
      //Update Register 7			//Threshold Low Value  (Read from Smart Sensor)
      tx_data[15] = sensor.tlv >> 8;
      tx_data[16] = (char)sensor.tlv;
      
      //Update Register 8			//Threshold Low Value  (Read from Smart Sensor)
      tx_data[17] = sensor.stlv >> 8;
      tx_data[18] = (char)sensor.stlv;
              
      //Update Register 9			//Span Gas Value (Read from Smart Sensor)
      tx_data[19] = sensor.calgas_val >> 8;
      tx_data[20] = (char)sensor.calgas_val;
	  
	  //Update Register 10			//Counts for 4mA
	  tx_data[21] = requests.mA_Low >> 8;
	  tx_data[22] = (char)requests.mA_Low;
	   
	  //Update Register 11			//Counts for 20mA
	  tx_data[23] = requests.mA_High >> 8;
	  tx_data[24] = (char)requests.mA_High;
	 
	  //Update Register 12				//Incoming Request (from Device)
	  tx_data[25] = Incoming_Requests >> 8;
	  tx_data[26] = (char)Incoming_Requests;
	  
	  //Update Register 13			//Reserved
	  tx_data[27] = 0;
	  tx_data[28] = 0;
	  
	  //Update Register 14			//Reserved
	  tx_data[29] = 0;
	  tx_data[30] = 0;				
	  	  
	  //Update Register 15			//Reserved
	  tx_data[31] = 0;
	  tx_data[32] = 0;
	  
	  //Update Register 16			//Reserved
	  tx_data[33] = 0;
	  tx_data[34] = 0;
	  
	  //Update Register 17			//Reserved
	  tx_data[35] = 0;
	  tx_data[36] = 0;
	  
	  //Update Register 18			//Reserved
	  tx_data[37] = 0;
	  tx_data[38] = 0;
	  
	  //Update Register 19			//Reserved
	  tx_data[39] = 0;
	  tx_data[40] = 0;
	  
	  //Update Register 20				//Reserved
	  tx_data[41] = 0;
	  tx_data[42] = 0;
	       
      tempc = (char)start_msg;
      tempc += (char)length_msg;	//Last Register Number
      
      j = 3;			//First 3 bytes are fixes, i.e 1- Slave ID, 2- Function Code, 3 - Data Length
      for (i = (char)start_msg; i < tempc; i++) //Select the Queried Registers only
      {
        k = i << 1;    //Adjust for Word Spacing, i.e. increment by 2 (As the Registers are integers and data buffer is character)
        tx_data[j++] = tx_data[k + 3];      //upper 8 bits
        tx_data[j++] = tx_data[k + 4];      //lower 8 bits
      }
      
      j--;

	   tempi = modbusCRC16 (tx_data,j); //Calculate the CRC

      tx_data[j + 1] = tempi >> 8;    //MSB of crc
      tx_data[j + 2] = (char)tempi;   //LSB of crc
    
      tx_length = j + 2;			//tx_length = data length + 2 bytes of CRC
      tx_cntr = 0;
      UDR0 = tx_data[tx_cntr++];	//Start Transmission ISR
    }
void wr_holding_registers(void)
	{
	unsigned char  j,k,l;
	unsigned int i,tempi;
	
	//Device MODBUS Address
	tx_data[0] = device.id;		//Slave ID
	tx_data[1] = 0x10;			//Function Code for Writing Holding Reg

	tempi = start_msg;			//Start Address
	tx_data[2] = tempi >> 8;
	tx_data[3] = (char) tempi;
	
	tempi = length_msg;			//Number of Register Written
	tx_data[4] = tempi >> 8;
	tx_data[5] = (char) tempi;
	
	tempi = start_msg;
	tempi += length_msg;
	l =0;
	for (i =start_msg; i < tempi; i++)
		{
		k = (unsigned char) i ;
		switch (k)
			{
			case 0x00:				//Write Register 1
				rtc_wr.h= rx_data[l + 8]; 
				break;
			case 0x01:				//Write Register 2
				rtc_wr.m = rx_data[l + 8]; 
				break;
			case 0x02:				//Write Register 3
				rtc_wr.s= rx_data[l + 8]; 
				break;
			case 0x03:				//Write Register 4
				rtc_wr.dd= rx_data[l + 8]; 
				break;
			case 0x04:				//Write Register 5
				rtc_wr.mm = rx_data[l + 8]; 
				break;
			case 0x05:				//Write Register 6
				rtc_wr.yy= rx_data[l + 8]; 
				break;
			case 0x06:				//Write Register 7
				requests.tlv= (rx_data[l + 7]<< 8)|(rx_data[l + 8]); 
				break;
			case 0x07:				//Write Register 8
				requests.stlv = (rx_data[l + 7]<< 8)|(rx_data[l + 8]); 
				break;
			case 0x08:				//Write Register 9
				requests.calgas_val=(rx_data[l + 7]<< 8)|(rx_data[l + 8]); 
				break;
			case 0x09:				//Write Register 10
				requests.mA_Low=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				req_cal = true;
				break;
			case 0x0A:				//Write Register 11
				requests.mA_High=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				req_cal = true;
				break;
			case 0x0B:				//Write Register 12
				Incoming_Requests = (rx_data[l + 7]<< 8)|(rx_data[l + 8]);	
				break;
			case 0x0C:				//Write Register 13, Reserved
				//rtc_wr.mm = rx_data[l + 8];
				break;
			case 0x0D:				//Write Register 14, reserved
				//rtc_wr.yy= rx_data[l + 8];
				break;
			case 0x0E:				//Write Register 15, reserved
				//requests.tlv= (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				break;
			case 0x0F:				//Write Register 16, reserved
				//requests.stlv = (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				break;
			case 0x10:				//Write Register 17, Reserved
				//requests.calgas_val=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				break;
			case 0x11:				//Write Register 18, Reserved
				//requests.mA_Low=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				//req_cal = true;
				break;
			case 0x12:				//Write Register 19, Reserved
				//requests.mA_High=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				//req_cal = true;
				break;
			case 0x13:				//Write Register 20, Reserved
				//Incoming_Requests = (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
				break;
			}
		l=l+2;	//Increment 2 Bytes (One Register Length)
			
		}
	
	j=5;	//Response is Always 6 Byte long (for Function Code 0x10 i.e. Dev ID, Fn Code, Start Address(2byte), Bytes Written (2 bytes))

	tempi = modbusCRC16 (tx_data,j); 

	tx_data[j + 1] = tempi >> 8;    //msb of crc
	tx_data[j + 2] = (char)tempi;   //lsb of crc
	
	tx_length = j + 2;
	tx_cntr = 0;
	UDR0 = tx_data[tx_cntr++];
	}	
void wr_single_register(void)
{
	unsigned char  l=5, h=4;
	//unsigned int k;
		
	//k = start_msg;
	switch (start_msg)
		{
		case 0x00:				//Write Register 1
		rtc_wr.h= rx_data[l];
		break;
		case 0x01:				//Write Register 2
		rtc_wr.m = rx_data[l];
		break;
		case 0x02:				//Write Register 3
		rtc_wr.s= rx_data[l];
		break;
		case 0x03:				//Write Register 4
		rtc_wr.dd= rx_data[l];
		break;
		case 0x04:				//Write Register 5
		rtc_wr.mm = rx_data[l];
		break;
		case 0x05:				//Write Register 6
		rtc_wr.yy= rx_data[l];
		break;
		case 0x06:				//Write Register 7
		requests.tlv= (rx_data[h]<< 8)|(rx_data[l]);
		break;
		case 0x07:				//Write Register 8
		requests.stlv = (rx_data[h]<< 8)|(rx_data[l]);
		break;
		case 0x08:				//Write Register 9
		requests.calgas_val=(rx_data[h]<< 8)|(rx_data[l]);
		break;
		case 0x09:				//Write Register 10
		requests.mA_Low=(rx_data[h]<< 8)|(rx_data[l]);
		req_cal = true;
		break;
		case 0x0A:				//Write Register 11
		requests.mA_High=(rx_data[h]<< 8)|(rx_data[l]);
		req_cal = true;
		break;
		case 0x0B:				//Write Register 12
		Incoming_Requests = (rx_data[h]<< 8)|(rx_data[l]);
		break;
		case 0x0C:				//Write Register 13, Reserved
		//rtc_wr.mm = rx_data[l + 8];
		break;
		case 0x0D:				//Write Register 14, reserved
		//rtc_wr.yy= rx_data[l + 8];
		break;
		case 0x0E:				//Write Register 15, reserved
		//requests.tlv= (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		break;
		case 0x0F:				//Write Register 16, reserved
		//requests.stlv = (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		break;
		case 0x10:				//Write Register 17, Reserved
		//requests.calgas_val=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		break;
		case 0x11:				//Write Register 18, Reserved
		//requests.mA_Low=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		//req_cal = true;
		break;
		case 0x12:				//Write Register 19, Reserved
		//requests.mA_High=(rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		//req_cal = true;
		break;
		case 0x13:				//Write Register 20, Reserved
		//Incoming_Requests = (rx_data[l + 7]<< 8)|(rx_data[l + 8]);
		break;
		}	
	send_echo();
}
void input_registers(void)
	{
	unsigned char  tempc,i,j,k;
	unsigned int tempi;
	
	//Device MODBUS Address
	tx_data[0] = device.id;		//Slave ID
	tx_data[1] = 0x04;			//Function Code for Reading I/P Registers

	tempc = (char)length_msg;		//Data Bytes
	tx_data[2] = tempc << 1;
	
	//Update Register 1			//Date from Device RTC
	tx_data[3] = 0;
	tx_data[4] = rtc_rd.dd;
	
	//Update Register 2			//Month from Device RTC
	tx_data[5] = 0;
	tx_data[6] = rtc_rd.mm;
	
	//Update Register 3			//Year from Device RTC
	tx_data[7] = (char)((rtc_rd.yy+2000)>>8);;
	tx_data[8] = (char) (rtc_rd.yy+2000);;
	
	//Update Register 4			//Mfg Date (Read from Smart Sensor)
	tx_data[9] = 0;
	tx_data[10] = sensor.mfg_date;
	
	//Update Register 5			//Mfg Month (Read from Smart Sensor)
	tx_data[11] = 0;
	tx_data[12] = sensor.mfg_month;
	
	//Update Register 6			//Mfg Year (Read from Smart Sensor)
	tx_data[13] = (char)((sensor.mfg_year+2000)>>8);
	tx_data[14] = (char) (sensor.mfg_year+2000);
	
	//Update Register 7			//Calibration (Read from Smart Sensor)
	tx_data[15] = 0;
	tx_data[16] = sensor.cal_date;
	
	//Update Register 8			//Calibration (Read from Smart Sensor)
	tx_data[17] = 0;
	tx_data[18] = sensor.cal_month;
	
	//Update Register 9			//Calibration Year (Read from Smart Sensor)
	tx_data[19] = (char) ((sensor.cal_year+2000)>>8);
	tx_data[20] = (char) (sensor.cal_year+2000);
	
	//Update Register 10			//Sensor Lo-Range  (Read from Smart Sensor)
	tx_data[21] = 0;
	tx_data[22] = 0;
	
	//Update Register 11			//Sensor Hi-Range  (Read from Smart Sensor)
	tx_data[23] = sensor.rngh >> 8;
	tx_data[24] = (char)sensor.rngh;
	
	//Update Register 12			//Threshold Low Value  (Read from Smart Sensor)
	tx_data[25] = sensor.tlv >> 8;
	tx_data[26] = (char)sensor.tlv;
	
	//Update Register 13			//Threshold Low Value  (Read from Smart Sensor)
	tx_data[27] = sensor.stlv >> 8;
	tx_data[28] = (char)sensor.stlv;
	
	//Update Register 14			//Span Gas Value (Read from Smart Sensor)
	tx_data[29] = sensor.calgas_val >> 8;
	tx_data[30] = (char)sensor.calgas_val;
			
	//Update Register 15 - 22	(8 Nos)	//Gas Name (Read from Smart Sensor)
	for (i=0;i<8;i++)
	{
		j = i<<1;
		tx_data[31 + j] = 0;
		tx_data[32 + j] = sensor.gas_name[i];
	}
	//Update Register 23 - 26 (4 Nos)		//Gas Unit (Read from Smart Sensor)
	for (i=0;i<4;i++)
	{
		j = i<<1;
		tx_data[47 + j] = 0;
		tx_data[48 + j] = sensor.gas_unit[i];
	}
	//Update Register 27				//Sensor Life (Read from Smart Sensor)
	tx_data[55] = sensor.lap_days  >> 8;
	tx_data[56] = (char)sensor.lap_days; 
	
	//Update Register 28				//No. of Calibration Attempts (Read from Smart Sensor)
	tx_data[57] = sensor.cal_attempt >> 8;
	tx_data[58] = (char)sensor.cal_attempt; 
	
	//Update Register 29				//Zero Counts from EEPROM(Smart Sensor)
	tx_data[59] = sensor.caloff_count >>8;
	tx_data[60] = (char)sensor.caloff_count;
	
	//Update Register 30				//Span Counts from EEPROM(Smart Sensor)
	tx_data[61] = sensor.calgain_count >>8;
	tx_data[62] = (char)sensor.calgain_count;
	
	//Update Register 31				//Status Coils (from Device)
	tx_data[63] = Status_Coils >> 8;
	tx_data[64] = (char)Status_Coils;
	
	//Update Register 32			//Temperature (In Device)
	tx_data[65] = measured.Ambient >> 8;
	tx_data[66] = (char)measured.Ambient;
	
	//Update Register 33				//System Voltage Reading (from Device ADC)
	tx_data[67] = measured.Supply_vltg >>8;
	tx_data[68] = (char)measured.Supply_vltg;
	
	//Update Register 34				//Raw Counts for Sensor O/P
	tx_data[69] = measured.Raw >> 8;
	tx_data[70] = (char)measured.Raw;
	
	//Update Register 35				//Sensor mV from ADC
	tx_data[71] = measured.mV >>8;
	tx_data[72] = (char)measured.mV;
	
	//Update Register 36				//Sensor Reading (from Device ADC)
	tx_data[73] = measured.Reading >> 8;
	tx_data[74] = (char)measured.Reading;
	
	//Update Register 37			//Resolution (Decimal Point)   (Read from Smart Sensor)
	tx_data[75] = 0;
	tx_data[76] = sensor.dp;
	
	//Update Register 38			//Reserved
	tx_data[77] = 0;
	tx_data[78] = 0;
	
	//Update Register 39			//Reserved
	tx_data[79] = 0;
	tx_data[80] = 0;
	
	//Update Register 40			//Reserved
	tx_data[81] = 0;
	tx_data[82] = 0;
		
	tempc = (char)start_msg;
	tempc += (char)length_msg;
	
	j = 3;
	for (i = (char)start_msg; i < tempc; i++)
	{
		k = i << 1;    //adjust for word spacing; SEND THE QUERRIED REGISTERS ONLY
		tx_data[j++] = tx_data[k + 3];      //upper 8 bits
		tx_data[j++] = tx_data[k + 4];      //lower 8 bits
	}
	
	j--;

	tempi = modbusCRC16 (tx_data,j);

	tx_data[j + 1] = tempi >> 8;    //msb of crc
	tx_data[j + 2] = (char)tempi;   //lsb of crc
	
	tx_length = j + 2;
	tx_cntr = 0;
	UDR0 = tx_data[tx_cntr++];
}
void decode_slave(void)
    {
		//  uint8 i;
		unsigned int tempi1,tempi2,address;
        
		if (comm_flags & SERIAL_BUSY)
		{
		//C_CLEARBIT(LED_BLUE);  
		//C_SETBIT(LED_RED);
		
       // tempi1 = crc16(rx_cntr - 3, 0);              //Computed CRC in tempi1
		tempi1 = modbusCRC16 (rx_data,rx_cntr-3);
        tempi2 = (int)rx_data[rx_cntr - 2];           //Get MSB of CRC form received string
        tempi2 <<= 8;
        tempi2 += (int)rx_data[rx_cntr - 1];          //Get LSB of CRC form received string

        if(tempi1 == tempi2)                        //Check if Received CRC and Computed CRC are equal 
			{
			//C_CLEARBIT(LED_RED);
			//C_SETBIT(LED_GREEN);  
			if (rx_data[0] == device.id)           //Check if Slave ID is matching
			  {                                                         
				
				
			switch (rx_data[1])				//Function Check
				{
						  
				  case 0x04:								//Read I/P Register
					start_msg = (int)rx_data[2];        //message start
					start_msg <<= 8;
					start_msg += (int)rx_data[3];
					length_msg = (int)rx_data[4];       //message length
					length_msg <<= 8;
					length_msg += (int)rx_data[5];   
					tempi1 = start_msg + length_msg;
                
					if (tempi1 <= MAX_IP_REGISTER)      //Total Input Registers to be read
						input_registers();
					else
						address_error();      
					break;
					
				case 0x03:										//Read Holding Register
						start_msg = (int)rx_data[2];        //message start
						start_msg <<= 8;
						start_msg += (int)rx_data[3];
						length_msg = (int)rx_data[4];       //message length
						length_msg <<= 8;
						length_msg += (int)rx_data[5];
						tempi1 = start_msg + length_msg;
				
						if (tempi1 <= MAX_HL_REGISTER)      //Total Holding Registers to be read
							holding_registers();
						else
							address_error();
						break;
				case 0x10:										//Write Holding Register
						start_msg = (int)rx_data[2];        //message start
						start_msg <<= 8;
						start_msg += (int)rx_data[3];
						length_msg = (int)rx_data[4];       //message length
						length_msg <<= 8;
						length_msg += (int)rx_data[5];
						tempi1 = start_msg + length_msg;
				
						if (tempi1 <= MAX_HL_REGISTER)      //Total Holding Registers to be read
						wr_holding_registers();
						else
						address_error();
						break;
              
				  case 0x06:								//Write Single Holding Register
						address = (int)rx_data[2];           //message start
						address <<= 8;
						address += (int)rx_data[3];
                      
						if (address <= MAX_HL_REGISTER)
							{
							start_msg= address;
							wr_single_register();
							}
						else if (address == 100)	//If Register Address is 100, Set the Device ID
							{
							device.id = rx_data[5];
							eeprom_write_byte(&m_Device_ID, device.id);
							send_echo();
							_delay_ms(20);
							}
						else
						  address_error();
					  break;
              
				  default:
					function_error();
				  break;
				}                 
			}
		else //In Case of ID Mismatch, Disable RS485 Transmission and Enable Reception
			C_CLEARBIT(TX_EN); 
        }
	else
		{
		//In Case of CRC Error, Disable RS485 Transmission and Enable Reception 
		C_CLEARBIT(TX_EN);
		}
        rx_cntr = 0;
        comm_flags &= ~SERIAL_BUSY;  
      }
    }
void housekeeping(void)
    {  
    //Running @2000Hz => In 1s = 2000 Counts, 1000ms = 2000 Counts, => 2 Counts in 1 ms
    if (comm_flags & SERIAL_ERROR)     //on error reset all comm parameters
        {
        comm_flags = 0;
        timeout = 0;
        rx_cntr = 0;
        }
      else
          {	
          if (comm_flags & TIMER_RUN)    //run timeout timer only if data stream has started
                {
                  timeout++;
                  if (timeout > 20)            //close packet after 10ms of inactivity (1ms = 2 counts)
                      { 
                      //Enable RS485 Transmission (Driver) and Disable Reception
                        C_SETBIT(TX_EN);
                        _delay_us(10);
                        comm_flags &= ~TIMER_RUN;
                        timeout = 0;
                        comm_flags |= SERIAL_BUSY;
                      }		
                }
          }
    }