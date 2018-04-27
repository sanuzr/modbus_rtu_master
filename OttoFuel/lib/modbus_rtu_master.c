/*****************************************************************************************************************
Project : MODBUS Master Library fro AVR Controllers (Used in Otto Fuel MK-15)
Version : 0.1
Date    : 21/02/2018
Author  : Sujit Kr Nayak
Company : Uniphos Envirotronic Pvt Ltd

Chip type               : ATmega328
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024 


Rev. History:

V- 1.0  - 20 Feb 2018:    First Version Created, Working (function Code: 03, 04, 06)

		
*/
//#include "variables.h"
#include <avr/io.h>
#include "io.c"


//Serial Bus Status
#define      SERIAL_ERROR    0x01
#define      SERIAL_BUSY     0x02
#define      TIMER_RUN		 0x04
#define      NO_RESPONSE     0x08
#define      RESP_TIMER_RUN	 0x10

#define		 MODBUS_EXCEPTION_FLAG 0x80		//In case of Exception, the MSB of Function Code is Set

uint8_t	comm_flags,Rx_Buf[256],Tx_Buf[256], Rx_Value[24], system_flags, frame_timeout;// sensor_error;
//MODBUS VARIABLES
static uint16_t   rx_cntr,tx_cntr,tx_length, data_bytes, resp_timeout;

// USART0 Receiver interrupt service routine
ISR (USART_RX_vect)
	{
	char readStatus,readData;
	readStatus = UCSR0A;
	
	while(readStatus & RX_COMPLETE)		//If UART Receive is Completed and Unread Data is there in the Buffer
		{
		readData =  UDR0;//Copy Serial Buffer Data to Variable
						 
		if (!(comm_flags & SERIAL_BUSY))	//If MCU is not Busy in Processing Previous Requests/Response
			{
			if (frame_timeout > 100)        //Set Serial Error for Broken frame after 50ms (In 1ms, 2 counts @2000Hz)
				{
				comm_flags |= SERIAL_ERROR;
				//frame_timeout = 0;
				}
			else
				{
				comm_flags |= TIMER_RUN;
				frame_timeout = 0;
				}

			if(readStatus & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) //Error in the Received Data
				{
				//C_FLIPBIT(LED_HEATER);
				comm_flags |= SERIAL_ERROR;
				frame_timeout = 0;
				}
			else
				{
				//C_SETBIT(LED_HEATER);
				Rx_Buf[rx_cntr++] = readData;	//Copy the Received Byte to the Receive Buffer ARray
				if (rx_cntr > 250)				//If Received Bytes is more than 250, truncate it
				rx_cntr = 250;
				}
			}
		readStatus = UCSR0A;
		}
	}
// USART0 Transmitter interrupt service routine
ISR(USART_TX_vect)
{
	if(UCSR0A & DATA_REGISTER_EMPTY)	//Wait for Empty Transmit Buffer (UDRE0 bit)
	{
		if(tx_cntr <= tx_length)		//Transmitted bytes is less than the total bytes to be transmitted
		{
			UDR0 = Tx_Buf[tx_cntr++];	//Copy one Byte to the Serial Buffer and Increment the counter
		}
		else							//Finished transmitting all bytes
			{
			tx_length = 0;				//Reset the Tx Length Variable
			tx_cntr = 0;				//Reset the Transmitted Byte Counter
			rx_cntr = 0;				//Reset the Received Byte Counter
			//C_SETBIT(LED_BLUE);
			//C_CLEARBIT(LED_GREEN);
			C_CLEARBIT(TX_EN);			//Disable RS485 Transmission and Enable Reception
			comm_flags |= RESP_TIMER_RUN;
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

//MODBUS Query to Slave, to be used by Master Device [Slave Addr][Function Code][Start Addr Hi][Start Addr Lo][No of Reg Hi][No of Reg Lo][CRC Hi][CRC Lo]
void MBQuery(uint8_t Device_Id, uint8_t Fn_Code, uint16_t Reg_Addr, uint16_t Reg_Wr_Value, uint8_t Reg_Nos)
    {
    uint8_t Data_Count;
    uint16_t tempi; 

    Data_Count = 0;
    //Device ID
    Tx_Buf[Data_Count++] = Device_Id;
    //Function Code
    Tx_Buf[Data_Count++] = Fn_Code;
    //Register Address (4 bytes)
    Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Addr >> 8));
    Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Addr));
    

    if(Fn_Code == 0x03 || Fn_Code == 0x04)    // Number of Bytes if Function is Read  Holding Register,
        {
        Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Nos >> 8));
        Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Nos));
        }

    if(Fn_Code == 0x06)      //Data to be written if Function is Write  Holding Register,
        {
		Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Wr_Value >> 8));
		Tx_Buf[Data_Count++] = (uint8_t) (0x00FF & (Reg_Wr_Value));
        }
	Data_Count--;
	tempi = modbusCRC16 (Tx_Buf, Data_Count);
    
    Tx_Buf[Data_Count+1] = (uint8_t) (0x00FF & (tempi >> 8));
	Tx_Buf[Data_Count+2] = (uint8_t) (0x00FF & (tempi));;
	tx_length = Data_Count+2;
    tx_cntr = 0;
    rx_cntr =0;
	C_SETBIT(TX_EN); //Disable Reception and Enable Transmission
    UDR0 = Tx_Buf[tx_cntr++]; 
    }

//Decode the response received from Slave, to be used in Master Device
uint16_t decode_slave(void)
    {
    //  uint8 i;
      uint8_t bytes_no, tempi1,tempi2, i;
      uint16_t response;
    // PORTF |= 0x80;
    
	 if(comm_flags & NO_RESPONSE)        //Through Error if no response is received within 400ms
		{
		 //sprintf(lcd_buffer,"No Response");
		 //lcd_puts(lcd_buffer);
		 response = 101;
		 //return (101);			//No Response
		}
	 if(comm_flags & SERIAL_ERROR)        //Through Error if no response is received within 400ms
		{
		 //sprintf(lcd_buffer,"Serial Error");
		 //lcd_puts(lcd_buffer);
		 response = 102;
		 //return (102);			//No Response
		}
		
		
		
     if (comm_flags & SERIAL_BUSY)
		{ //PORTF |= 0x02;
        tempi1 = modbusCRC16 (Rx_Buf,rx_cntr-3);
        tempi2 = (int)Rx_Buf[rx_cntr - 2];           //Get MSB of CRC form received string
        tempi2 <<= 8;
        tempi2 |= (int)Rx_Buf[rx_cntr - 1];          //Get LSB of CRC form received string
/*
		lcd_gotoxy(0,0);
		sprintf(lcd_buffer,"RxCtr:%02d", rx_cntr);
		lcd_puts(lcd_buffer);
		_delay_ms(100);
		lcd_gotoxy(0,1);
		for(i=0;i<rx_cntr; i++)
			{
			sprintf(lcd_buffer,"%02x, ", Rx_Buf[i]);
			lcd_puts(lcd_buffer);
			}
		_delay_ms(2000);
*/
        if(tempi1 == tempi2)           //If telegram LRC matches with computed LRC, proceed
			{ 
			//    PORTF |= 0x03; 
			if (Rx_Buf[0] == Tx_Buf[0]) //If reply is from Same device as query
				{
				// PORTF |= 0x04;
				//sprintf(lcd_buffer,"Got it Right!");
				//lcd_puts(lcd_buffer);                                                       
				//function check
				
				if (Rx_Buf[1] & MODBUS_EXCEPTION_FLAG)	//Check for Exception (If Exception is there, the MSB is always 1.
					{
					comm_flags &= ~SERIAL_BUSY;
					response = Rx_Buf[2];
					//return (Rx_Buf[2]); //Return the Exception Code
					}
				else
					{
					response = 100;
					switch (Rx_Buf[1])
						{
						case 3:									//Read Holding Register
							bytes_no = (Rx_Buf[2]);				//Number of Register to be Read
							for (i=0;i<(bytes_no);i++)          //Copy all bytes to Rx_Value Buffer
								{
								Rx_Value[i] = Rx_Buf[3+i];  //Copy the String to Rx_Value, Read Value Starts from 4th Byte
								//Read_Value <<=8;
								//Read_Value |= (ascii2hex(Rx_Buf[7+i]));   //Convert String to Integer and Save in Read_Value
								}
							data_bytes = i;                             //Total Data Field in Rx_Value Array.
							//comm_flags &= ~SERIAL_BUSY;
							//return (100);		//Valid Response and Processed
						break;
						
						case 4:									//Read Holding Register
							bytes_no = (Rx_Buf[2]);				//Number of Register to be Read
							for (i=0;i<(bytes_no);i++)          //Copy all bytes to Rx_Value Buffer
							{
								Rx_Value[i] = Rx_Buf[3+i];  //Copy the String to Rx_Value, Read Value Starts from 4th Byte
								//Read_Value <<=8;
								//Read_Value |= (ascii2hex(Rx_Buf[7+i]));   //Convert String to Integer and Save in Read_Value
							}
							data_bytes = i;                             //Total Data Field in Rx_Value Array.
							//comm_flags &= ~SERIAL_BUSY;
							//return (100);		//Valid Response and Processed
						break;
              
						case 6:    
							for (i=0;i<4;i++)                //Reply for Function Code:6 is always the same String (4 data bytes)
								{
								if (Tx_Buf[i+2] != Rx_Buf[i+2])  //If the Tx Bytes is different from the Rx bytes, return Error
									//comm_flags &= ~SERIAL_BUSY;
									response = 105;
									//return (100);		//Valid Response and Processed
								}         
						break;
					
						}
					}
				}
			else
				{
				//sprintf(lcd_buffer,"Slave Mismatch");
				//lcd_puts(lcd_buffer);
				//return (103);	//If Slave ID Mismatch
				response = 103;
				//rx_cntr =0;
				}
			}
        
		else
			{
			//sprintf(lcd_buffer,"CRC Error");
			//lcd_puts(lcd_buffer);
			response = 104;
			//rx_cntr =0;
			//return (108);	//CRC Mismatch
			}
		//rx_cntr = 0;
		comm_flags &= ~SERIAL_BUSY;
		}
	rx_cntr = 0;
	return (response);
	}



void housekeeping(void)			//Running @2000Hz in Timer
    {
	//uint8_t i; 
    if (comm_flags & SERIAL_ERROR)     //On Serial Error reset all comm parameters
		{
        comm_flags &= ~SERIAL_ERROR;	//Clear the Serial Error Flag
        //resp_timeout = 0;
		//frame_timeout = 0;
        rx_cntr = 0;
        }
     else
        {
        if (comm_flags & TIMER_RUN)				//Run frame timer only if the timer is set
			{
			frame_timeout++;					//Increment the Frame Timer
            if (frame_timeout > 20)            //Close Packet after 10ms of Inactivity (1ms = 2 counts)
				{ 
                comm_flags &= ~TIMER_RUN;		//Stop the Frame Timer
				comm_flags &= ~RESP_TIMER_RUN;	//Stop the Response Timer
				resp_timeout = 0;				//Reset the Time Out Counter
				frame_timeout = 0;				//Reset the Time Out Counter
				comm_flags &= ~NO_RESPONSE;		//Clear the No-Response Flag
				comm_flags |= SERIAL_BUSY;		//Set the Serial Busy Flag
				C_SETBIT(TX_EN); //Disable Reception and Enable Transmission
				}		
			}
		if (comm_flags & RESP_TIMER_RUN)		//Run response timer only if the Response Timer is activated
			{
			resp_timeout++;						//Increment the Response Timer@0.5ms
			if (resp_timeout > 800)				//Set No-Response Flag after 400ms of no answer (1ms = 2 counts)
				{
				comm_flags &= ~RESP_TIMER_RUN;	//Stop the Response Timer
				resp_timeout = 0;				//Reset the Time Out Counter
				comm_flags |= NO_RESPONSE;		//Set the No-Response Flag 
				//rx_cntr=0;						//Reset the Rx Counter
				C_SETBIT(TX_EN); //Disable Reception and Enable Transmission
				}
			}
		}
	}