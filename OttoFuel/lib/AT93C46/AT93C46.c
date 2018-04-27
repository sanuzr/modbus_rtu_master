/*
  NVSRAM2 CHIP SELECT CHANGED TO P1^1

*/
#include "AT93C46.h"

#define GAS_NAME_1	1, 1		//EEPROM Address for Gas Name: Block1 (Char 1, 2)
#define GAS_NAME_2	1, 2		//EEPROM Address for Gas Name: Block2 (Char 3, 4)
#define GAS_NAME_3	1, 3		//EEPROM Address for Gas Name: Block3 (Char 5, 6)
#define GAS_NAME_4	1, 4		//EEPROM Address for Gas Name: Block4 (Char 7, 8)

#define CAL_DD		1, 18		//EEPROM Address for Last Calibration Date
#define CAL_MM		1, 19		//EEPROM Address for Last Calibration Month
#define CAL_YY		1, 20		//EEPROM Address for Last Calibration Year
#define CAL_STD		1, 11		//EEPROM Address for Calibration Standard Value
#define CAL_OFF_CNT	1, 12		//EEPROM Address for Calibration Offset Counts
#define CAL_CNTS	1, 13		//EEPROM Address for Calibration Counts (for Applied Standard)

#define RANGE_HI	1, 14		//EEPROM Address for Sensor High Range
#define ADD_TLV		1, 15		//EEPROM Address for TLV Value
#define ADD_STLV	1, 16		//EEPROM Address for TLV Value
#define SET_TLV(val)	nvram_write(ADD_TLV, (val))
#define SET_STLV(val)	nvram_write(ADD_STLV, (val))
#define SET_ZCNT(val)	nvram_write(CAL_OFF_CNT, (val))
#define SET_SCNT(val)	nvram_write(CAL_CNTS, (val))
#define SET_STD(val)	nvram_write(CAL_STD, (val))
#define SET_CDD(val)	nvram_write(CAL_DD, (val))
#define SET_CMM(val)	nvram_write(CAL_MM, (val))
#define SET_CYY(val)	nvram_write(CAL_YY, (val))


#define DIGIT_INFO	1, 10		//EEPROM Address for Digit Information (LSB: No of Digits, LSB: Decimal Points)

#define MFG_DD		1, 21		//EEPROM Address for Manufacturing Date
#define MFG_MM		1, 22		//EEPROM Address for Manufacturing  Month
#define MFG_YY		1, 23		//EEPROM Address for Manufacturing  Year

#define SS_SR_NO_1	1, 35		//EEPROM Address for Smart Sensor Serial Number First two Digits
#define SS_SR_NO_2	1, 36		//EEPROM Address for Smart Sensor Serial Number Second two Digits


void nvram_write_en(unsigned char b, bool p);
void nvram_write(unsigned char b, unsigned char c, unsigned int i);
unsigned int nvram_read(unsigned char b, unsigned char c);

void nvram_write_en(unsigned char b, bool p)
{
	unsigned char c, d, e;
	int t;

	if (p == 1)
	{
		c = 0xc0;
	}
	else
	{
		c = 0x00;
	}
	C_CLEARBIT(NVSCLK);				// SCK = Low;
	C_SETBIT(NVDI);					// = ON;
	switch (b)
	{
		case 1:
			C_SETBIT(NVCSRAM1);				// = ON;
			break;
		
		case 2:
			C_SETBIT(NVCSRAM2);			// = ON;
			break;
		
	}
	C_SETBIT(NVSCLK);				// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);					// = OFF;
	C_CLEARBIT(NVDI);				// = OFF;
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);				// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);				// = OFF;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);				// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);				// = OFF;
	d = 8;
	e = 128;
	while(d != 0)
	{
		if((c & e) == 0)
		{
			C_CLEARBIT(NVDI);		// = OFF;
		}
		else
		{
			C_SETBIT(NVDI);				// = ON;
		}
		C_SETBIT(NVSCLK);				// = ON;
		asm("nop");//_nop_();
		//_nop_();
		C_CLEARBIT(NVSCLK);			// = OFF;
		e = e / 2;
		d--;
	}
	for(t=0; t<=10000; t++)
	{
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		asm("nop");//_nop_();
	}
	switch (b)
	{
		case 1:
			C_CLEARBIT(NVCSRAM1);				// = OFF;
			break;
		
		case 2:
			C_CLEARBIT(NVCSRAM2);		// = OFF;
			break;
		
	}
}

void nvram_write(unsigned char b, unsigned char c, unsigned int i)
{
	unsigned char d, e;

	C_CLEARBIT(NVSCLK);			// = OFF;
	C_SETBIT(NVDI);				// = ON;
	switch (b)
	{
		case 1:
			C_SETBIT(NVCSRAM1);				// = ON;
			break;
		
		case 2:
			C_SETBIT(NVCSRAM2);			// = ON;
			break;
		
	}
	C_SETBIT(NVSCLK);// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	C_CLEARBIT(NVDI);// = OFF;
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);		// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	C_SETBIT(NVDI);// = ON;
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);	// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	d = 6;
	e = 32;
	while(d != 0)
	{
		if((c & e) == 0)
		{
			C_CLEARBIT(NVDI);// = OFF;
		}
		else
		{
			C_SETBIT(NVDI);// = ON;
		}
		C_SETBIT(NVSCLK);// = ON;
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
		e = e / 2;
		d--;
	}
	c = i / 0x100;
	d = 8;
	e = 128;
	while(d != 0)
	{
		if((c & e) == 0)
		{
			C_CLEARBIT(NVDI);// = OFF;
		}
		else
		{
			C_SETBIT(NVDI);	// = ON;
		}
		C_SETBIT(NVSCLK);	// = ON;
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		C_CLEARBIT(NVSCLK);	// = OFF;
		e = e / 2;
		d--;
	}
	c = i % 0x100;
	d = 8;
	e = 128;
	while(d != 0)
	{
		if((c & e) == 0)
		{
			C_CLEARBIT(NVDI);// = OFF;
		}
		else
		{
			C_SETBIT(NVDI);// = ON;
		}
		C_SETBIT(NVSCLK);// = ON;
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
		e = e / 2;
		d--;
	}
	switch (b)
	{
		case 1:
			C_CLEARBIT(NVCSRAM1);// = OFF;
			break;
		
		case 2:
			C_CLEARBIT(NVCSRAM2);// = OFF;
			break;
		
	}
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	switch (b)
	{
		case 1:
			C_SETBIT(NVCSRAM1);// = ON;
			break;
		
		case 2:
			C_SETBIT(NVCSRAM2);		// = ON;
			break;
		
	}
	while(C_CHECKBIT(PIN_NVDO))
	{
		C_SETBIT(NVSCLK);// = ON;
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
	}
	for(i=0; i<=1000; i++)
	{
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		asm("nop");//_nop_();
		asm("nop");//_nop_();
	}
	switch (b)
	{
		case 1:
			C_CLEARBIT(NVCSRAM1);// = OFF;
			break;
		
		case 2:
			C_CLEARBIT(NVCSRAM2);// = OFF;
			break;
		
	}
}

unsigned int nvram_read(unsigned char b, unsigned char c)
{
	unsigned char d, e;
	unsigned int i;

	i = 0;
	C_CLEARBIT(NVSCLK);// = OFF;
	C_SETBIT(NVDI);// = ON;
	//C_SETBIT(NVDO);// = ON;
	switch (b)
	{
		case 1:
			C_SETBIT(NVCSRAM1);// = ON;
			break;
		
		case 2:
			C_SETBIT(NVCSRAM2);// = ON;
			break;
		
	}
	C_SETBIT(NVSCLK);// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	C_CLEARBIT(NVDI);// = OFF;
	asm("nop");//_nop_();
	C_SETBIT(NVSCLK);// = ON;
	asm("nop");//_nop_();
	asm("nop");//_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;

	d = 6;
	e = 32;
	while(d != 0)
	{
		if((c & e) == 0)
		{
			C_CLEARBIT(NVDI);// = OFF;
		}
		else
		{
			C_SETBIT(NVDI);// = ON;
		}
		C_SETBIT(NVSCLK);// = ON;
		asm("nop");  //_nop_();
		asm("nop");  //_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
		e = e / 2;
		d--;
	}
	C_SETBIT(NVSCLK);// = ON;
	asm("nop");  //_nop_();
	asm("nop");  //_nop_();
	C_CLEARBIT(NVSCLK);// = OFF;
	//C_SETBIT=(NVDO);// = ON;
	d = 8;
	e = 128;
	while(d != 0)
	{
		if(C_CHECKBIT(PIN_NVDO))			//if(PIN(NVDO) == 1)
		{
			i = i + (e * 0x100);
		}
		C_SETBIT(NVSCLK);// = ON;
		asm("nop");  //_nop_();
		asm("nop");  //_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
		e = e / 2;
		d--;
	}
	d = 8;
	e = 128;
	while(d != 0)
	{
		if(C_CHECKBIT(PIN_NVDO))//if(PIN(NVDO) == 1)
		{
			i = i + e;
		}
		C_SETBIT(NVSCLK);		// = ON;
		asm("nop");  //_nop_();
		asm("nop");  //_nop_();
		C_CLEARBIT(NVSCLK);// = OFF;
		e = e / 2;
		d--;
	}
	switch (b)
	{
		case 1:
			C_CLEARBIT(NVCSRAM1);// = OFF;
			break;
		
		case 2:
			C_CLEARBIT(NVCSRAM2);// = OFF;
			break;
		
	}
	return(i);
}
