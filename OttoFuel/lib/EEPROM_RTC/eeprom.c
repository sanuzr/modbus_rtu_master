#include "eeprom.h"
#include <avr/io.h>
#include <util/delay.h>

//I2C EEPROM Setting

#define EEPROM_ADD		   0xA0    //EEPROM IC Address
#define FIRST_RUN		   0x03    //Byte to detect first Run of the Instrument  2 Bytes (4,5)
#define ADDR_MA_4		   0x05    //4mA Counts for the Current Driver 2 Bytes (5,6)
#define ADDR_MA_20		   0x07    //20mA Counts for the Current Driver 2 Bytes (7,8)
#define ADD_CNTR		   0x09    //Address Counter 2bytes (9,10)
#define SR_NO			   0x0B    //Sr No Number 2 bytes (11,12)
#define RESERVED		   64    //0-63(64 bytes)/1 page is reserved) 0x40 =64
 
 
 // -------------BCD functions-----------
 
 char bin2bcd(char num)
 {
	 return ((num/10 * 16) + (num % 10));
 }
 
 char bcd2bin(char num)
 {
	 return ((num/16 * 10) + (num % 16));
 }
 
 
//-------------TWI Functions-------------

void TWIInit(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
    //enable TWI
    TWCR = (1<<TWEN);
}

//send start signal
void TWIStart(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal with 10ms delay
void TWIStop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
    _delay_ms(10);
}

//send stop signal w/o delay
void TWIStopADC(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	_delay_ms(1);
}

void TWIWrite(unsigned char data)
{
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//read byte with ACK
unsigned char TWIReadACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

//read byte with NACK
unsigned char TWIReadNACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
//	_delay_ms(10);
    return TWDR;
}

unsigned char TWIGetStatus(void)
{
    unsigned char status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}


//-------------RTC DS1307 Functions-------------

unsigned char rtc_read(unsigned char address)
{
    unsigned char data;
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(address);
    TWIStop(); // 09022011_1 
    //_delay_ms(10);
    TWIStart();
    TWIWrite(0xd1);
    data=TWIReadNACK();
    TWIStop();
    return data;
}

void rtc_write(unsigned char address,unsigned char data)
{
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(address);
    TWIWrite(data);
    TWIStop();
}

void rtc_init(unsigned char rs,unsigned char sqwe,unsigned char out)
{
    rs&=3;
    if (sqwe) rs|=0x10;
    if (out) rs|=0x80;
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(7);
    TWIWrite(rs);
    TWIStop();
}

void RtcGetTime(unsigned char *hour,unsigned char *min,unsigned char *sec)
{
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(0);
    TWIStop(); // 09022011_1
    //_delay_ms(10);
    TWIStart();
    TWIWrite(0xd1);
    *sec=bcd2bin(TWIReadACK());
    *min=bcd2bin(TWIReadACK());
    *hour=bcd2bin(TWIReadNACK());
    TWIStop();
}

void RtcSetTime(unsigned char hour,unsigned char min,unsigned char sec)
{
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(0);
    TWIWrite(bin2bcd(sec));
    TWIWrite(bin2bcd(min));
    TWIWrite(bin2bcd(hour));
    TWIStop();
}

void RtcGetDate(unsigned char *week_day,unsigned char *day,unsigned char *month,unsigned char *year)
{
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(3); // 09022011_1
    TWIStop(); // 09022011_1
    //_delay_ms(10);
    TWIStart();
    TWIWrite(0xd1);
    *week_day=TWIReadACK(); // 09022011_1
    *day=bcd2bin(TWIReadACK());
    *month=bcd2bin(TWIReadACK());
    *year=bcd2bin(TWIReadNACK());
    TWIStop();
}

void RtcSetDate(unsigned char week_day,unsigned char day,unsigned char month,unsigned char year)
{
    TWIStart();
    TWIWrite(0xd0);
    TWIWrite(3); // 09022011_1
    TWIWrite(week_day); // 09022011_1
    TWIWrite(bin2bcd(day));
    TWIWrite(bin2bcd(month));
    TWIWrite(bin2bcd(year));
    TWIStop();
}




//-------------I2C EEPROM Functions 24C256/512-------------
//Write Functions
void EepromByteWrite(unsigned char dev_id, unsigned int add, unsigned char *data)
{
    unsigned char addl,addh; 
    addh= (unsigned char) ((add & 0xFF00) >>8);
    addl= (unsigned char) (add & 0x00FF);
    //dev_id = (dev_id|0);
    TWIStart();
    TWIWrite(dev_id);
    TWIWrite(addh);
    TWIWrite(addl);
    TWIWrite(*data);
    TWIStop();
}
        
//Page Write  (For Error Free Operation No. of Bytes should be multiple of 8)     
void EepromPageWrite(unsigned char dev_id, unsigned int add, unsigned char *data, unsigned char bytes)
{
    unsigned char i, addl,addh; 
    addh= (unsigned char) ((add & 0xFF00) >>8);
    addl= (unsigned char) (add & 0x00FF);
    TWIStart();
    TWIWrite(dev_id);
    TWIWrite(addh);
    TWIWrite(addl);
    for(i=0;i<bytes;i++)
        {
        TWIWrite(*data++);
        }
    TWIStop();
} 

//Read Functions      
void EepromByteRead(unsigned char dev_id, unsigned int add, unsigned char *data)
{
	unsigned char addl,addh;
	addh= (unsigned char) ((add & 0xFF00) >>8);
	addl= (unsigned char) (add & 0x00FF);
	TWIStart();
	TWIWrite(dev_id);
	TWIWrite(addh);
	TWIWrite(addl);
	TWIStart();
	TWIWrite((dev_id|0x01));
	*data = TWIReadNACK();
	TWIStop();
}
   
//Page Read     
void EepromPageRead(unsigned char dev_id, unsigned int add, unsigned char *data, unsigned char bytes)
{
    unsigned char addl,addh;
    addh= (unsigned char) ((add & 0xFF00) >>8);
    addl= (unsigned char) (add & 0x00FF);
    TWIStart();
    TWIWrite(dev_id);
    TWIWrite(addh);
    TWIWrite(addl);
    TWIStart();
    TWIWrite((dev_id|0x01));
//        for (i=0;i<bytes;i++)
//            {
//            if(i==bytes-1)
//            *data= TWIReadNACK();
//            else 
//            *data++=TWIReadACK();
//            }
    while(--bytes)
        {
        *data++=TWIReadACK();
        //bytes--;
        }
    *data= TWIReadNACK();
    TWIStop();
}

//Writing Multiple Bytes of Data without handling the address (Multiple of 16 bytes are allowed)
unsigned char EepromWrite(unsigned char *data, unsigned char bytes)
{
	unsigned char i, addl, addh;
	unsigned int sr_no, adr;
	EepromPageRead(EEPROM_ADD, ADD_CNTR, (unsigned char *) &adr, 2); 
	_delay_ms(10); 
	EepromPageRead(EEPROM_ADD, SR_NO, (unsigned char *) &sr_no, 2); 
	_delay_ms(10);

if (adr >=32500)  // MAximum Storage for 24C256 is 256Kb => 32 KB => 32 x 1024 = 
    {
    return 1;
    }
else
    {
    addh= (unsigned char) ((adr & 0xFF00) >>8);
    addl= (unsigned char) (adr & 0x00FF);
    if(bytes ==1)
        { 
        //dev_id = (dev_id|0);
        TWIStart();
        TWIWrite(EEPROM_ADD);
        TWIWrite(addh);
        TWIWrite(addl);
        TWIWrite(*data);
        TWIStop();
        //Above code can be replaced with the EepromPageWrite Function with proper argument handling
        //EepromByteWrite(EEPROM_ADD, adr,  (unsigned char *) &*dadr);
        //_delay_ms(10);
        adr+=1;;
        EepromPageWrite(EEPROM_ADD, ADD_CNTR,  (unsigned char *) &adr, 2);
        _delay_ms(10);
        return 0;
        }
    else
        {
        TWIStart();
        TWIWrite(EEPROM_ADD);
        TWIWrite(addh);
        TWIWrite(addl);
        for(i=0;i<bytes;i++)
            {
            TWIWrite(*data++);
            }
        TWIStop();
        //Above code can be replaced with the EepromPageWrite Function with proper argument handling
        //EepromPageWrite(EEPROM_ADD, adr,  (unsigned char *) &*data, bytes);
        //_delay_ms(10);
        adr+=bytes;
        sr_no+=1;
        EepromPageWrite(EEPROM_ADD, ADD_CNTR,  (unsigned char *) &adr,2);
        _delay_ms(10);
        EepromPageWrite(EEPROM_ADD, SR_NO,  (unsigned char *) &sr_no,2);
        _delay_ms(10);
        return 0;
        } 
    }
}
    
void EepromErase(void)
{
unsigned int adr = RESERVED, sr_no=0;
_delay_ms(10);

EepromPageWrite(EEPROM_ADD, ADD_CNTR,  (unsigned char *) &adr, 2);		_delay_ms(100);

EepromPageWrite(EEPROM_ADD, SR_NO,  (unsigned char *) &sr_no, 2);		_delay_ms(100);
}


void DAC_Write(unsigned char addr, unsigned int value)
{
    unsigned char hValue, lValue;
    lValue = (unsigned char) (value);
    hValue = (unsigned char) ((value>>8) & (0x0F));
    TWIStart();
    TWIWrite(addr);
    TWIWrite(hValue);
    TWIWrite(lValue);
    TWIStop();
}