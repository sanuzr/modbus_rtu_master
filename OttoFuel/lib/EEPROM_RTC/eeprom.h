/*
* eeprom.h
*/ 
#ifndef I2C_EEPROM_H_INCLUDED
#define I2C_EEPROM_H_INCLUDED

char bin2bcd(char num);
char bcd2bin(char num);
void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
void TWIStopADC(void);
void TWIWrite(unsigned char data);
unsigned char TWIReadACK(void);
unsigned char TWIReadNACK(void);
unsigned char TWIGetStatus(void);

void RtcGetTime(unsigned char *hour,unsigned char *min,unsigned char *sec);
void RtcSetTime(unsigned char hour,unsigned char min,unsigned char sec);
void RtcGetDate(unsigned char *week_day,unsigned char *day,unsigned char *month,unsigned char *year);
void RtcSetDate(unsigned char week_day,unsigned char day,unsigned char month,unsigned char year);

void EepromByteWrite(unsigned char dev_id, unsigned int add, unsigned char *data);
void EepromPageWrite(unsigned char dev_id, unsigned int add, unsigned char *data, unsigned char bytes);
void EepromByteRead(unsigned char dev_id, unsigned int add, unsigned char *data);
void EepromPageRead(unsigned char dev_id, unsigned int add, unsigned char *data, unsigned char bytes);
unsigned char EepromWrite(unsigned char *data, unsigned char bytes);
void EepromErase(void);

#endif