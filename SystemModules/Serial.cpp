#include "Serial.hpp"

#include "serial_task.h"


void Serial::begin(void){
	serial_task_start();
	isBegin = true;
}

void Serial::end(void){}

void Serial::flush(void){
	if(isBegin)
		serial_clear_tx_buffer();
}

int Serial::available(void){
	if(isBegin)
		return (int)serial_available_bytes();
	else
		return 0;
}

int Serial::availableForWrite(void){
	if(isBegin)
		return (int)serial_available_bytes_for_write();
	else
		return 0;
}

int Serial::print(std::string msg){
	if(isBegin)
		return serial_putbytes((uint8_t*)msg.c_str(), msg.size());
	else
		return 0;
}


int Serial::print(uint8_t data){
	if(isBegin)
		return serial_putbyte(data);
	else
		return 0;
}
int Serial::print(float data){
	memset(sendBuf, 0, sizeof(sendBuf));
	sprintf(sendBuf, "%f", data);
	
	if(isBegin)
		return serial_putbytes((uint8_t*)&sendBuf, strlen(sendBuf));
	else
		return 0;
}
int Serial::print(int data){
	memset(sendBuf, 0, sizeof(sendBuf));
	sprintf(sendBuf, "%d", data);
	
	if(isBegin)
		return serial_putbytes((uint8_t*)&sendBuf, strlen(sendBuf));
	else
		return 0;
}
	
int Serial::println(std::string msg){
	msg += "\n";
	
	return print(msg);
}
int Serial::println(uint8_t data){
	if(isBegin){
		memset(sendBuf, 0, sizeof(sendBuf));
		sprintf(sendBuf, "%u", data);
		
		serial_putbyte(data);
		serial_putbyte('\n');
		return 2;
	}
	else
		return 0;
}
int Serial::println(float data){
	memset(sendBuf, 0, sizeof(sendBuf));
	sprintf(sendBuf, "%f\n", data);
	
	if(isBegin){
		serial_putbytes((uint8_t*)&sendBuf, strlen(sendBuf));
		return sizeof(data) + 1;
	}
	else
		return 0;
}
int Serial::println(int data){
	memset(sendBuf, 0, sizeof(sendBuf));
	sprintf(sendBuf, "%d\n", data);
	
	if(isBegin){
		serial_putbytes((uint8_t*)sendBuf, strlen(sendBuf));
		return sizeof(data) + 1;
	}
	else
		return 0;
}
	
int Serial::write(uint8_t data){
	
	
}
int Serial::write(std::string str){}
	
int Serial::write(uint8_t* pData, uint16_t len){
	serial_putbytes((uint8_t*)pData, len);
	return len;
}
	
uint8_t Serial::read(void){}
uint8_t Serial::readBytes(uint8_t* pBuf, uint16_t len){}
uint8_t Serial::readBytesUntil(uint8_t ch, uint8_t* pBuf, uint16_t len){}

uint8_t Serial::peek(void){}

/* call when if(Class)*/
Serial::operator bool() const{}
			
	