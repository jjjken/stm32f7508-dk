#ifndef STREAMABSTRACT_HPP
#define STREAMABSTRACT_HPP

#include <string>

#include "main.h"


class StreamAbstract
{
	public:
		StreamAbstract() {}
		~StreamAbstract() {}
		
		virtual void begin(void){}
		virtual void end(void){}
		
		virtual void flush(void) {}
		virtual int available(void) = 0;
		virtual int availableForWrite(void) = 0;
		
		virtual int print(std::string msg)= 0;
		virtual int print(uint8_t data)= 0;
		virtual int print(float data)= 0;
		virtual int print(int data)= 0;
			
		virtual int println(std::string msg) = 0;
		virtual int println(uint8_t data) = 0;
		virtual int println(float data) = 0;
		virtual int println(int data) = 0;
			
		virtual int write(uint8_t data) = 0;
		virtual int write(std::string str) = 0;
		virtual int write(uint8_t* pData, uint16_t len) = 0;
			
		virtual uint8_t read(void) = 0;
		virtual uint8_t readBytes(uint8_t* pBuf, uint16_t len) = 0;
		virtual uint8_t readBytesUntil(uint8_t ch, uint8_t* pBuf, uint16_t len) = 0;
		
		virtual uint8_t peek(void) = 0;

		/* call when if(Class)*/
		virtual operator bool() const{}
};


#endif
