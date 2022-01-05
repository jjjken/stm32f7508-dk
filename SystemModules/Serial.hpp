#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "StreamAbstract.hpp"


class Serial: private StreamAbstract
{
public:
		Serial(){}
		~Serial(){}

		virtual void begin(void);
		virtual void end(void);
		
		virtual void flush(void);
		virtual int available(void);
		virtual int availableForWrite(void);
		
		virtual int print(std::string msg);
		virtual int print(uint8_t data);
		virtual int print(float data);
		virtual int print(int data);
			
		virtual int println(std::string msg);
		virtual int println(uint8_t data);
		virtual int println(float data);
		virtual int println(int data);
			
		virtual int write(uint8_t data);
		virtual int write(std::string str);
		virtual int write(uint8_t* pData, uint16_t len);
			
		virtual uint8_t read(void);
		virtual uint8_t readBytes(uint8_t* pBuf, uint16_t len);
		virtual uint8_t readBytesUntil(uint8_t ch, uint8_t* pBuf, uint16_t len);
		
		virtual uint8_t peek(void);

		/* call when if(Class)*/
		virtual operator bool() const;

private:
		bool isBegin;
		char sendBuf[10];

};

extern Serial serial1;

#endif
