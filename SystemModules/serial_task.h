#ifndef _SERIAL_TASK_H
#define _SERIAL_TASK_H

#ifdef __cplusplus
extern "C" 
{
#endif

#include "main.h"
#include "printf.h"

#define SERIAL_TX_BUFFER_SIZE   256
#define SERIAL_RX_BUFFER_SIZE   256

void serial_task_start(void);

int serial_putbyte(uint8_t _b);
int serial_putbytes(uint8_t* pData, uint16_t size);
size_t serial_getbyte(uint8_t* pData);
size_t serial_getbytes(uint8_t* pData, uint16_t size);
size_t serial_available_bytes(void);
size_t serial_available_bytes_for_write(void);

void serial_clear_tx_buffer(void);

#ifdef __cplusplus
}
#endif


#endif
