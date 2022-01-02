#ifndef _COM_H
#define _COM_H

#ifdef __cplusplus
extern "C" 
{
#endif

#include "main.h"
#include "stm32f7508_discovery.h"

#define COM_PORT_NUMBER COM1
#define COM_PORT_HANDLE (&huart1)

struct com_struct{
	UART_HandleTypeDef* uart_handle;
	void (*tx_complete_callback_func)(void);
	void (*rx_complete_callback_func)(void);
	uint8_t* rx_buffer;
	uint16_t size_of_rx_buffer;
	bool use_dma;
};

extern UART_HandleTypeDef huart1;

void com_init(uint8_t* rx_buffer, void(*rx_callback)(void), void(*tx_callback)(void));
void com_deinit(void);

int com_send(uint8_t* pData, uint16_t size);
void com_register_rx_complete_callback_fucntion(void (*cb)(void));
void com_register_tx_complete_callback_fucntion(void (*cb)(void));
void com_receive_enable(void);

#ifdef __cplusplus
}
#endif


#endif
