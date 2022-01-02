#include "com.h"

static uint8_t* _rx_data;

UART_HandleTypeDef huart1 = {0};

void (*_rx_complete_callback_fucntion)(void);
void (*_tx_complete_callback_function)(void);

void com_init(uint8_t* rx_buffer, void(*rx_callback)(void), void(*tx_callback)(void)){
    
    if(rx_buffer)
        _rx_data = rx_buffer;
    
    if(rx_callback)
        _rx_complete_callback_fucntion = rx_callback;

    if(tx_callback)
        _tx_complete_callback_function = tx_callback;

    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    BSP_COM_Init(COM_PORT_NUMBER, &huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

}

void com_deinit(void){

    BSP_COM_DeInit(COM_PORT_NUMBER, &huart1);
}

int com_send(uint8_t* pData, uint16_t size){

    HAL_StatusTypeDef ret = HAL_ERROR;

    ret = HAL_UART_Transmit_IT(COM_PORT_HANDLE, pData, size);

    return ret == HAL_OK ? 0 : -1;
}

void com_register_rx_complete_callback_fucntion(void (*cb)(void)){

    _rx_complete_callback_fucntion = cb;
}

void com_register_tx_complete_callback_fucntion(void (*cb)(void)){

    _tx_complete_callback_function = cb;
}

void com_receive_enable(void){

    HAL_UART_Receive_IT(COM_PORT_HANDLE, _rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){

    if(UartHandle->Instance == huart1.Instance){
        if(_rx_complete_callback_fucntion){
            _rx_complete_callback_fucntion();
        }

        com_receive_enable();
    }
}

void HAL_UART_TxCpltCallBack(UART_HandleTypeDef *UartHandle){

    if(UartHandle->Instance == huart1.Instance){
        if(_tx_complete_callback_function){
            _tx_complete_callback_function();
        }
    }
}
