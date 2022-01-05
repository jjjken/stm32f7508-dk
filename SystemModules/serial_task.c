#include "serial_task.h"
#include "cmsis_os.h"

#include "com.h"

#include "system_globals.h"

#define SEND_BUFFER_SIZE    64
#define SERIAL_TRANSMIT_TIMEOUT ((TickType_t)10)    //10ms

//Variables for the task
StaticTask_t _serial_task_struct;
StackType_t _serial_task_stack[TASK_SERIAL_STACK_SIZE];
TaskHandle_t _serial_task_handle;

//Variables for the UART data transmit stream
static uint8_t _serial_msg_tx_stream_storage[ SERIAL_TX_BUFFER_SIZE ];
StaticStreamBuffer_t _serial_msg_tx_stream_struct;
StreamBufferHandle_t _serial_msg_tx_stream_handle;

//Variables for the UART data receive stream
static uint8_t _serial_msg_rx_stream_storage[ SERIAL_RX_BUFFER_SIZE ];
StaticStreamBuffer_t _serial_msg_rx_stream_struct;
StreamBufferHandle_t _serial_msg_rx_stream_handle;

//Variables for the UART TX complete semaphore
SemaphoreHandle_t _serail_tx_complete_semaphore_handle;
StaticSemaphore_t _serial_tx_complete_semaphore_struct;

static void _serial_task(void const *arg);

static void uart_rx_callback(void);
static void uart_tx_callback(void);

//retarget the _putchar
void _putchar(char character){
    serial_putbyte((uint8_t)character);
}

static uint8_t _rx_data;

void serial_task_start(void){
    const size_t xTriggerLevel = 1;

    com_init(&_rx_data, uart_rx_callback, uart_tx_callback);
	
    _serial_msg_tx_stream_handle =
        xStreamBufferCreateStatic(					
            SERIAL_TX_BUFFER_SIZE,
            xTriggerLevel,
            _serial_msg_tx_stream_storage,
            &_serial_msg_tx_stream_struct);
	
    _serial_msg_rx_stream_handle =
        xStreamBufferCreateStatic(					
            SERIAL_RX_BUFFER_SIZE,
            xTriggerLevel,
            _serial_msg_rx_stream_storage,
            &_serial_msg_rx_stream_struct);

    _serail_tx_complete_semaphore_handle = 
        xSemaphoreCreateBinaryStatic(&_serial_tx_complete_semaphore_struct);

    _serial_task_handle = 
        xTaskCreateStatic(
            (TaskFunction_t)_serial_task,   //Function implement the task
            TASK_NAME_SERIAL,               //name
            TASK_SERIAL_STACK_SIZE,         //Stack size
            NULL,                           //Parameter to the task
            tskIDLE_PRIORITY,               //Priority
            _serial_task_stack,             //Arry use as the task stack
            &_serial_task_struct);          //handle for the task
                                
    configASSERT(_serail_tx_complete_semaphore_handle);                           
    configASSERT(_serial_msg_tx_stream_handle);                                
    configASSERT(_serial_msg_rx_stream_handle);
    configASSERT(_serial_task_handle);

}

static void _serial_task(void const *arg){

    uint8_t _send_buf[SEND_BUFFER_SIZE] = {0};
    size_t _qty_to_send;
    size_t _number_in_stream;

    com_receive_enable();

    for(;;){
        memset(_send_buf, 0, SEND_BUFFER_SIZE);
        _number_in_stream = xStreamBufferBytesAvailable(_serial_msg_tx_stream_handle); //get the number of items in the queue

        if(_number_in_stream > 0){
            _qty_to_send = _number_in_stream > SEND_BUFFER_SIZE ?  SEND_BUFFER_SIZE : _number_in_stream;
            
            xStreamBufferReceive(_serial_msg_tx_stream_handle, _send_buf, _qty_to_send, 0);

            com_send(_send_buf, _qty_to_send);

            if(xSemaphoreTake(_serail_tx_complete_semaphore_handle, SERIAL_TRANSMIT_TIMEOUT) != pdTRUE){
                //Add code here to handle the transmit timeout
            }
        }

        vTaskDelay((TickType_t )1); //Delay 1ms
    }
}


/**
 * @brief   put single character to the queue
 * 
 * @param   b character
 * @return  int -1: failed
 *              1: ok
 */
int serial_putbyte(uint8_t _b){

    portBASE_TYPE taskWoken = pdFALSE;

    if (__get_IPSR() != 0) {
        if(xStreamBufferSendFromISR(_serial_msg_tx_stream_handle, &_b, 1, &taskWoken) != pdPASS) {
            portEND_SWITCHING_ISR(taskWoken);
            return -1;
        }
    }
    else {
        if(xStreamBufferSend(_serial_msg_tx_stream_handle, &_b, 1, 0) != pdPASS) {
            return -1;
        }
    }

    return 1;
}

/**
 * @brief put the bytes to the buffer
 *  
 * @param pData pointer to the data
 * @param size  size of data needs to be put to the buffer
 * @return int  number put to the buffer
 */
int serial_putbytes(uint8_t* pData, uint16_t size){

    portBASE_TYPE taskWoken = pdFALSE;

    if (__get_IPSR() != 0) {
        if(xStreamBufferSendFromISR(_serial_msg_tx_stream_handle, pData, size, &taskWoken) != pdPASS) {
            portEND_SWITCHING_ISR(taskWoken);
            return -1;
        }
    }
    else {
        if(xStreamBufferSend(_serial_msg_tx_stream_handle, pData, size, 0) != pdPASS) {
            return -1;
        }
    }

    return size;
}

/**
 * @brief get single byte from the RX buffer
 * 
 * @param pData     pointer to the data buffer
 * @return size_t   number to data received
 */
size_t serial_getbyte(uint8_t* pData){

    portBASE_TYPE taskWoken = pdFALSE;

    if (__get_IPSR() != 0) {
        if(xStreamBufferReceiveFromISR(_serial_msg_rx_stream_handle, pData, 1, &taskWoken) != pdPASS) {
            portEND_SWITCHING_ISR(taskWoken);
            return 0;
        }
    }
    else {
        if(xStreamBufferReceive(_serial_msg_rx_stream_handle, pData, 1, 0) != pdPASS) {
            return 0;
        }
    }

    return 1;
}   

/**
 * @brief get multiple number of bytes from the RX buffer
 * 
 * @param pData pointer to the data
 * @param size  maximum number of bytes to receive
 * @return size_t number of data received
 */
size_t serial_getbytes(uint8_t* pData, uint16_t size){
    
    size_t _number_to_read = 0;
    size_t _number_of_data_in_stream = 0;
    portBASE_TYPE taskWoken = pdFALSE;

    _number_of_data_in_stream = xStreamBufferBytesAvailable(_serial_msg_rx_stream_handle);
    _number_to_read = (size > _number_of_data_in_stream) ? _number_of_data_in_stream : size;

    if (__get_IPSR() != 0) {
        if(xStreamBufferReceiveFromISR(_serial_msg_rx_stream_handle, pData, _number_to_read, &taskWoken) != pdPASS) {
            portEND_SWITCHING_ISR(taskWoken);
            return 0;
        }
    }
    else {
        if(xStreamBufferReceive(_serial_msg_rx_stream_handle, pData, _number_to_read, 0) != pdPASS) {
            return 0;
        }
    }

    return _number_to_read;
}

void serial_clear_tx_buffer(void){

	xStreamBufferReset(_serial_msg_tx_stream_handle);
}

/**
 * @brief  Get number of bytes in the RX buffer
 * 
 * @return size_t   number of bytes in the RX buffer
 */
size_t serial_available_bytes(void){

		return xStreamBufferBytesAvailable(_serial_msg_rx_stream_handle);
}

size_t serial_available_bytes_for_write(void){
		return xStreamBufferBytesAvailable(_serial_msg_tx_stream_handle);
}

static void uart_rx_callback(void){

    portBASE_TYPE taskWoken = pdFALSE;

    xStreamBufferSendFromISR(_serial_msg_rx_stream_handle, &_rx_data, 1, &taskWoken);
    portEND_SWITCHING_ISR(taskWoken);
}

static void uart_tx_callback(void){

    portBASE_TYPE taskWoken = pdFALSE;

    xSemaphoreGiveFromISR(_serail_tx_complete_semaphore_handle, &taskWoken);
    portEND_SWITCHING_ISR(taskWoken);
}
