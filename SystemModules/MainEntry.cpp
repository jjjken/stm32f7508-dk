#include "Serial.hpp"
#include "cmsis_os.h"

#include "printf.h"


#define UDP_REMOTE_PORT    8881 
#define UDP_LOCAL_PORT     8880 

extern "C" void main_task_start(void);

Serial serial1;

TaskHandle_t main_task_handle;


void main_task(void* arg);
void test(void);
void udp_server_init(void);

void main_task_start(void){
	xTaskCreate((TaskFunction_t)main_task, "main", (configMINIMAL_STACK_SIZE * 12), NULL, tskIDLE_PRIORITY, &main_task_handle);
}

void main_task(void* arg){
	uint8_t addr[4] = {0};
	char print_buffer[50] = {0};
	
	serial1.begin();

	
	for(;;){

		
		vTaskDelay(2000);
	}

}

void test(void){

	serial1.println("My Name is JJJ");
	serial1.println(1.7514f);
	serial1.println(4151);
}

