#include "Serial.hpp"
#include "cmsis_os.h"

extern "C" void main_task_start(void);

Serial serial1;

TaskHandle_t main_task_handle;

void main_task(void* arg);

void test(void);

void main_task_start(void){
	xTaskCreate((TaskFunction_t)main_task, "main", (configMINIMAL_STACK_SIZE * 12), NULL, tskIDLE_PRIORITY, &main_task_handle);
}

void main_task(void* arg){
	
	serial1.begin();
	
	for(;;){
		//serial.println("Hello world");
		test();
		vTaskDelay(2000);
	}

}

void test(void){

	serial1.println("My Name is JJJ");
	serial1.println(1.7514f);
	serial1.println(4151);
}
