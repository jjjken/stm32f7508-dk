#include "Serial.hpp"
#include "cmsis_os.h"
#include "lwip.h"
#include "printf.h"

extern "C" void main_task_start(void);

Serial serial1;

TaskHandle_t main_task_handle;

void main_task(void* arg);
void test(void);

extern struct netif gnetif;

void main_task_start(void){
	xTaskCreate((TaskFunction_t)main_task, "main", (configMINIMAL_STACK_SIZE * 12), NULL, tskIDLE_PRIORITY, &main_task_handle);
}

void main_task(void* arg){
	uint32_t ip_addr = 0;
	uint8_t addr[4] = {0};
	char print_buffer[50] = {0};
	
	serial1.begin();

	ip_addr = gnetif.ip_addr.addr;
	
	for(;;){
		//serial.println("Hello world");
		//test();
		ip_addr = gnetif.ip_addr.addr;

		addr[0] = ip_addr >> 24;
		addr[1] = ip_addr >> 16;
		addr[2] = ip_addr >> 8;
		addr[3] = ip_addr;
		
		memset(print_buffer, 0, sizeof(print_buffer));
		
		sprintf(print_buffer, "IP %d.%d.%d.%d\n\r",(ip_addr & 0xff), ((ip_addr >> 8) & 0xff), ((ip_addr >> 16) & 0xff), (ip_addr >> 24));
		serial1.write((uint8_t*)print_buffer, strlen(print_buffer));
		
		vTaskDelay(2000);
	}

}

void test(void){

	serial1.println("My Name is JJJ");
	serial1.println(1.7514f);
	serial1.println(4151);
}
