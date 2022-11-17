#include <asf.h>
#include "modbus_interface.h"
#include "gps.h"
//#include "led_control.h"

// Modbus specific information
#define MODBUS_SLAVE_ID 1
#define MODBUS_BPS 115200
#define MODBUS_TIMEOUT 2000
#define MODBUS_SER_PORT UART1
#define MODBUS_EN_PORT PIOA
#define MODBUS_EN_PIN PIO_PA7


int main(void) {
	sysclk_init();
	
	modbus_init(MODBUS_SLAVE_ID, MODBUS_SER_PORT, MODBUS_BPS, MODBUS_EN_PORT, MODBUS_EN_PIN, MODBUS_TIMEOUT);
	
	gps_setup();

	while (1) {
		modbus_update();
		//set_LEDs();
		gps_handle();
	}
	
	while(1) {
		modbus_update();
	}
}
