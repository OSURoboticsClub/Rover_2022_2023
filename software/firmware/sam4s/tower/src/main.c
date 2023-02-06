#include <asf.h>
#include "modbus_uart0.h"
#include "gps.h"
//#include "led_control.h"

// Modbus specific information
#define MODBUS_SLAVE_ID 1
#define MODBUS_BPS 115200
#define MODBUS_TIMEOUT 2000
#define MODBUS_SER_PORT UART0
#define MODBUS_EN_PORT PIOA
#define MODBUS_EN_PIN PIO_PA8


static void board_setup(void) {
	WDT->WDT_MR |= WDT_MR_WDDIS; // Disable watchdog timer to prevent uC resetting every 15 seconds :)
}


int main(void) {
	sysclk_init();
	board_setup();
	
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
