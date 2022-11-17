#include <asf.h>
#include "modbus_interface.h"
#include "stepper.h"

#define STEPPER_PWM_CHANNEL PWM_CHANNEL_3
#define STEPPER_DIR_PORT PIOA
#define STEPPER_DIR_PIN PIO_PA6
#define STEPPER_STEP_PORT PIOA
#define STEPPER_STEP_PIN PIO_PA7


#define LSw1_PORT				PIOA
#define LSw1_PIN				PIO_PA0

#define LSw2_PORT				PIOA
#define LSw2_PIN				PIO_PA1

#define LSw3_PORT				PIOA
#define LSw3_PIN				PIO_PA2

#define LSw4_PORT				PIOA
#define LSw4_PIN				PIO_PA3

#define Lazer_PORT				PIOA
#define Lazer_PIN				PIO_PA13

#define vidSel0_PORT			PIOA
#define vidSel0_PIN				PIO_PA14

#define vidSel1_PORT			PIOA
#define vidSel1_PIN				PIO_PA15

// modbus settings
#define SLAVEID 15

#define MODBUS_SLAVE_ID 15
#define MODBUS_BPS 115200
#define MODBUS_TIMEOUT 2000
#define MODBUS_SER_PORT UART0
#define MODBUS_EN_PORT PIOA
#define MODBUS_EN_PIN PIO_PA11

//int Registers
#define stepperPosition 1
#define cameraSelect 2
#define lazerEnable 3
#define limSw1	4
#define limSw2	5
#define limSw3	6
#define limSw4	7

int steps_per_rev = 465;

void board_init(void) {
	pmc_enable_periph_clk(ID_PIOA);			//This enables GPIO Outpus, (necessary)
	pmc_enable_periph_clk(ID_PIOB);
	
	pio_set_input(LSw1_PORT,LSw1_PIN,PIO_DEBOUNCE);
	pio_set_input(LSw2_PORT,LSw2_PIN,PIO_DEBOUNCE);
	pio_set_input(LSw3_PORT,LSw3_PIN,PIO_DEBOUNCE);
	pio_set_input(LSw4_PORT,LSw4_PIN,PIO_DEBOUNCE);
	
	pio_set_output(Lazer_PORT,Lazer_PIN,LOW,DISABLE,DISABLE);
	pio_set_output(vidSel0_PORT,vidSel0_PIN,LOW,DISABLE,DISABLE);
	pio_set_output(vidSel1_PORT,vidSel1_PIN,LOW,DISABLE,DISABLE);
}

int main(void) {
	sysclk_init();
	board_init();
	
	modbus_init(MODBUS_SLAVE_ID, MODBUS_SER_PORT, MODBUS_BPS, MODBUS_EN_PORT, MODBUS_EN_PIN, MODBUS_TIMEOUT);
	modbus_init(MODBUS_SLAVE_ID);
	
	stepper_s stepper;
	stepper_setup(&stepper, STEPPER_PWM_CHANNEL, STEPPER_DIR_PORT, STEPPER_DIR_PIN, STEPPER_STEP_PORT, STEPPER_STEP_PIN);

	// Some tests for now switching between frequencies.
	//stepper_set_position(&stepper, 465);
	//for (volatile uint32_t i = 0; i < (12000000) * 3; i++);
	//stepper_set_velocity(&stepper, 10, STEPPER_DIR_CCW);
	//for (volatile uint32_t i = 0; i < (12000000) * 3; i++);
	//stepper_stop(&stepper);

	while (1) {
		modbus_update();

		if(stepper.position != intRegisters[stepperPosition]) stepper_set_position(&stepper, intRegisters[stepperPosition]);	//If the position needs to change, change it.
		
		intRegisters[limSw1] = pio_get(LSw1_PORT,PIO_TYPE_PIO_INPUT,LSw1_PIN);
		intRegisters[limSw2] = pio_get(LSw2_PORT,PIO_TYPE_PIO_INPUT,LSw2_PIN);
		intRegisters[limSw3] = pio_get(LSw3_PORT,PIO_TYPE_PIO_INPUT,LSw3_PIN);
		intRegisters[limSw4] = pio_get(LSw4_PORT,PIO_TYPE_PIO_INPUT,LSw4_PIN);
		
		if(intRegisters[lazerEnable]) pio_set(Lazer_PORT, Lazer_PIN);
		else pio_clear(Lazer_PORT, Lazer_PIN);
		
		
		if(intRegisters[cameraSelect]) pio_set(vidSel0_PORT, vidSel0_PIN);				//This is the only video select pin that matters
		else pio_clear(vidSel0_PORT, vidSel0_PIN);
	}
}
