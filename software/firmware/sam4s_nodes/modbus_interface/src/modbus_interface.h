#ifndef PORT_H
#define PORT_H

#include <stdint.h>
#include <asf.h>

#define REGISTER_AR_SIZE	256		//Size of the register array for a given data type

extern uint16_t		intRegisters[REGISTER_AR_SIZE];
extern float		floatRegisters[REGISTER_AR_SIZE];
extern char			charRegisters[REGISTER_AR_SIZE];
extern bool			boolRegisters[REGISTER_AR_SIZE];


extern uint16_t timeout;

void portSetup(int, Uart*, const uint32_t, Pio*, const uint32_t, const uint16_t);

void serial_write(uint8_t*, uint16_t);

uint32_t millis_wr(void);

void modbus_update_wr(void);

#endif
