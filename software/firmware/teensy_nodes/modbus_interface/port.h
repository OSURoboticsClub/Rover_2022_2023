#ifndef PORT_H
#define PORT_H

#include <stdint.h>

#define REGISTER_AR_SIZE	256		//Size of the register array for a given data type

// interrupt handler for incoming data
void serialEvent();

#ifdef __cplusplus
extern "C"
{
#endif

extern uint16_t		intRegisters[REGISTER_AR_SIZE];
extern float		floatRegisters[REGISTER_AR_SIZE];
extern char			charRegisters[REGISTER_AR_SIZE];
extern bool			boolRegisters[REGISTER_AR_SIZE];

void portSetup(uint8_t, uint8_t, const uint32_t, const uint16_t);

void portWrite(uint8_t *, uint16_t);

void modbus_update_wr();

uint32_t millis_wr();

#ifdef __cplusplus
}
#endif

#endif
