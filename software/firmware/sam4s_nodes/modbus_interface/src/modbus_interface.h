#ifndef MODBUS_INTERFACE_H
#define MODBUS_INTERFACE_H

#include <stdint.h>
#include <asf.h>

#define REGISTER_AR_SIZE	256		//Size of the register array for a given data type

extern uint16_t		intRegisters[REGISTER_AR_SIZE];
extern float		floatRegisters[REGISTER_AR_SIZE];
extern char			charRegisters[REGISTER_AR_SIZE];
extern bool			boolRegisters[REGISTER_AR_SIZE];


extern uint16_t timeout;

// Referenced by nodes
void modbus_init(int, Uart*, const uint32_t, Pio*, const uint32_t, const uint16_t);
void modbus_update(void);
bool modbus_comm_good(void);

// Referenced by slave library
void serial_write(uint8_t*, uint16_t);
uint32_t get_elapsed_ms(void);

#endif
