#include <Arduino.h>
#include "modbus_interface.h"

int lol = 0;

void setup() {
  modbus_init(1, 1, 1, 1, 1);
}

void loop() {
  modbus_update();
  lol += intRegisters[0];
}