#include <Arduino.h>
#include "port.h"

int lol = 0;

void setup() {
  portSetup(1, 1, 1, 1);
}

void loop() {
  modbus_update_wr();
  lol += intRegisters[0];
}