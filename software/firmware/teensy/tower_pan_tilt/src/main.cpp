////////// Includes //////////
#include <Arduino.h>
#include <modbus_interface.h>
#include <Servo.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  RS485_EN = 2,
  RS485_RX = 9,
  RS485_TX = 10,

  SERVO_PAN = 5,
  SERVO_TILT = 4,
  SERVO_HITCH = 21,

  LED_RED = 1,
  LED_GREEN = 32,
  LED_BLUE = 6,

  LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
  CENTER_ALL = 0,           // Input/Output
  PAN_ADJUST_POSITIVE = 1,  // Input/Output
  PAN_ADJUST_NEGATIVE = 2,  // Input/Output
  TILT_ADJUST_POSITIVE = 3, // Input/Output
  TILT_ADJUST_NEGATIVE = 4, // Input/Output
  HITCH_SERVO_POSITIVE = 5,
  HITCH_SERVO_NEGATIVE = 6
};

////////// Global Variables //////////
const uint8_t node_id = 2;
const uint8_t mobus_serial_port_number = 2;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

// Pan/tilt hard limits
const int pan_min = 1470;
const int pan_center = 1605;
const int pan_max = 1725;

const int tilt_min = 1020;
const int tilt_center = 1820;
const int tilt_max = 2400;

// Pan/tilt positions
int pan_position = pan_center;
int tilt_position = tilt_center;

Servo pan_servo;
Servo tilt_servo;
Servo hitch_servo;

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::RS485_EN, OUTPUT);

  pinMode(HARDWARE::SERVO_PAN, OUTPUT);
  pinMode(HARDWARE::SERVO_TILT, OUTPUT);
  pinMode(HARDWARE::SERVO_HITCH, OUTPUT);

  pan_servo.attach(HARDWARE::SERVO_PAN);
  tilt_servo.attach(HARDWARE::SERVO_TILT);
  hitch_servo.attach(HARDWARE::SERVO_HITCH);

  pan_servo.writeMicroseconds(pan_center);
  tilt_servo.writeMicroseconds(tilt_center);

  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);

  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  // Set default pin states
  digitalWrite(HARDWARE::LED_RED, LOW);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);

  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
}

void poll_modbus() {
  modbus_update();
  communication_good = modbus_comm_good();
}

void set_leds() {
  if (poll_state > 4) {
    message_count++;
    if (message_count > 2) {
      digitalWrite(HARDWARE::LED_BLUE_EXTRA, !digitalRead(HARDWARE::LED_BLUE_EXTRA));
      message_count = 0;
    }

    digitalWrite(HARDWARE::LED_GREEN, LOW);
    digitalWrite(HARDWARE::LED_RED, HIGH);
  } else if (!communication_good) {
    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
    digitalWrite(HARDWARE::LED_GREEN, HIGH);
    digitalWrite(HARDWARE::LED_RED, LOW);
  }
}

void set_pan_tilt_adjustments() {
  if (communication_good) {
    if (intRegisters[MODBUS_REGISTERS::CENTER_ALL]) {
      pan_servo.writeMicroseconds(constrain(pan_position, pan_min, pan_max));
      tilt_servo.writeMicroseconds(constrain(tilt_position, tilt_min, tilt_max));

      pan_position = pan_center;
      tilt_position = tilt_center;

      intRegisters[MODBUS_REGISTERS::CENTER_ALL] = 0;
    }

    pan_position = constrain(pan_position - intRegisters[MODBUS_REGISTERS::PAN_ADJUST_POSITIVE] + intRegisters[MODBUS_REGISTERS::PAN_ADJUST_NEGATIVE], pan_min, pan_max);
    tilt_position = constrain(tilt_position + intRegisters[MODBUS_REGISTERS::TILT_ADJUST_POSITIVE] - intRegisters[MODBUS_REGISTERS::TILT_ADJUST_NEGATIVE], tilt_min, tilt_max);

    pan_servo.writeMicroseconds(pan_position);
    tilt_servo.writeMicroseconds(tilt_position);
//    Serial.print(pan_position);
//    Serial.print("\t");
//    Serial.println(tilt_position);

    intRegisters[MODBUS_REGISTERS::PAN_ADJUST_POSITIVE] = 0;
    intRegisters[MODBUS_REGISTERS::PAN_ADJUST_NEGATIVE] = 0;
    intRegisters[MODBUS_REGISTERS::TILT_ADJUST_POSITIVE] = 0;
    intRegisters[MODBUS_REGISTERS::TILT_ADJUST_NEGATIVE] = 0;
    //intRegisters[MODBUS_REGISTERS::HITCH_SERVO_POSITIVE] = 0;
    //intRegisters[MODBUS_REGISTERS::HITCH_SERVO_NEGATIVE] = 0;

  }
}

void set_hitch_adjustments() {
  if (communication_good) {
    if (intRegisters[MODBUS_REGISTERS::HITCH_SERVO_POSITIVE]) {
      hitch_servo.write(60);
    } else if (intRegisters[MODBUS_REGISTERS::HITCH_SERVO_NEGATIVE]) {
      hitch_servo.write(120);
    }
  }
}

void setup() {
 Serial.begin(9600);
//  while(!Serial);
  setup_hardware();

  modbus_init(node_id, mobus_serial_port_number, HARDWARE::RS485_EN, 115200, 150);
}

void loop() {
  poll_modbus();
  set_leds();
  set_pan_tilt_adjustments();
  set_hitch_adjustments();
}
