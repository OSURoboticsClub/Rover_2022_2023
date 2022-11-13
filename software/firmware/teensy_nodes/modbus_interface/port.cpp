#include <Arduino.h>
#include "port.h"
#include "modbus.h"


HardwareSerial *port = NULL;
usb_serial_class *sw_port = NULL;

uint16_t timeout;

static void _begin(uint32_t baud)
{
    if (sw_port)
        sw_port->begin(baud);
    else
        port->begin(baud);
}

static int _read()
{
    return sw_port ? sw_port->read() : port->read();
}

static void _write(uint8_t *packet, uint16_t packetSize)
{
    if (sw_port)
        sw_port->write(packet, packetSize);
    else    
        port->write(packet, packetSize);
}

static int _available()
{
    return sw_port ? sw_port->available() : port->available();
}

#ifdef __cplusplus
extern "C"
{
#endif

void portSetup(uint8_t serialNumber, uint8_t TXEnablePin, const uint32_t baud, const uint16_t serialTimeout)
{
    timeout = serialTimeout;

    switch (serialNumber)
    {
    case 1:
        port = &Serial1;
        break;
    case 2:
        port = &Serial2;
        break;
    case 3:
        port = &Serial3;
        break;
    case 0:
        sw_port = &Serial;
        break;
    default:
        break;
    }

    _begin(baud);
    if (port && TXEnablePin > 1)
    {
        pinMode(TXEnablePin, OUTPUT);
        digitalWrite(TXEnablePin, LOW);
        port->transmitterEnable(TXEnablePin);
    }

    while (_read() >= 0)
        ;
}

void portWrite(uint8_t *packet, uint16_t packetSize)
{
    _write(packet, packetSize);
}

void modbus_update_wr() {
    modbus_update();
}

uint32_t millis_wr() {
    return millis();
}


#ifdef __cplusplus
}
#endif

void serialEvent_Handler()
{
    while (_available())
    { // confirm there is data ready to be read
        rxBuffer.data[rxBuffer.head] = _read();
        rxBuffer.head = PKT_WRAP_ARND(rxBuffer.head + 1); // iterate the head through the ring buffer
    }
}

// interrupt handler for incoming data
void serialEvent3()
{
    serialEvent_Handler();
}

void serialEvent()
{
    serialEvent_Handler();
}