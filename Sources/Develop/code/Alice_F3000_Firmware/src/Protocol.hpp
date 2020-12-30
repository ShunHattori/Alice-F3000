#pragma once
#include <Arduino.h>
#include "SoftwareSerial.h"

class Protocol
{
private:
    SoftwareSerial *uart;
    uint8_t id, pin;
    uint16_t baud;
    void parse();

public:
    Protocol(uint8_t, uint16_t);
    virtual ~Protocol() {}
    void initialize();
    void update_id(uint8_t);
    int get_pwm();
    int receive();
};

Protocol::Protocol(uint8_t pin, uint16_t baud)
{
    this->pin = pin;
    this->baud = baud;
};

void Protocol::initialize()
{
    uart = new SoftwareSerial(pin, 0);
    uart->begin(baud);
    uart->listen();
}

void Protocol::update_id(uint8_t id)
{
    this->id = id;
}

int Protocol::receive()
{
    // if (!(uart->available() > 2))
    //     return 0;
    // if (uart->read() != 0xff)
    //     return 1;
    // if (uart->read() != data.motor_driver_id)
    //     return 2;
    // data.pwm = uart->read();
    // if (uart->read() != 0)
    //     data.pwm = uart->read();
    // return 3;
}

int Protocol::get_pwm()
{
    return 0;
}