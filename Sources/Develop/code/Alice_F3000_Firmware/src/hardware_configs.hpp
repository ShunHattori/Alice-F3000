#pragma once
#include <Arduino.h>

struct serial_comfig
{
    const uint32_t SOFTWARE_SERIAL_BAUD = 57600;
    const uint32_t HARDWARE_SERIAL_BAUD = 115200;
    const uint8_t RX_PIN = 2;
} SERIAL_CONFIG;

struct pin_assign
{
    const uint8_t SERIAL_RX = SERIAL_CONFIG.RX_PIN; //2
    const uint8_t SOFTWARE_RESET = 3;
    const uint8_t SPI_MOSI = MOSI; //11
    const uint8_t SPI_MISO = MISO; //12
    const uint8_t SPI_SCK = SCK;   //13
    const uint8_t SPI_SS = 4;
    const uint8_t _74HC595_LATCH = 6;
    const uint8_t WS2812B = 8;
    const uint8_t PWM_CW = 9;
    const uint8_t PWM_CCW = 10;
    const uint8_t DIP_1 = A0;
    const uint8_t DIP_2 = A1;
    const uint8_t DIP_3 = A2;
    const uint8_t DIP_4 = A3;
    const uint8_t HIGH_LOW_IN_1 = A5;
    const uint8_t HIGH_LOW_IN_2 = A4;
    const uint8_t INPUT_MODE_DETECT = 5; //UART入力 or (HIGH/LOW&PWM直)入力を検出
} PIN_ASSIGN;
