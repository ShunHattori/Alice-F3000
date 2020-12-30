#pragma once
#include <Arduino.h>
#include "Adafruit_NeoPixel.h"

class PixelController
{
private:
    Adafruit_NeoPixel *pixel;
    uint8_t pixel_num, pixel_pin;
    uint32_t *prevColor;
    const uint8_t brightness = 100;

public:
    PixelController(uint8_t, uint8_t);
    ~PixelController(){};

    void initialize();
    void setColor(uint8_t, uint32_t);
};

PixelController::PixelController(uint8_t pixel_num, uint8_t pixel_pin)
{
    this->pixel_num = pixel_num;
    this->pixel_pin = pixel_pin;
    prevColor = new uint32_t[pixel_num];
}

void PixelController::initialize()
{
    pixel = new Adafruit_NeoPixel(pixel_num, pixel_pin, NEO_GRB + NEO_KHZ800);
    pixel->begin();
    pixel->clear();
    pixel->setBrightness(brightness);
    pixel->show();
}

void PixelController::setColor(uint8_t index, uint32_t color_32bits)
{
    //guard invalid access to array
    if (pixel_num <= index)
        return;
    if (prevColor[index] == color_32bits)
        return;
    pixel->setPixelColor(index, color_32bits);
    pixel->show();
    prevColor[index] = color_32bits;
}