#pragma once
#include <Arduino.h>

class AntiChattering
{
private:
    uint64_t sampled_time;
    int16_t pin, detect_flag_number, sampling_period,
        button_press_count, analog_threshold;
    bool button_state, is_pin_analog;
    void process();
    void setStateFlag();

public:
    AntiChattering(int);
    void initialize();
    void update();
    bool getState();

    virtual ~AntiChattering(){};
};

AntiChattering::AntiChattering(int pin_)
    : pin(pin_),
      detect_flag_number(10),
      sampling_period(1),
      analog_threshold(800),
      button_state(0),
      is_pin_analog(0){};

void AntiChattering::initialize()
{
    sampled_time = millis();
    pinMode(pin, INPUT);
    if (A0 <= pin && pin <= A7)
        is_pin_analog = true;
}

void AntiChattering::update()
{
    process();
    setStateFlag();
    return;
}

void AntiChattering::process()
{
    if (uint16_t(sampling_period) > (millis() - sampled_time))
        return;
    if (is_pin_analog)
    {
        if (analogRead(pin) > analog_threshold)
            button_press_count++;
        else
            button_press_count--;
    }
    else
    {
        if (digitalRead(pin))
            button_press_count++;
        else
            button_press_count--;
    }
    button_press_count = constrain(button_press_count, 0, detect_flag_number);
    sampled_time = millis();
}

void AntiChattering::setStateFlag()
{
    if (button_press_count == detect_flag_number)
        button_state = 1;
    else if (button_press_count == 0)
        button_state = 0;
}

bool AntiChattering::getState()
{
    return button_state;
}
