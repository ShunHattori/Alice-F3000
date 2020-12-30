#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "SoftwareSerial.h"
#include "Adafruit_NeoPixel.h"
#include "hardware_configs.hpp"
#include "AntiChattering.hpp"
#include "Observer.hpp"
#include "Protocol.hpp"
#include "PixelController.hpp"

/**************************************************************/
/*                        TODO - features
*  [x] SERTIAL - CONTROL(PC)
*  [ ] Protocol - receive logic
*  [x] WS2812B LED CONTROL
*  [ ] WAITMS1 - class logic
*/
/**************************************************************/

enum
{
  cw,
  ccw,
};

struct color_32bit
{
  const uint32_t RED = 0xFF0000;
  const uint32_t GREEN = 0x00FF00;
  const uint32_t BLUE = 0x0000FF;
  const uint32_t ORANGE = 0xFF8000;
  const uint32_t MAGENTA = 0xFF00FF;
  const uint32_t SKYBLUE = 0x00FFFF;
} COLOR_32BIT;

struct data
{
  const uint8_t MAX_PWM = 250;
  uint32_t pixel_color = 0;
  int16_t pwm = 0;
  uint8_t seven_segment_high_byte = 0;
  uint8_t seven_segment_low_byte = 0;
  uint8_t motor_driver_id = 0;
  bool running_mode = 0;
  bool self_debug_direction = cw;
  bool is_uart_mode = 0;
  const uint8_t seven_segment_pattern[10] = {
      {0b11111100}, //0
      {0b01100000}, //1
      {0b11011010}, //2
      {0b11110010}, //3
      {0b01100110}, //4
      {0b10110110}, //5
      {0b10111110}, //6
      {0b11100000}, //7
      {0b11111110}, //8
      {0b11100110}, //9
  };
} data;

Protocol protocol(PIN_ASSIGN.SERIAL_RX, SERIAL_CONFIG.SOFTWARE_SERIAL_BAUD);
AntiChattering DIP_1(PIN_ASSIGN.DIP_1);
AntiChattering DIP_2(PIN_ASSIGN.DIP_2);
AntiChattering DIP_3(PIN_ASSIGN.DIP_3);
AntiChattering DIP_4(PIN_ASSIGN.DIP_4);
AntiChattering HIGH_LOW_IN_1(PIN_ASSIGN.HIGH_LOW_IN_1);
AntiChattering HIGH_LOW_IN_2(PIN_ASSIGN.HIGH_LOW_IN_2);
AntiChattering SOFTWARE_RESET(PIN_ASSIGN.SOFTWARE_RESET);
AntiChattering INPUT_MODE_DETECT(PIN_ASSIGN.INPUT_MODE_DETECT);
PixelController WS2812B(1, PIN_ASSIGN.WS2812B);
Observer<uint8_t> obs_motor_driver_id;
Observer<bool> obs_HIGH_LOW_1, obs_HIGH_LOW_2;
SPISettings SPI_LOWSPEED_SETTING = SPISettings(125000, MSBFIRST, SPI_MODE0);   //125kHz
SPISettings SPI_HIGHSPEED_SETTING = SPISettings(4000000, MSBFIRST, SPI_MODE0); //4.0MHz

/**************************************************************/
/*                        AVR FUNCTIONS                       */
void setPwmFrequencyUNO(int, int);
void wdt_reboot();
/**************************************************************/

/**************************************************************/
/*                      CAPSULE FUNCTIONS                     */
void initialize_switches();
void update_motor_driver_id();
void update_seven_segment(uint8_t);
void update_output_pwm();
void update_switches();
bool is_text_contain(String, String);
uint8_t get_DIP_states();
int waitMs1(uint16_t);
int waitMs2(uint16_t);
/**************************************************************/

/**************************************************************/
/*                       MODES FUNCTIONS                      */
void UART_INPUT_MODE();
void HIGH_LOW_INPUT_MODE();
/**************************************************************/

/**************************************************************/
/*                   DEBUG & TEST FUNCTIONS                   */
template <class arg1, class arg2>
void CONSOLE_DEBUG(String identifier, arg1 var1, arg2 var2);
void hardware_serial_debug();
void temp_test_codes();
/**************************************************************/

template <class arg1, class arg2>
void CONSOLE_DEBUG(String identifier, arg1 var1, arg2 var2)
{
  String output;
  output = "@:[" + String(identifier) + "]" + '\t';
  output += "(" + String(var1) + ")" + '\t';
  output += String(var2) + "\r\n";
  Serial.print(output);
}

void UART_INPUT_MODE()
{
  if (protocol.receive() == 3)
  {
    data.pixel_color = COLOR_32BIT.GREEN;
    WS2812B.setColor(0, data.pixel_color);
    data.pwm = protocol.get_pwm();
  }
  if (!SOFTWARE_RESET.getState())
    wdt_reboot();
}

void HIGH_LOW_INPUT_MODE()
{
  if (obs_HIGH_LOW_1.isChanged(HIGH_LOW_IN_1.getState()) ||
      obs_HIGH_LOW_2.isChanged(HIGH_LOW_IN_2.getState()))
  {
    if (HIGH_LOW_IN_1.getState() == HIGH)
    {
      data.pwm = (1) * get_DIP_states() * 16;
      data.pixel_color = COLOR_32BIT.ORANGE;
    }
    else if (HIGH_LOW_IN_2.getState() == HIGH)
    {
      data.pwm = (-1) * get_DIP_states() * 16;
      data.pixel_color = COLOR_32BIT.SKYBLUE;
    }
    else
    {
      data.pwm = 0;
      data.pixel_color = COLOR_32BIT.MAGENTA;
    }
    CONSOLE_DEBUG("HIGH_LOW_INPUT_MODE", "Switch State", "changed");
    CONSOLE_DEBUG("HIGH_LOW_INPUT_MODE", "IN_1", HIGH_LOW_IN_1.getState());
    CONSOLE_DEBUG("HIGH_LOW_INPUT_MODE", "IN_2", HIGH_LOW_IN_2.getState());
    CONSOLE_DEBUG("HIGH_LOW_INPUT_MODE", "data.pwm", data.pwm);
    CONSOLE_DEBUG("HIGH_LOW_INPUT_MODE", "data.pixel_color", data.pixel_color);
    WS2812B.setColor(0, data.pixel_color);
  }
}

void setup()
{
  Serial.begin(SERIAL_CONFIG.HARDWARE_SERIAL_BAUD);
  WS2812B.initialize();
  protocol.initialize();
  SPI.begin();
  initialize_switches();
  pinMode(PIN_ASSIGN.SPI_SS, OUTPUT);
  pinMode(PIN_ASSIGN._74HC595_LATCH, OUTPUT);
  setPwmFrequencyUNO(PIN_ASSIGN.PWM_CW, 1);
  setPwmFrequencyUNO(PIN_ASSIGN.PWM_CCW, 1);
  CONSOLE_DEBUG("setup", "initialize", "done");
}

void update_motor_driver_id()
{
  if (!waitMs1(50))
    return;
  data.motor_driver_id = get_DIP_states();
  if (obs_motor_driver_id.isChanged(data.motor_driver_id)) //test point2
  {
    protocol.update_id(data.motor_driver_id);
    update_seven_segment(data.motor_driver_id);
  }
}

void loop()
{
  update_switches();

  if (!waitMs2(10))
    return;
  // temp_test_codes();
  // hardware_serial_debug();

  //UART通信モードかHIGH/LOW入力モードかの状態を取得
  data.is_uart_mode = INPUT_MODE_DETECT.getState();
  // CONSOLE_DEBUG("loop", "data.is_uart_mode", data.is_uart_mode);

  update_motor_driver_id();

  if (data.is_uart_mode)
    UART_INPUT_MODE();
  else
    HIGH_LOW_INPUT_MODE();

  update_output_pwm();
}

void update_seven_segment(uint8_t num)
{
  num = constrain(num, 0, 99);
  data.seven_segment_high_byte = uint8_t(num / 10);
  data.seven_segment_low_byte = uint8_t(num % 10);
  CONSOLE_DEBUG("update_seven_segment", "dip numbered", num);
  CONSOLE_DEBUG("update_seven_segment", "HIGH byte", data.seven_segment_high_byte);
  CONSOLE_DEBUG("update_seven_segment", "LOW byte", data.seven_segment_low_byte);
  //SPI OUTPUTS
  SPI.beginTransaction(SPI_LOWSPEED_SETTING);
  digitalWrite(PIN_ASSIGN._74HC595_LATCH, LOW);
  digitalWrite(PIN_ASSIGN.SPI_SS, LOW);
  SPI.transfer(data.seven_segment_pattern[data.seven_segment_high_byte]);
  SPI.transfer(data.seven_segment_pattern[data.seven_segment_low_byte]);
  digitalWrite(PIN_ASSIGN.SPI_SS, HIGH);
  digitalWrite(PIN_ASSIGN._74HC595_LATCH, HIGH);
  SPI.endTransaction();
}

uint8_t get_DIP_states()
{
  // return 15; //test point1
  uint8_t state = uint8_t(DIP_1.getState() << 0) |
                  uint8_t(DIP_2.getState() << 1) |
                  uint8_t(DIP_3.getState() << 2) |
                  uint8_t(DIP_4.getState() << 3);
  return state;
}

void initialize_switches()
{
  DIP_1.initialize();
  DIP_2.initialize();
  DIP_3.initialize();
  DIP_4.initialize();
  HIGH_LOW_IN_1.initialize();
  HIGH_LOW_IN_2.initialize();
  SOFTWARE_RESET.initialize();
  INPUT_MODE_DETECT.initialize();
}
void update_switches()
{
  DIP_1.update();
  DIP_2.update();
  DIP_3.update();
  DIP_4.update();
  HIGH_LOW_IN_1.update();
  HIGH_LOW_IN_2.update();
  SOFTWARE_RESET.update();
  INPUT_MODE_DETECT.update();
}

void update_output_pwm()
{
  uint8_t pwm, direction;
  pwm = data.pwm > 0 ? data.pwm : -data.pwm;
  direction = data.pwm > 0 ? cw : ccw;

  if (direction == cw)
  {
    analogWrite(PIN_ASSIGN.PWM_CW, pwm);
    analogWrite(PIN_ASSIGN.PWM_CCW, 0);
  }
  else
  {
    analogWrite(PIN_ASSIGN.PWM_CW, 0);
    analogWrite(PIN_ASSIGN.PWM_CCW, pwm);
  }
}

int waitMs1(uint16_t milliseconds)
{
  static unsigned long long prevLaunched = 0;
  if ((millis() - prevLaunched) > milliseconds)
  {
    prevLaunched = millis();
    return 1;
  }
  return 0;
}
int waitMs2(uint16_t milliseconds)
{
  static unsigned long long prevLaunched = 0;
  if ((millis() - prevLaunched) > milliseconds)
  {
    prevLaunched = millis();
    return 1;
  }
  return 0;
}

void wdt_reboot()
{
  wdt_enable(WDTO_15MS);
  CONSOLE_DEBUG("wdt_reboot", "DETECT RESET SIGNAL. REBOOTING...", "");
  while (1)
  {
  }
}

void setPwmFrequencyUNO(int pin, int divisor)
{
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    if (pin == 5 || pin == 6)
    {
      TCCR0B = (TCCR0B & (0b11111000 | divisor));
    }
    else
    {
      // resister setting for fast pwm mode (62.5KHz)
      // Serial.println(TCCR1A, BIN);
      // TCCR1A = (TCCR1A | 0b00000001);
      // Serial.println(TCCR1A, BIN);
      // Serial.println(TCCR1B, BIN);
      // TCCR1B = (TCCR1B | 0b00001000);
      // Serial.println(TCCR1B, BIN);
      TCCR1B = (TCCR1B & (0b11111000 | divisor));
      // Serial.println(TCCR1B, BIN);
    }
  }
  else if (pin == 3 || pin == 11)
  {
    TCCR2B = (TCCR2B & (0b11111000 | divisor));
  }
}

bool is_text_contain(String cmd, String head)
{
  // Serial.println(cmd.indexOf(head));
  if (cmd.indexOf(head) >= 0)
  {
    return true;
  }
  return false;
}

void hardware_serial_debug()
{
  if (!Serial.available())
    return;

  String command = Serial.readStringUntil('\n');
  if (is_text_contain(command, "set") || is_text_contain(command, "SET"))
  {
    if (is_text_contain(command, "motor") || is_text_contain(command, "MOTOR"))
    {
      Serial.print("#Enter PWM value (~");
      Serial.print(-data.MAX_PWM);
      Serial.print(" to ");
      Serial.print(data.MAX_PWM);
      Serial.println(")");
      while (true)
      {
        if (Serial.available())
        {
          String pwm_st = Serial.readStringUntil('\n');
          int pwm = constrain(pwm_st.toInt(), -data.MAX_PWM, data.MAX_PWM);
          Serial.print("PWM:[");
          Serial.print(pwm);
          Serial.println("] applied.");
          data.pwm = pwm;
          break;
        }
      }
    }
    if (is_text_contain(command, "led") || is_text_contain(command, "LED"))
    {
      Serial.println("#Enter RGB value (32bit HEX)");
      while (true)
      {
        if (Serial.available())
        {
          String RGB_32bit_st = Serial.readStringUntil('\n');
          uint32_t RGB_32bit = constrain(RGB_32bit_st.toInt(), 0, 4294967295);
          Serial.print("LED:[");
          Serial.print(RGB_32bit);
          Serial.println("] applied.");
          data.pixel_color = RGB_32bit;
          WS2812B.setColor(0, data.pixel_color);
          break;
        }
      }
    }
  }
  else if (is_text_contain(command, "get"))
  {
  }
}

void temp_test_codes()
{
  analogWrite(PIN_ASSIGN.PWM_CW, 128);
  analogWrite(PIN_ASSIGN.PWM_CCW, 128);

  while (1)
  {
    WS2812B.setColor(0, 0xffff00);
    update_switches();
    Serial.print(DIP_1.getState());
    Serial.print('\t');
    Serial.print(DIP_2.getState());
    Serial.print('\t');
    Serial.print(DIP_3.getState());
    Serial.print('\t');
    Serial.print(DIP_4.getState());
    Serial.print('\t');
    Serial.print(HIGH_LOW_IN_1.getState());
    Serial.print('\t');
    Serial.print(HIGH_LOW_IN_2.getState());
    Serial.print('\t');
    Serial.print(SOFTWARE_RESET.getState());
    Serial.print('\t');
    Serial.print(INPUT_MODE_DETECT.getState());
    Serial.print("\r\n");

    static unsigned long long times = micros();
    if (micros() - times < 500)
      continue;
    times = micros();

    SPI.beginTransaction(SPI_LOWSPEED_SETTING);
    digitalWrite(PIN_ASSIGN.SPI_SS, LOW); // (4)
    SPI.transfer(0b10101010);             // (5)
    //SPI.transfer16(0b1010101010010010);
    digitalWrite(PIN_ASSIGN.SPI_SS, HIGH); // (6)
    SPI.endTransaction();
  }
}
