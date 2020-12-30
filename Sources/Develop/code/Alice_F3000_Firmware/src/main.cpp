#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "SoftwareSerial.h"
#include "Adafruit_NeoPixel.h"

/**************************************************************/
/*                        TODO - features
*  [x] SERTIAL - CONTROL(PC)
*  [ ] Protocol - receive logic
*  [x] WS2812B LED CONTROL
*  [ ] WAITMS - class logic
*/
/**************************************************************/

struct serial_comfig
{
  const uint16_t SERIAL_BAUD = 57600;
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

enum
{
  cw,
  ccw,
};

struct color_32bit
{
  uint32_t RED = 0xFF0000;
  uint32_t GREEN = 0x00FF00;
  uint32_t BLUE = 0x0000FF;
  uint32_t ORANGE = 0xFF8000;
  uint32_t MAGENTA = 0xFF00FF;
  uint32_t SKYBLUE = 0x00FFFF;
} COLOR_32BIT;

struct data
{
  const uint8_t MAX_PWM = 250;
  int16_t pwm = 0;
  uint8_t seven_segment_high_byte = 0;
  uint8_t seven_segment_low_byte = 0;
  uint8_t motor_driver_id = 0;
  bool running_mode = 0;
  bool self_debug_direction = cw;
  bool uart_mode = 0;
  uint32_t pixel_color = 0;
  uint8_t seven_segment_pattern[10] = {
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

class Protocol
{
private:
  SoftwareSerial *uart;
  uint8_t id;
  void parse();

public:
  Protocol() {}
  virtual ~Protocol() {}
  void initialize();
  void update_id(uint8_t);
  int get_pwm();
  int receive();
};

void Protocol::initialize()
{
  uart = new SoftwareSerial(PIN_ASSIGN.SERIAL_RX, 0);
  uart->begin(SERIAL_CONFIG.SERIAL_BAUD);
  uart->listen();
}

void Protocol::update_id(uint8_t id)
{
  this->id = id;
}

int Protocol::receive()
{
  if (!(uart->available() > 2))
    return 0;
  if (uart->read() != 0xff)
    return 1;
  if (uart->read() != data.motor_driver_id)
    return 2;
  data.pwm = uart->read();
  if (uart->read() != 0)
    data.pwm = uart->read();
  return 3;
}

int Protocol::get_pwm()
{
  return 0;
}

/***********************************/
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

/********************************/

template <typename T>
class Observer
{
private:
  T cache;

public:
  Observer(){};
  virtual ~Observer(){};

  bool isChanged(T param);
  bool rise(T param);
};

template <typename T>
inline bool Observer<T>::isChanged(T param)
{
  bool flag = false;
  if (param != cache)
    flag = true;
  cache = param;
  return flag;
}

template <typename T>
inline bool Observer<T>::rise(T param)
{
  bool flag = false;
  if (param != cache && param >= 1)
    flag = true;
  cache = param;
  return flag;
}

Protocol protocol;
AntiChattering DIP_1(PIN_ASSIGN.DIP_1);
AntiChattering DIP_2(PIN_ASSIGN.DIP_2);
AntiChattering DIP_3(PIN_ASSIGN.DIP_3);
AntiChattering DIP_4(PIN_ASSIGN.DIP_4);
AntiChattering HIGH_LOW_IN_1(PIN_ASSIGN.HIGH_LOW_IN_1);
AntiChattering HIGH_LOW_IN_2(PIN_ASSIGN.HIGH_LOW_IN_2);
AntiChattering SOFTWARE_RESET(PIN_ASSIGN.SOFTWARE_RESET);
AntiChattering INPUT_MODE_DETECT(PIN_ASSIGN.INPUT_MODE_DETECT);
Observer<uint8_t> obs_DIP;
Observer<bool> obs_HIGH_LOW_1, obs_HIGH_LOW_2;
Adafruit_NeoPixel WS2812B(1, PIN_ASSIGN.WS2812B, NEO_GRB + NEO_KHZ800);
SPISettings SPI_LOWSPEED_SETTING = SPISettings(125000, MSBFIRST, SPI_MODE0);   //125kHz
SPISettings SPI_HIGHSPEED_SETTING = SPISettings(4000000, MSBFIRST, SPI_MODE0); //4.0MHz

int waitMs(uint16_t);
void update_motor_driver_id(uint8_t);
void update_seven_segment(uint8_t);
void update_output_pwm();
void initialize_switches();
void update_switches();
void wdt_reboot();
void setPwmFrequencyUNO(int, int);
void set_WS2818B(uint32_t);
void hardware_serial_debug();
bool contain2(String, String);
uint8_t get_DIP_states();

void hardware_serial_debug()
{
  if (!Serial.available())
    return;

  String command = Serial.readStringUntil('\n');
  if (contain2(command, "set") || contain2(command, "SET"))
  {
    if (contain2(command, "motor") || contain2(command, "MOTOR"))
    {
      Serial.print("@Enter PWM value (~");
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
    if (contain2(command, "led") || contain2(command, "LED"))
    {
      Serial.println("@Enter RGB value (32bit HEX)");
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
          set_WS2818B(data.pixel_color);
          break;
        }
      }
    }
  }
  else if (contain2(command, "get"))
  {
  }
}

void signal()
{
  analogWrite(PIN_ASSIGN.PWM_CW, 128);
  analogWrite(PIN_ASSIGN.PWM_CCW, 128);

  while (1)
  {
    WS2812B.setPixelColor(0, WS2812B.Color(0xff, 0xff, 0x00));
    WS2812B.show();
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

void setup()
{
  Serial.begin(256000);
  uint8_t timer = digitalPinToTimer(2);
  uint8_t bit = digitalPinToBitMask(2);
  uint8_t port = digitalPinToPort(2);
  volatile uint8_t *out;
  out = portOutputRegister(port);
  Serial.println(timer, BIN);
  Serial.println(bit, BIN);
  Serial.println(port, BIN);
  Serial.println(*out, BIN);
  while (1)
  {
  }

  Serial.begin(500000);
  Serial.println("BOOT");
  WS2812B.begin();
  WS2812B.clear();
  WS2812B.setBrightness(100);
  WS2812B.setPixelColor(0, WS2812B.Color(0xff, 0xff, 0xff));
  WS2812B.show();
  SPI.begin();
  pinMode(PIN_ASSIGN.SPI_SS, OUTPUT);
  protocol.initialize();
  initialize_switches();
  setPwmFrequencyUNO(PIN_ASSIGN.PWM_CW, 1);
  setPwmFrequencyUNO(PIN_ASSIGN.PWM_CCW, 1);
  // signal();
}
void loop()
{
  hardware_serial_debug();
  update_switches();
  if (waitMs(50))
  {
    data.motor_driver_id = get_DIP_states();
    if (obs_DIP.isChanged(data.motor_driver_id)) //test point2
    {
      update_motor_driver_id(data.motor_driver_id);
      update_seven_segment(data.motor_driver_id);
    }
  }

  //UART通信モードかHIGH/LOW入力モードかの状態を取得
  data.uart_mode = INPUT_MODE_DETECT.getState();
  // Serial.println(data.uart_mode);
  if (data.uart_mode) //UART通信入力モードのとき
  {
    if (protocol.receive() == 3)
    {
      data.pixel_color = COLOR_32BIT.GREEN;
      set_WS2818B(data.pixel_color);
      data.pwm = protocol.get_pwm();
    }
    if (!SOFTWARE_RESET.getState())
      wdt_reboot();
  }
  else
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
      set_WS2818B(data.pixel_color);
    }
  }

  update_output_pwm();
}

void update_motor_driver_id(uint8_t id)
{
  protocol.update_id(id);
}

void update_seven_segment(uint8_t num)
{
  num = constrain(num, 0, 99);
  data.seven_segment_high_byte = uint8_t(num / 10);
  data.seven_segment_low_byte = uint8_t(num % 10);
  Serial.println(data.seven_segment_high_byte);
  Serial.println(data.seven_segment_low_byte);
  //SPI OUTPUTS
  SPI.beginTransaction(SPI_LOWSPEED_SETTING);
  digitalWrite(PIN_ASSIGN.SPI_SS, LOW);
  digitalWrite(PIN_ASSIGN._74HC595_LATCH, LOW);
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

int waitMs(uint16_t milliseconds)
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
  Serial.println("@DETECT RESET SIGNAL. REBOOTING...");
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
      TCCR1B = (TCCR1B & (0b11111000 | divisor));
    }
  }
  else if (pin == 3 || pin == 11)
  {
    TCCR2B = (TCCR2B & (0b11111000 | divisor));
  }
}

void set_WS2818B(uint32_t color_32bits)
{
  static uint32_t prevColor = 0;
  if (prevColor == color_32bits)
    return;
  WS2812B.setPixelColor(0, color_32bits);
  WS2812B.show();
  prevColor = color_32bits;
}

bool contain2(String cmd, String head)
{
  // Serial.println(cmd.indexOf(head));
  if (cmd.indexOf(head) >= 0)
  {
    return true;
  }
  return false;
}