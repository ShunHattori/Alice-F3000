#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "SoftwareSerial.h"

class Protocol
{
private:
  SoftwareSerial &port;
  uint8_t id;
  void parse();

public:
  Protocol() {}
  virtual ~Protocol() {}
  void initialize(SoftwareSerial &);
  void update_id(uint8_t);
  int get_pwm();
  int receive();
};

void Protocol::initialize(SoftwareSerial &port)
{
  this->port = port;
}

void Protocol::update_id(uint8_t id)
{
  this->id = id;
}

int Protocol::receive()
{
  if (!(port.available() > 2))
    return 0;

  return 0;
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
  if (sampling_period > (millis() - sampled_time))
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

struct serial_comfig
{
  const uint16_t SERIAL_BAUD = 57600;
  const uint8_t RX_PIN = 2;
} SERIAL_CONFIG;

struct pin_assign
{
  const uint8_t PWM_CW = 9;
  const uint8_t PWM_CCW = 10;
  const uint8_t SERIAL_RX = SERIAL_CONFIG.RX_PIN; //2
  const uint8_t SOFTWARE_RESET = 3;
  const uint8_t SPI_MOSI = MOSI; //11
  const uint8_t SPI_SCK = SCK;   //13
  const uint8_t SPI_SS = 4;
  const uint8_t DIP_1 = 5;
  const uint8_t DIP_2 = 6;
  const uint8_t DIP_3 = 7;
  const uint8_t DIP_4 = 8;
  const uint8_t HIGH_LOW_IN_1 = 2;
  const uint8_t HIGH_LOW_IN_2 = 3;
  const uint8_t INPUT_MODE_DETECT = A0; //UART入力 or (HIGH/LOW&PWM直)入力を検出
  // const uint8_t INPUT_MODE_FLIP = A0;  //ハードウェアでの実装
  // const uint8_t SELF_DEBUG_SLIDE = A1;
  // const uint8_t SELF_DEBUG_TACT = A2;
} PIN_ASSIGN;

enum
{
  cw,
  ccw,
};

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
} data;

SoftwareSerial uart(PIN_ASSIGN.SERIAL_RX, 0);
Protocol protocol;
AntiChattering DIP_1(PIN_ASSIGN.DIP_1);
AntiChattering DIP_2(PIN_ASSIGN.DIP_2);
AntiChattering DIP_3(PIN_ASSIGN.DIP_3);
AntiChattering DIP_4(PIN_ASSIGN.DIP_4);
AntiChattering HIGH_LOW_IN_1(PIN_ASSIGN.HIGH_LOW_IN_1);
AntiChattering HIGH_LOW_IN_2(PIN_ASSIGN.HIGH_LOW_IN_2);
AntiChattering SOFTWARE_RESET(PIN_ASSIGN.SOFTWARE_RESET);
AntiChattering INPUT_MODE_DETECT(PIN_ASSIGN.INPUT_MODE_DETECT);
// AntiChattering INPUT_MODE_FLIP(PIN_ASSIGN.INPUT_MODE_FLIP);
// AntiChattering SELF_DEBUG_SLIDE(PIN_ASSIGN.SELF_DEBUG_SLIDE);
// AntiChattering SELF_DEBUG_TACT(PIN_ASSIGN.SELF_DEBUG_TACT);
Observer<uint8_t>
    obs_DIP;

int waitMs(uint16_t);
void update_motor_driver_id();
void update_seven_segment();
void update_output_pwm();
void initialize_switches();
void update_switches();
void wdt_reboot();

uint8_t get_DIP_states();

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  uart.begin(SERIAL_CONFIG.SERIAL_BAUD);
  uart.listen();
  protocol.initialize(uart);
  initialize_switches();
}

void loop()
{
  update_switches();
  if (waitMs(1000))
  {
    if (obs_DIP.isChanged(get_DIP_states()))
    {
      update_motor_driver_id();
      update_seven_segment();
      protocol.update_id(data.motor_driver_id);
    }
  }

  //UART通信モードかHIGH/LOW入力モードかの状態を取得
  data.uart_mode = INPUT_MODE_DETECT.getState();

  if (data.uart_mode) //UART通信入力モードのとき
  {
    protocol.receive();
    data.pwm = protocol.get_pwm();
    if (SOFTWARE_RESET.getState())
      wdt_reboot();
  }
  else
  {
    if (HIGH_LOW_IN_1.getState())
      data.pwm = (1) * get_DIP_states() * 16;
    else if (HIGH_LOW_IN_2.getState())
      data.pwm = (-1) * get_DIP_states() * 16;
    else
      data.pwm = 0;
  }

  update_output_pwm();
}

void update_motor_driver_id()
{
  //スイッチから計算したidをdata.motor_driver_idに書き込み
  protocol.update_id(get_DIP_states());
}

void update_seven_segment()
{
  //SPI出力
}

uint8_t get_DIP_states()
{
  uint8_t state = (DIP_1.getState() >> 0) |
                  (DIP_2.getState() >> 1) |
                  (DIP_3.getState() >> 2) |
                  (DIP_4.getState() >> 3);
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
  while (1)
  {
  }
}
