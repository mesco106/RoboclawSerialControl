#ifndef __GPIO_H__
#define __GPIO_H__

class Gpio {
public:
  Gpio(int pin) : m_pin(pin) { }
  void setup() { pinMode(m_pin, OUTPUT); }
  void on() { digitalWrite(m_pin, HIGH); }
  void off() { digitalWrite(m_pin, LOW); }
private:
  int m_pin;
};

#endif