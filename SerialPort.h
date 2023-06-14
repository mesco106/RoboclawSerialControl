#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#include "Command.h"

class SerialPort {
public:
  using ReceivedEvent = void (*)(const String &);
  SerialPort(long baud, ReceivedEvent callback) : m_baud(baud), m_received(callback) { }
  void setup() {
    Serial.begin(m_baud);
  }
  void update() {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == m_delimeter) {
        m_received( clean(m_buffer) );
        m_buffer = String();
      } else
        m_buffer += c;
    }
  }
  void send(const String &message) {
    Serial.print(message + m_delimeter);
  }
private:
  long m_baud;
  ReceivedEvent m_received;
  const char m_delimeter = '\n';
  String m_buffer;
};

#endif