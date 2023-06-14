#include "WString.h"
#ifndef __TOKEN_H__
#define __TOKEN_H__

template<typename T>
void print(const T &value) {
  Serial.print(value);
}

template<typename T>
void println(const T &value) {
  Serial.println(value);
}

String clean(String message) {
  message.trim();
  String result;
  bool blank = false;
  for (char c : message) {
    if (isPrintable(c) && !(c == ' ' && blank))
        result += c;
    blank = c == ' ' ? true : false;
  }
  return result;
}

String token(String message, int index) {
  message = clean(message);
  int n = message.length();
  int s = 0;
  int e = -1;
  for (int i = 0; i <= index; ++i) {
    s = e + 1;
    e = message.indexOf(' ', s);
    if (e < 0) {
      e = n;
      if (i != index)
        return String();
      break;
    }
  }
  return message.substring(s, e);
}

String command(const String &message) {
  return token(message, 0);
}

float argument(const String &message, int index, float default_value = 0) {
  String value = token(message, index);
  if (value == String()) 
    return default_value;
  return value.toFloat();
}

String argumentString(const String &message, int index, float default_value = 0) {
  String value = token(message, index);
  return value;
}

#endif