#ifndef _LOGGER_H
#define _LOGGER_H

#include <stdarg.h>
#include <Arduino.h>

#define LOG_MAX_LENGTH  128

class Logger {
public:
  Logger(bool debug) { _debug = debug; }
  void info(const char* format, ...);
  void debug(const char* format, ...);

private:
  bool _debug;
};

void Logger::info(const char* format, ...) {
  char buf[LOG_MAX_LENGTH];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  Serial.print(buf);
  va_end(args);  
}

void Logger::debug(const char* format, ...) {
  if (_debug) {
    char buf[LOG_MAX_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    Serial.print(buf);
    va_end(args);  
  }
}

#endif