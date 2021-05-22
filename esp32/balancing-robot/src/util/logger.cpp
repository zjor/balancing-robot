#include <stdarg.h>
#include <Arduino.h>

#include "logger.h"

#define LOG_MAX_LENGTH  128

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