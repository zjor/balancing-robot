#ifndef _LOGGER_H
#define _LOGGER_H

class Logger {
public:
  Logger(bool debug) { _debug = debug; }
  void info(const char* format, ...);
  void debug(const char* format, ...);

private:
  bool _debug;
};

#endif