#ifndef INDICATOR_H
#define INDICATOR_H

#define R_CH 0
#define G_CH 1
#define B_CH 2

class Indicator {
  public:
    Indicator(int redPin = 32, int greenPin = 33, int bluePin = 25);
    void init();
    void setColor(int color);  
  private:
    int _redPin;
    int _greenPin;
    int _bluePin;
};

#endif