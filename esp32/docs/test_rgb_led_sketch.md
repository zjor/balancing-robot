# Test RGB LED sketch

```C++
#include <Arduino.h>

#define R_LED_PIN 32
#define G_LED_PIN 33
#define B_LED_PIN 25

#define R_CH 0
#define G_CH 1
#define B_CH 2

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;


RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

void setup() {
  Serial.begin(115200);
  ledcAttachPin(R_LED_PIN, R_CH);
  ledcAttachPin(G_LED_PIN, G_CH);
  ledcAttachPin(B_LED_PIN, B_CH);

  ledcSetup(R_CH, 1000, 8);
  ledcSetup(G_CH, 1000, 8);
  ledcSetup(B_CH, 1000, 8);

}
int i = 0;

void loop() {
  HsvColor hsv;
  hsv.h = i;
  hsv.s = 255;
  hsv.v = 255;

  RgbColor rgb = HsvToRgb(hsv);

  ledcWrite(R_CH, rgb.r);
  ledcWrite(G_CH, rgb.g);
  ledcWrite(B_CH, rgb.b);

  i++;
  if (i > 255) {
    i = 0;
  }
  delay(10);
}
```