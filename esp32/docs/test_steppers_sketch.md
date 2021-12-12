# Test Steppers Sketch

```C++
#define MOT1_EN 15
#define MOT1_DIR 14
#define MOT1_STEP 12

#define MOT2_EN 16
#define MOT2_DIR 26
#define MOT2_STEP 27

void do_step(int step_pin) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(4);
  digitalWrite(step_pin, LOW);  
}

void setup() {
  pinMode(MOT1_EN, OUTPUT);
  pinMode(MOT1_DIR, OUTPUT);
  pinMode(MOT1_STEP, OUTPUT);

  pinMode(MOT2_EN, OUTPUT);
  pinMode(MOT2_DIR, OUTPUT);
  pinMode(MOT2_STEP, OUTPUT);

  digitalWrite(MOT1_EN, LOW);
  digitalWrite(MOT2_EN, LOW);

  digitalWrite(MOT1_DIR, LOW);
  digitalWrite(MOT2_DIR, LOW);

  digitalWrite(MOT1_STEP, LOW);
  digitalWrite(MOT2_STEP, LOW);
  
}

void loop() {
  static int direction = HIGH;
  
  for (int i = 0; i < 200; i++) {
    do_step(MOT1_STEP);
    do_step(MOT2_STEP);
    delay(15);
  }
  direction = !direction;
  digitalWrite(MOT1_DIR, direction);
  digitalWrite(MOT2_DIR, direction);  
}
```