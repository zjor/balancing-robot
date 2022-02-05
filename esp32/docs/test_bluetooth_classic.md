# Test Bluetooth classic sketch

ESP32 supports both BLE and Bluetooth classic protocol.
Bluetooth classic will be used to control the robot via serial protocol.

```
#include "BluetoothSerial.h"

BluetoothSerial btSerial;

void setup() {
  Serial.begin(115200);
  btSerial.begin("ESP32");
}

void loop() {
  if (btSerial.available()) {
    int c = btSerial.read();
    Serial.print((char) c);
  }
}
```