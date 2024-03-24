#include "Arduino.h"
// // #include <Wire.h>
// #include <SimpleFOC.h>
// #include <SimpleFOCDrivers.h>
// #include "drivers/tmc6200/TMC6200.hpp"


// appropriate ESP32 hardware pins
#define STATUS_LED 4

void setup() {
	delay(3000);
	Serial.begin(115200);

    pinMode(STATUS_LED, OUTPUT);

    delay(100);

}

void loop() {
	// motor.move(target_velocity);
    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
}