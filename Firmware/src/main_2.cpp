// #include "Arduino.h"
// // #include <Wire.h>
// #include <SimpleFOC.h>
// #include <SimpleFOCDrivers.h>
// #include "drivers/tmc6200/TMC6200.hpp"


// // appropriate ESP32 hardware pins
// #define DRV_EN 45
// #define FAULT 35
// #define HALL_A 40
// #define HALL_B 41
// #define HALL_C 42
// #define STATUS_LED 4

// // SPI multiplexer
// #define CS_A0 5
// #define CS_A1 6
// #define CS_A2 7

// // BLDCMotor motor = BLDCMotor(11);
// TMC6200Driver6PWM driver = TMC6200Driver6PWM(9, 10, 11, 12, 13, 14, CS_A0, DRV_EN);

// // void printTMC6200Status() {
// // 	TMC6200GStatus status = driver.getStatus();
// // 	Serial.println("TMC6200 Status:");
// // 	Serial.print("Reset: ");
// // 	Serial.println(status.isReset());

// //   Serial.print("Over temperature pre warning: ");
// //   Serial.println(status.isOverTemperaturePreWarning());

// //   Serial.print("Over temperature: ");
// //   Serial.println(status.isOverTemperature());

// //   Serial.print("Charge pump under voltage: ");
// //   Serial.println(status.isChargePumpUnderVoltage());

// //   Serial.print("U shorted: ");
// //   Serial.println(status.hasUShorted());

// //   Serial.print("V shorted: ");
// //   Serial.println(status.hasVShorted());

// //   Serial.print("W shorted: ");
// //   Serial.println(status.hasWShorted());

// //   Serial.print("U shorted ground: ");
// //   Serial.println(status.isUShortedToGround());

// //   Serial.print("U shorted supply: ");
// //   Serial.println(status.isUShortedToSupply());

// //   Serial.print("V shorted ground: ");
// //   Serial.println(status.isVShortedToGround());

// //   Serial.print("V shorted supply: ");
// //   Serial.println(status.isVShortedToSupply());

// //   Serial.print("W shorted ground: ");
// //   Serial.println(status.isWShortedToGround());

// //   Serial.print("W shorted supply: ");
// //   Serial.println(status.isWShortedToSupply());


// // 	if (digitalRead(FAULT)) {
// //     digitalWrite(DRV_EN, LOW);
// //     delayMicroseconds(5);
// //     digitalWrite(DRV_EN, HIGH);
// //   }
// // }





// void setup() {
// 	delay(3000);
// 	Serial.begin(115200);

//     pinMode(UL, OUTPUT);
//     pinMode(VL, OUTPUT);
//     pinMode(WL, OUTPUT);
    
//     digitalWrite(WL, HIGH);
//     digitalWrite(UL, HIGH);
//     digitalWrite(VL, HIGH);

//   // pinMode(FAULT, INPUT);

// 	Serial.println("INITIALIZING");

// 	// driver.voltage_power_supply = 12;
// 	driver.init();
// 	Serial.println("Driver initialized");
// 	// motor.linkDriver(&driver);
// 	// motor.controller = MotionControlType::velocity_openloop;
// 	// motor.voltage_limit = 3;
// 	// motor.velocity_limit = 20;
// 	// motor.init();
// 	Serial.println("Init complete...");

// 	delay(100);
// 	// printTMC6200Status();
// }


// // // velocity set point variable
// // float target_velocity = 7.0;


// void loop() {
// 	// motor.move(target_velocity);
// }