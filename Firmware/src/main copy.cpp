// #include <Arduino.h>
// #include <SimpleFOC.h>
// #include "registers.h"
// #include "SimpleFOCDrivers.h"
// #include "drivers/tmc6200/TMC6200.hpp"
// #include <Adafruit_INA219.h> 
// #include "Wire.h"


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

// // Appropriate BLDC constants, adjust these per your needs
// #define V_IN 12
// #define PP 6


// BLDCMotor motor = BLDCMotor(PP);
// // TMC6200Driver6PWM driver = TMC6200Driver6PWM(9, 10, 11, 12, 13, 14, -1);
// TMC6200Driver6PWM driver = TMC6200Driver6PWM(9, 10, 11, 12, 13, 14, CS_A0, DRV_EN);
// // InlineCurrentSense current_sense  = InlineCurrentSense(0.033, 10, 17, 16, 15);
// static float internal_velocity = 0;


// // // Encoder sensor = Encoder(41, 42, 600);
// // MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// // // HallSensor sensor = HallSensor(HALL_A, HALL_B, HALL_C, PP);
// // // void doA(){sensor.handleA();}
// // // void doB(){sensor.handleB();}
// // // void doC(){sensor.handleC();}

// // // commander interface
// // Commander command = Commander(Serial);
// // void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
// // void onMotor(char* cmd){ command.motor(&motor, cmd); }

// // TMC6200GStatus printDriverStatus() {
// //     TMC6200GStatus status = driver.getStatus();
// //     Serial.println("");
// //     Serial.println("");
// //     Serial.print("hasUShorted: "); Serial.println(status.hasUShorted());
// //     Serial.print("hasVShorted: "); Serial.println(status.hasVShorted());
// //     Serial.print("hasWShorted: "); Serial.println(status.hasWShorted());
// //     Serial.print("isUShortedToGround: "); Serial.println(status.isUShortedToGround());
// //     Serial.print("isUShortedToSupply: "); Serial.println(status.isUShortedToSupply());
// //     Serial.print("isVShortedToGround: "); Serial.println(status.isVShortedToGround());
// //     Serial.print("isVShortedToSupply: "); Serial.println(status.isVShortedToSupply());
// //     Serial.print("isWShortedToGround: "); Serial.println(status.isWShortedToGround());
// //     Serial.print("isWShortedToSupply: "); Serial.println(status.isWShortedToSupply());
// //     Serial.print("isOverTemperaturePreWarning: "); Serial.println(status.isOverTemperaturePreWarning());
// //     Serial.print("isChargePumpUnderVoltage: "); Serial.println(status.isChargePumpUnderVoltage());

// //     // print the status register
// //     Serial.print("Status register: "); Serial.println(status.status.reg);

// //     return status;
// // }

// // void handlePotentialDriverFault(int verbose = 1)
// // {
// //   if (digitalRead(FAULT)) {
// //     Serial.println("Driver faults detected!");
// //     if (verbose) printDriverStatus();

// //     // the driver must be cycled to clear the fault
// //     digitalWrite(DRV_EN, LOW);
// //     delay(5);
// //     digitalWrite(DRV_EN, HIGH);
// //     delay(5);
// //     Serial.println("Status cleared");
// //     Serial.println("");
// //     Serial.println("");

// //   }
// //   else {
// //     Serial.println("No driver faults detected");
// //   }
// // }


// void setup() {
//   // Wire.begin(41, 42);
//   delay(3000);
//   Serial.begin(115200);

//   pinMode(CS_A0, OUTPUT);
//   pinMode(CS_A1, OUTPUT);
//   pinMode(CS_A2, OUTPUT);
//   pinMode(STATUS_LED, OUTPUT);
//   pinMode(DRV_EN, OUTPUT);
//   pinMode(FAULT, INPUT);

//   digitalWrite(CS_A0, HIGH);
//   digitalWrite(CS_A1, LOW);
//   digitalWrite(CS_A2, LOW);

//   // printDriverStatus();
  
//   // clear any existing TMC6200 faults
//   // handlePotentialDriverFault();

//   // // reset the TMC6200
//   // digitalWrite(DRV_EN, LOW);
//   // delay(1000);
//   // digitalWrite(DRV_EN, HIGH);
//   // digitalWrite(STATUS_LED, HIGH);
//   // Serial.println("Driver reset");

//   // motor.sensor_direction = CCW;

//   // sensor.init();
//   // Serial.println("Sensor initialized");
//   // // sensor.enableInterrupts(doA, doB, doC);
//   // motor.linkSensor(&sensor);
//   // Serial.println("Sensor linked to motor");

//   // handlePotentialDriverFault();

//   // driver.voltage_power_supply = V_IN;
//   // driver.init();
//   // Serial.println("Driver initialized");

//   // motor.linkDriver(&driver);
//   // Serial.println("Driver linked to motor");

//   Serial.print("reg1: ");Serial.println(driver.getInputs().reg);
//   Serial.print("reg2: ");Serial.println(driver.getInputs().reg);
//   while(driver.getInputs().VERSION != TMC6200_VERSION){
//     Serial.print("reg: ");Serial.println(driver.getInputs().reg);
//     Serial.print("VERSION: ");Serial.println(driver.getInputs().VERSION);
//     Serial.println("Error: TMC6200 not found");
//   }

//   Serial.println("VERSION == TMC6200_VERSION!!!");

//   // motor.voltage_limit = 20;
//   // motor.voltage_sensor_align = 2;
//   // motor.LPF_velocity.Tf = 0.5f;

//   // motor.useMonitoring(Serial);

//   // // set motion control loop to be used
//   // motor.controller = MotionControlType::torque;
//   // // motor.controller = MotionControlType::velocity;

//   // // initialise motor
//   // motor.init();

//   // // optional current sense, not fully tested
//   // // current_sense.init();
//   // // motor.linkCurrentSense(&current_sense);

//   // // align encoder and start FOC
//   // motor.initFOC();

//   // // set the inital target value
//   // motor.target = 0;

//   // command.add('T', doTarget, "target voltage");

//   // // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
//   // Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 0V."));

//   // delay(1000);
// }

// void loop() {

//   // // main FOC algorithm function
//   // motor.loopFOC();

//   // // Motion control function
//   // motor.move();

//   // // user communication
//   // command.run();
// }