//
//    Valve cycling code for Project Apollo v1 prototype
//
#include <Arduino.h>
#include <Pinout.h>
#include <OxygenSensor.h>
#include <DebugConsole.h>

#include "A4988.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

#define MICROSTEPS 4
#define DIR_Z 48
#define STEP_Z 46
#define SLEEP_Z 62
#define DIR_Y 61
#define STEP_Y 60
#define SLEEP_Y 56

#define STOPPER_PIN 19
#define STEP_DIV 8 // steps per mm of travel


#define CYL_AREA 78.54
#define WORKING_LEN 30


// Respiratory Cycle Parameters
#define HALF_CYCLE 2 // seconds, each for expiration/inspiration
#define TIDAL_VOLUME 350 //cc/ml
#define RESERVE_VOLUME 1000


// Derived values
float cycleTravelDistance = TIDAL_VOLUME / CYL_AREA;
float initialTravelDistance = WORKING_LEN - (RESERVE_VOLUME / CYL_AREA) - cycleTravelDistance;
float rotationPerCycle = (cycleTravelDistance / STEP_DIV) * 10; //10mm = 1cm
int motorRPM = (rotationPerCycle / HALF_CYCLE) * 60;

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
A4988 stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);
A4988 stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
bool isHome = false;
bool insp = false;
bool readTime = false;
SyncDriver controller(stepperZ, stepperY);

void setup() {
  Serial.begin(115200);
  Serial.println("BEGIN");
  Serial.println("Calcultated RPM:");
  Serial.println(motorRPM);
  Serial.println("cycleTravelDistance");
  Serial.println(cycleTravelDistance);
  Serial.println("initialTravelDistance");
  Serial.println(initialTravelDistance);

  // Setup O2 sensor

  setup_o2sensor();

  pinMode(62, OUTPUT);
  pinMode(56, OUTPUT);
  digitalWrite(62, LOW);
  digitalWrite(56, LOW);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);

  pinMode(STOPPER_PIN, INPUT_PULLUP);

  stepperZ.setSpeedProfile(A4988::LINEAR_SPEED, 4000, 4000);
  stepperY.setSpeedProfile(A4988::LINEAR_SPEED, 4000, 4000);

  stepperZ.begin(motorRPM, MICROSTEPS);
  stepperY.begin(motorRPM, MICROSTEPS);
  stepperY.enable();
  stepperZ.enable();


  home();
  Serial.println("initial move");
  Serial.println(initialTravelDistance * 10);

  move(initialTravelDistance * 10);

}

void move(float mm) {
  long quant = MOTOR_STEPS * MICROSTEPS / STEP_DIV;
  quant = quant * mm;
  controller.startMove(-quant, -quant);
  
}

void home() {
  move(10);
  move(-350);
  while(!isHome){
      // statement
      if (digitalRead(STOPPER_PIN) == LOW){
            Serial.println("STOPPER REACHED");

            /*
             * Choosing stop() vs startBrake():
             *
             * constant speed mode, they are the same (stop immediately)
             * linear (accelerated) mode with brake, the motor will go past the stopper a bit
             */

            stepperY.stop();
            stepperZ.stop();
            // stepperY.disable();
            // stepperZ.disable();
            // digitalWrite(62, HIGH);
            // digitalWrite(56, HIGH);
            move(10);
            isHome = true;
            // stepper.startBrake();
        }
      controller.nextAction();
  }

  bool moving = true;
  while(moving) {
    unsigned wait_time_micros = controller.nextAction();
    if (wait_time_micros <= 0) {
      moving = false;
    }
  }

}

// void halfCycle(int state_5way)
// {
//   // Read O2 value
//   // loop_o2sensor(100);

//   delay_with_dbg_info(1000);
  

// }

void loop() {
  if (digitalRead(STOPPER_PIN) == LOW){
      Serial.println("STOPPER REACHED");

      stepperY.stop();
      stepperZ.stop();
      stepperY.disable();
      stepperZ.disable();
      digitalWrite(62, HIGH);
      digitalWrite(56, HIGH);

  }

  unsigned wait_time_micros = controller.nextAction();
  if (wait_time_micros <= 0) {
    if (insp) {
      insp = false;
      move(-cycleTravelDistance * 10);
    } else {
      insp = true;
      loop_o2sensor(100);
      move(cycleTravelDistance * 10);
    }
  }
}
