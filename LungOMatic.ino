//
//    Valve cycling code for Project Apollo v1 prototype
//

#include "pinout.h"
#include "oxygen_sensor.h"
#include "debug_console.h"
#include "BasicStepperDriver.h"
#include "A4988.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_RPM 3000
// Target RPM for Y axis motor

#define STOPPER_PIN 19
// 1 motor
#define DIR_Z 48
#define STEP_Z 46
#define SLEEP_Z 62
// 2 motor
#define DIR_Y 61
#define STEP_Y 60
#define SLEEP_Y 56
#define MICROSTEPS 16
// 100mm tube = 314 cm2
#define CYL_AREA 314
#define TIDAL_VOLUME 628

#define STEP_DIV 22

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
A4988 stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);
A4988 stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
bool isHome = false;
bool insp = false;
SyncDriver controller(stepperZ, stepperY);

void setup() {
  Serial.begin(115200);


  // Setup O2 sensor

  setup_o2sensor();
  // stepperZ.begin(MOTOR_RPM, MICROSTEPS);
  // stepperY.begin(MOTOR_RPM, MICROSTEPS);

  pinMode(62, OUTPUT);
  pinMode(56, OUTPUT);
  digitalWrite(62, LOW);
  digitalWrite(56, LOW);

  pinMode(STOPPER_PIN, INPUT_PULLUP);
  stepperZ.begin(MOTOR_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_RPM, MICROSTEPS);
  stepperY.enable();
  stepperZ.enable();


  home();


}

void move(int mm) {
  long quant = 100 * MOTOR_STEPS * MICROSTEPS / STEP_DIV;
  quant = quant * mm;
  controller.startMove(quant, quant);
  
}

void home() {

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

      /*
       * Choosing stop() vs startBrake():
       *
       * constant speed mode, they are the same (stop immediately)
       * linear (accelerated) mode with brake, the motor will go past the stopper a bit
       */

      stepperY.stop();
      stepperZ.stop();
      stepperY.disable();
      stepperZ.disable();
      digitalWrite(62, HIGH);
      digitalWrite(56, HIGH);
      // stepper.startBrake();
  }

  unsigned wait_time_micros = controller.nextAction();
  if (wait_time_micros <= 0) {
    if (insp) {
      insp = false;
      move(-(TIDAL_VOLUME / CYL_AREA) * 10);
    } else {
      insp = true;
      move((TIDAL_VOLUME / CYL_AREA) * 10);
    }
  }
}
