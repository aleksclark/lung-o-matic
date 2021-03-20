//
//    Valve cycling code for Project Apollo v1 prototype
//

#include "pinout.h"
#include "oxygen_sensor.h"
#include "debug_console.h"


void setup() {
  Serial.begin(115200);


  // Setup O2 sensor
  setup_o2sensor();
}


void delay_with_dbg_info(int milliseconds)
{
  DBG_println_float("- Sleeping ", (float)milliseconds/1000, " seconds ...", 3);
  delay(milliseconds);
}


void halfCycle(int state_5way)
{
  // Read O2 value
  loop_o2sensor(100);

  delay_with_dbg_info(1000);
  

}

void loop() {
  DBG_println_buffered("- Start HIGH cycle ...")
  halfCycle(HIGH);
  DBG_println_buffered("- Start LOW cycle ...")
  halfCycle(LOW);  
}
