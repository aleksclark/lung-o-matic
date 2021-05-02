
#include <Arduino.h>

//
// Gasboadd 7500E O2 sensor
//

#include <Pinout.h>
#include "gasboard7500E.h"
#include <SoftwareSerial.h>

#include "OxygenSensor.h"
#include <DebugConsole.h>



SoftwareSerial softSer = SoftwareSerial(O2SENS_RX_PIN, O2SENS_TX_PIN);


#ifdef O2SENSE_NEED_METADATA
bool has_sernum = false;
bool has_vernum = false;
#endif

// Bytes per UART packet
const int bytes_per_packet = 30;
const bool sensor_log_enabled = true;
const bool sensor_log_timing_enabled = true;

void setup_o2sensor()
{
  #ifdef O2SENSE_NEED_METADATA
  const uint8_t cmd_vernum[] = {O2SENSE_CMD_VERSIONNUMBER};
  const uint8_t cmd_sernum[] = {O2SENSE_CMD_SERIALNUMBER};
  #endif

  delay(100);

  o2sens_init();
  pinMode(O2SENS_RX_PIN, INPUT);
  pinMode(O2SENS_TX_PIN, OUTPUT);
  softSer.begin(O2SENSE_BAUD_RATE);
  
  if (sensor_log_enabled)
  {

    delay(3000); // power on delay
    #ifdef O2SENSE_NEED_METADATA
  
    softSer.write((uint8_t*)cmd_vernum, 4);
    delay(500);
    softSer.write((uint8_t*)cmd_sernum, 4);
    delay(500);
    #endif
  }

  DBG_println("Gasboard 7500E sensor initialized!");
}


// Print sensor values. Constrain execution time in the given number of milliseconds 
void loop_o2sensor(int wait_after_ms)
{

  const uint8_t cmd_read[] = {O2SENSE_CMD_READ_DATA};
  unsigned long time_start_ms = time_start_ms = millis();
  softSer.write((uint8_t*)cmd_read, 4);
  delay(20);
  for(int i = 0; i < bytes_per_packet; i++)
  {
    if (softSer.available()) // at least 1 byte from UART arrived
    {

      o2sens_feedUartByte(softSer.read()); // give byte to the parser
      if (o2sens_hasNewData()) // a complete packet has been processed
      {

        o2sens_clearNewData(); // clear the new packet flag

        float o2concentration = (float)o2sens_getConcentration16()/10;
        float o2flow = (float)o2sens_getFlowRate16()/10;
        float ambientTemperature = o2sens_getTemperature16()/10;

        DBG_println("- Sensor data:");
        DBG_println_float("     O2 concentration = ", o2concentration, "%", 1);
        DBG_println_float("     Flow = ", o2flow, " liter/min", 1);
        DBG_println_float("     Temperature = ", ambientTemperature, " Celsius", 1);
      }
    }
  }

  unsigned long time_stop_ms = millis();
  int time_delta_ms = (int)(time_stop_ms - time_start_ms);
  if (sensor_log_timing_enabled)
  {
    DBG_println_buffered("- Time spent in loop_sensor() = %d milliseconds", time_delta_ms);
  }


}
