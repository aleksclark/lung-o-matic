
#include <Arduino.h>

//
// Gasboadd 7500E O2 sensor
//

#include "gasboard7500E.h"
#include <SoftwareSerial.h>

#include "pinout.h"
#include "oxygen_sensor.h"
#include "debug_console.h"


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
    DBG_print("Hello ");

    delay(3000); // power on delay
    #ifdef O2SENSE_NEED_METADATA
  
    // I don't think these commands are working
  
    softSer.write((uint8_t*)cmd_vernum, 4);
    delay(500);
    softSer.write((uint8_t*)cmd_sernum, 4);
    delay(500);
    #endif
    DBG_println("World");
  }

  DBG_println("Gasboard 7500E sensor initialized!");
}


// Print sensor values. Constrain execution time in the given number of milliseconds 
void loop_o2sensor(int wait_after_ms)
{
  unsigned long time_start_ms = time_start_ms = millis();
  
  for(int i = 0; i < bytes_per_packet; i++)
  {
    if (softSer.available()) // at least 1 byte from UART arrived
    {
      DBG_println("got a byte");
      o2sens_feedUartByte(softSer.read()); // give byte to the parser
      uint8_t* all_data = o2sens_getRawBuffer();
      for (int i = 0; i < all_data[1] + 3; i++)
          {
            DBG_print_buffered("%02X ", all_data[i]);
          }
      if (o2sens_hasNewData()) // a complete packet has been processed
      {
        DBG_println("new_data");
        o2sens_clearNewData(); // clear the new packet flag
        DBG_println_buffered(", %u , %u , %u"
            , o2sens_getConcentration16()
            , o2sens_getFlowRate16()
            , o2sens_getTemperature16()
            );
        // print out the whole buffer for debugging


        // if (sensor_log_enabled)
        // {
        //   // display time in milliseconds for logging
        //   DBG_print(millis(), DEC); 
        //   DBG_print(": ");
    
          

        //   // print out the human readable data
        //   DBG_println_buffered(", %u , %u , %u"
        //     , o2sens_getConcentration16()
        //     , o2sens_getFlowRate16()
        //     , o2sens_getTemperature16()
        //     );

        //   if (has_sernum == false)
        //   {
        //     uint8_t* data = o2sens_getSerialNumber();
        //     if (data[0] != 0 || data[1] != 0 || data[2] != 0 || data[3] != 0 || data[4] != 0)
        //     {
        //       has_sernum = true;
        //       DBG_println_buffered("Serial Number: %02X %02X %02X %02X %02X"
        //         , data[0]
        //         , data[1]
        //         , data[2]
        //         , data[3]
        //         , data[4]
        //         );
        //     }
        //   }
        //   if (has_vernum == false)
        //   {
        //     char* data = o2sens_getVersionNumber();
        //     if (data[0] != 0) // first byte 
        //     {
        //       has_vernum = true;
        //       DBG_println_buffered("Version Number: %s", data);
        //     }
        //   }
        // }

        // float o2concentration = (float)o2sens_getConcentration16()/10;
        // float o2flow = (float)o2sens_getFlowRate16()/10;
        // float ambientTemperature = o2sens_getTemperature16()/10;

        // DBG_println("- Sensor data:");
        // DBG_println_float("     O2 concentration = ", o2concentration, "%", 1);
        // DBG_println_float("     Flow = ", o2flow, " liter/min", 1);
        // DBG_println_float("     Temperature = ", ambientTemperature, " Celsius", 1);
      }
    } else {
      DBG_print("no byte");
    }
  }

  unsigned long time_stop_ms = millis();
  int time_delta_ms = (int)(time_stop_ms - time_start_ms);
  if (sensor_log_timing_enabled)
  {
    DBG_println_buffered("- Time spent in loop_sensor() = %d milliseconds", time_delta_ms);
  }

  int time_left_to_wait_ms = wait_after_ms - time_delta_ms;
  delay(time_left_to_wait_ms);
}
