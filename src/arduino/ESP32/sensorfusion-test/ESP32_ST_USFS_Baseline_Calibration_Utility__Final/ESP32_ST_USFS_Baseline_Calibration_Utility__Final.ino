//the code based on Gregory Tomasch::

/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */
 /*
   This utility is written specifically to work with the "Dragonfly" and "Butterfly" STM32L4 microcontroller development boards
 */

#include <Wire.h>
#include "I2Cdev.h"
#include "EM7180.h"
#include "IMU.h"
#include "Alarms.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"
#include <ETH.h>
#include <SimpleFTPServer.h>
#include "FS.h"
#include "SD_MMC.h"
#include <Adafruit_GPS.h>
#include <time.h>



FtpServer ftpSrv;
// Define function pointers for class instances
I2Cdev      *i2c_0;
EM7180      *Sentral_0;
IMU         *imu_0;


const char* NTP_SERVER = "pool.ntp.org";
const char* TZ_INFO    ="UTC";
time_t now;
tm tm;
bool calib_start=false;


#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();
int delay_loop=1000;


void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace){
	Serial.print(">>>>>>>>>>>>>>> _callback " );
	Serial.print(ftpOperation);
	/* FTP_CONNECT,
	 * FTP_DISCONNECT,
	 * FTP_FREE_SPACE_CHANGE
	 */
	Serial.print(" ");
	Serial.print(freeSpace);
	Serial.print(" ");
	Serial.println(totalSpace);

	// freeSpace : totalSpace = x : 360

	if (ftpOperation == FTP_CONNECT) Serial.println(F("CONNECTED"));
	if (ftpOperation == FTP_DISCONNECT) Serial.println(F("DISCONNECTED"));
};


void _transferCallback(FtpTransferOperation ftpOperation, const char* name, unsigned int transferredSize){
	Serial.print(">>>>>>>>>>>>>>> _transferCallback " );
	Serial.print(ftpOperation);
	/* FTP_UPLOAD_START = 0,
	 * FTP_UPLOAD = 1,
	 *
	 * FTP_DOWNLOAD_START = 2,
	 * FTP_DOWNLOAD = 3,
	 *
	 * FTP_TRANSFER_STOP = 4,
	 * FTP_DOWNLOAD_STOP = 4,
	 * FTP_UPLOAD_STOP = 4,
	 *
	 * FTP_TRANSFER_ERROR = 5,
	 * FTP_DOWNLOAD_ERROR = 5,
	 * FTP_UPLOAD_ERROR = 5
	 */
	Serial.print(" ");
	Serial.print(name);
	Serial.print(" ");
	Serial.println(transferredSize);
};



// Declare main loop helper functions
void DRDY_Handler();
void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM);

void DRDY_Handler()
{
  drdy = 1;
}

void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
    eventStatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if(eventStatus[sensorNUM] & 0x04) Quat_flag[sensorNUM] = 1;
    if(eventStatus[sensorNUM] & 0x20) Gyro_flag[sensorNUM] = 1;
    if(eventStatus[sensorNUM] & 0x10) Acc_flag[sensorNUM]  = 1;
    if(eventStatus[sensorNUM] & 0x08) Mag_flag[sensorNUM]  = 1;
    if(eventStatus[sensorNUM] & 0x40) Baro_flag[sensorNUM] = 1;
    algostatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
}

void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM)
{
  if(Gyro_flag[sensorNUM] == 1)
  {
    em7180->Gyro_getADC();
    for(uint8_t i=0; i<3; i++)
    {
      gyroData[sensorNUM][i] = (float)gyroADC[sensorNUM][i]*DPS_PER_COUNT;
    }
    Gyro_flag[sensorNUM] = 0;
  }
  if(Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
  if(Acc_flag[sensorNUM])
  {
    em7180->ACC_getADC();
    em7180->ACC_Common();
    for(uint8_t i=0; i<3; i++)
    {
      accData[sensorNUM][i] = (float)accADC[sensorNUM][i]*G_PER_COUNT;
      LINaccData[sensorNUM][i] = (float)accLIN[sensorNUM][i]*G_PER_COUNT;
    }
    Acc_flag[sensorNUM] = 0;
  }
  if(Mag_flag[sensorNUM])
  {
    em7180->Mag_getADC();
    for(uint8_t i=0; i<3; i++)
    {
      magData[sensorNUM][i] = (float)magADC[sensorNUM][i]*SENTRAL_UT_PER_COUNT;
    }
    Mag_flag[sensorNUM] = 0;
  }
  if(Baro_flag[sensorNUM])
  {
    rawPressure[sensorNUM]    = em7180->Baro_getPress();
    pressure[sensorNUM]       = (float)rawPressure[sensorNUM]*0.01f +1013.25f;                                       // Pressure in mBar
    rawTemperature[sensorNUM] = em7180->Baro_getTemp();
    temperature[sensorNUM]    = (float)rawTemperature[sensorNUM]*0.01;                                               // Temperature in degrees C
    Baro_flag[sensorNUM]      = 0;
  }
}



void setup()
{
  // Open serial ports
  Serial.begin(115200);
  GPS.begin(9600);

  configTime(0, 0, NTP_SERVER);
  setenv("TZ", TZ_INFO, 1);


    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  GPSSerial.println(PMTK_Q_RELEASE);

  delay(1000);
  ETH.begin();
  delay(5000);
  //ETH.begin(arduinoIP, dnsIP, gatewayIP, subnetIP);
  // ETH.config(IPAddress(192, 168, 1, 101),IPAddress(192, 168, 1, 1),IPAddress(255, 255, 255, 0),IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1));
  Serial.println(ETH.localIP());
  delay(1000);
  
  if (SD_MMC.begin("/sdcard",false, false, 40000,5 )) {
      Serial.println("SD opened!");
      ftpSrv.setCallback(_callback);
      ftpSrv.setTransferCallback(_transferCallback);

      ftpSrv.begin("esp","esp");    //username, password for ftp.   (default 21, 50009 for PASV)
      delay(5000);
      ftpSrv.setLocalIp(ETH.localIP());

      delay(5000);
  }


  // Assign interrupt pin as input
  pinMode(INT_PIN, INPUT);
   
  // Instantiate Wire class for I2C
  SENSOR_0_WIRE_INSTANCE.begin(SDA_PIN, SCL_PIN);
  delay(100);
  SENSOR_0_WIRE_INSTANCE.setClock(400000);
  delay(500);
  
  // Instantiate Sentral_0 classes and create function pointers
  i2c_0     = new I2Cdev(&SENSOR_0_WIRE_INSTANCE);                                                                   // Use class instance/function pointer to specify I2C bus (may be more than one)
  Sentral_0 = new EM7180(i2c_0, 0);                                                                                  // Use class instance/function pointer to specify Sentral board (may be more than one)
  imu_0     = new IMU(Sentral_0, 0);                                                                                 // Use class instance/function pointer to specify the Sentral and I2C instances for the IMU calcs

  // Initialize Sentral_0
  #ifdef SERIAL_DEBUG
    Serial.print("Initializing Sentral_0");
    Serial.println("");
  #endif

  // Main function to set up Sentral and sensors 
  Sentral_0->initSensors();
  Serial.println("Init USFS ok!");

  // Give a little time to see startup results
  delay(10000);


  // Spreadsheet output column headings when "SERIAL_DEBUG" is not defined in config.h
  // #ifndef SERIAL_DEBUG
  //   Serial.print("Timestamp, q0, q1, q2, q3, Heading, Pitch, Roll"); Serial.println("");
  // #endif

  // Attach adat ready interrupt
  attachInterrupt(INT_PIN, DRDY_Handler, RISING);



  // Set sketch start time
  Start_time = micros();
  start=millis();


  delay(1000);
  // int delt_t_2;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{

  ftpSrv.handleFTP(); 


    // read data from the GPS in the 'main loop'
  char c = GPS.read();


  // if you want to debug, this is a good time to do it!
  //Serial.println("GPSecho?:"+String(GPSECHO));
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
   

  //Serial.print("Fix: "); Serial.println(String(GPS.milliseconds));

  actualtime=millis()-start;

  // Acquire data the Sentral
  FetchSentralData(Sentral_0, imu_0, 0); 


  if (actualtime<600000|| calib_start) {



    if(Serial.available()) {
      
      delay(5000);
      serial_input = Serial.read();
      Serial.println(serial_input);
      delay(5000);

      
    }

    //Input==3
    if (serial_input==51) {
    calib_start=true;
    Serial.println("Cal loop start");
    //Serial.println(calibratingA[0]);
    delay(5000);
    }

    //input==4
    if (serial_input==52) {
    calib_start=false;
    Serial.println("Cal loop stopped");
    delay(5000);
    }

    if(calibratingA[0]>1){
     
     delay_loop=0;

    }

    else{

      delay_loop=1000;
    }
    
    //serial_input = Serial.read();

    if (serial_input == 49 && calibratingA[0] < 1) {
      Serial.println("Cal aquisition start");
      delay(5000);
      calibratingA[0] = 512;;

      //Serial.println("Cal value ready");
      //delay(5000);

    }                                           // Type "1" to initiate Sentral_0 Accel Cal

    //delay(5000);
    if (serial_input == 50 && calibratingA[0] < 1) {Sentral_0->Save_Sentral_WS_params();}                              // Type "2" to initiate Sentral_0 "Warm Start" parameter save
    serial_input = 0;

    //See what data are available from the Sentral by polling the Status register
    if(drdy == 1)
    {
      drdy = 0;
      FetchEventStatus(i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly Sentral instance 0)
     }

       
    Serial.println("###############################################################");
      //Algorithm status
    Serial.print("Algorithm Status = "); Serial.print(algostatus[0]); Serial.println(""); Serial.println("");

    //Sentral_0 sensor and raw quaternion outout
    Serial.print("ax_0 = "); Serial.print((int)(1000.0f*accData[0][0])); Serial.print(" ay_0 = "); Serial.print((int)(1000.0f*accData[0][1]));
    Serial.print(" az_0 = "); Serial.print((int)(1000.0f*accData[0][2])); Serial.println(" mg");

    Serial.println("");
    Serial.print("Raw Accel Reading: For a valid Calib there are ");
    Serial.print(calibratingA[0]);
    Serial.println(" raw readings left. Please wait...");
    Serial.println("");

    Serial.print("gx_0 = "); Serial.print(gyroData[0][0], 2); Serial.print(" gy_0 = "); Serial.print(gyroData[0][1], 2); 
    Serial.print(" gz_0 = "); Serial.print(gyroData[0][2], 2); Serial.println(" deg/s"); Serial.println("");
    Serial.print("mx_0 = "); Serial.print((int)magData[0][0]); Serial.print(" my_0 = "); Serial.print((int)magData[0][1]);
    Serial.print(" mz_0 = "); Serial.print((int)magData[0][2]); Serial.println(" uT"); Serial.println("");
    Serial.println("Sentral_0 Quaternion (NED):"); Serial.print("Q0_0 = "); Serial.print(qt[0][0]);
    Serial.print(" Qx_0 = "); Serial.print(qt[0][1]); Serial.print(" Qy_0 = "); Serial.print(qt[0][2]); 
    Serial.print(" Qz_0 = "); Serial.print(qt[0][3]); Serial.println(""); Serial.println("");

    //Euler angles
    Serial.print("Sentral_0 Yaw, Pitch, Roll: ");
    Serial.print(heading[0], 2); Serial.print(", "); Serial.print(angle[0][1], 2); Serial.print(", "); Serial.println(angle[0][0], 2);
    Serial.println("");

    //Temperature and pressure
    Serial.print("Baro Pressure: "); Serial.print(pressure[0], 2); Serial.print(" mbar"); Serial.println("");
    Serial.print("Baro Temperature: "); Serial.print(temperature[0], 2); Serial.print(" deg C"); Serial.println("");
    Serial.println("");

  #

    // if (delt_t_2 > 5000)  {// Hotkey messaging
      if(calibratingA[0] < 1)
      { 
        Serial.println("Send '3' to enter the Calibration loop");
        Serial.println("");
        Serial.println("Send '4' to abort the Calibration loop. In this case you have to start the calibration later again.");
        Serial.println("");


        Serial.println("Calibration:");
        Serial.println("All three accelerometers must to be calibrated in the +/-1g condition in the following order x,y,z");
        Serial.println("Make sure the desired accelerometer axis is properly aligned with gravity and remains still");
        Serial.println("");
        Serial.println("Send '1' to collect and average the Raw Accel Readings   ");
        Serial.println("If you have collected all +/+1g calibration readings for all axes, you have to power-cycle the system. After the restart the new calibrtion is stored to the EEPROM. You can see it in the serial monitor after the start up.");
        Serial.println("");

  
      }
      Serial.println("Send '2' to save Sentral_0 Warm Start params after the Algorithm status is 8. But the accel calib as to be done before this.");
      Serial.println("");


       Serial.println("###############################################################");



      delay(delay_loop);

  }


    
// #ifdef SERIAL_DEBUG



  // }
//#endif

// Spreadsheet output when "SERIAL_DEBUG" is not defined in config.h
// #ifndef SERIAL_DEBUG
//   Serial.print(TimeStamp, 2);              Serial.print(","); Serial.print(qt[0][0], 2);         Serial.print(","); Serial.print(qt[0][1], 2);         
//   Serial.print(",");                       Serial.print(qt[0][2], 2);  Serial.print(",");        Serial.print(qt[0][3], 2); Serial.print(",");
//   Serial.print(heading[0], 2);             Serial.print(","); Serial.print(angle[0][1], 2);      Serial.print(","); Serial.print(angle[0][0], 2);
//   Serial.println("");
// #endif

// Toggle LED unless calibrating accelerometers
// if(calibratingA[0] < 1)
// {
//   Alarms::toggle_IndLED();
// } else
// {
//   Alarms::IndLEDoff();
// }




  if ((GPS.fix && calib_start==false)  || (actualtime>600000 && calib_start==false) ) {


      if (GPS.fix){
        // Serial.print("\nTime: ");
        // if (GPS.hour < 10) { Serial.print('0'); }
        // Serial.print(GPS.hour, DEC); Serial.print(':');
        // if (GPS.minute < 10) { Serial.print('0'); }
        // Serial.print(GPS.minute, DEC); Serial.print(':');
        // if (GPS.seconds < 10) { Serial.print('0'); }
        // Serial.print(GPS.seconds, DEC); Serial.print('.');
        // if (GPS.milliseconds < 10) {
        //   Serial.print("00");
        // } 
        // else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
        //   Serial.print("0");
        // }
        // Serial.println(GPS.milliseconds);
        // Serial.print("Date: ");
        // Serial.print(GPS.day, DEC); Serial.print('/');
        // Serial.print(GPS.month, DEC); Serial.print("/20");
        // Serial.println(GPS.year, DEC);
        // Serial.print("Fix: "); Serial.print((int)GPS.fix);
        // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

        //   Serial.print("Location: ");
        //   Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        //   Serial.print(", ");
        //   Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        //   Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        //   Serial.print("Angle: "); Serial.println(GPS.angle);
        //   Serial.print("Altitude: "); Serial.println(GPS.altitude);
        //   Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        //   Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      }
      
      else {


          ntp=true;
       
        
      }
 
        // Calculate loop cycle time
      currentTime = micros();
      cycleTime = currentTime - previousTime;
      previousTime = currentTime;
      // Serial.println(String(cycleTime));
      

   
      //See what data are available from the Sentral by polling the Status register
      if(drdy == 1)
      {
        drdy = 0;
        FetchEventStatus(i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly Sentral instance 0)
      }

      // Acquire data the Sentral
      FetchSentralData(Sentral_0, imu_0, 0);                                                                             // Sentral instance 0, IMU calculation instance 0 and Sensor instance 0
     
      // Update serial
      delt_t = millis() - last_refresh;
      
      // Serial.println("zähler:"+String(delt_t));
      // Serial.println("last refresh:"+String(last_refresh));
      // Serial.println("millis:"+String(millis()));


      // Serial.print("ax_0 = "); Serial.print((int)(1000.0f*accData[0][0])); Serial.print(" ay_0 = "); Serial.print((int)(1000.0f*accData[0][1]));

      if(delt_t>=50)                                                                                         // Update the serial monitor every "UPDATE_PERIOD" ms
      {


        Serial.println("Measurement is running");

        // Serial.println("test");

        last_refresh = millis();

        // Serial.println(String(String(1000.0f*accData[0][0])));

        String sample_name;
        String year;
        String month;
        String day;
        String hour;
        String minute;

        if (ntp) {


          ntp=true;
          time(&now);                       
          localtime_r(&now, &tm);           

               
          year=String(tm.tm_year + 1900);


          if ((tm.tm_mon+1)<10) 
          {
            month="0"+String(tm.tm_mon + 1);
          
          }
          else{month=String(tm.tm_mon + 1);
          
          
          }
          
            if ((tm.tm_mday)<10) 
          {
            day="0"+String(tm.tm_mday);
          
          }
          else{day=String(tm.tm_mday);
          
          
          }

       
         
          if ((tm.tm_hour)<10) 
          {
            hour="0"+String(tm.tm_hour);
          
          }
          else{hour=String(tm.tm_hour);
          
          
          }


          if ((tm.tm_min)<10) 
          {
            minute="0"+String(tm.tm_min);
          
          }
          else{minute=String(tm.tm_min);
          
          
          }

        }


        else {


          char c = GPS.read();


          // if you want to debug, this is a good time to do it!
          //Serial.println("GPSecho?:"+String(GPSECHO));
          if (GPSECHO)
            if (c) Serial.print(c);
          // if a sentence is received, we can check the checksum, parse it...
          if (GPS.newNMEAreceived()) {
            // a tricky thing here is if we print the NMEA sentence, or data
            // we end up not listening and catching other sentences!
            // so be very wary if using OUTPUT_ALLDATA and trying to print out data
            //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
          if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
              return; // we can fail to parse a sentence in which case we should just wait for another
            }

          // String year;

          year="20"+String(GPS.year);
          // String month;
          if(GPS.month<10){
            month="0"+String(GPS.month);
          }
          else {
            month=String(GPS.month);
        
          }
          
          // String day;
          if(GPS.day<10){
            day="0"+String(GPS.day);
          }
          else {
            day=String(GPS.day);
        
          }
          
          // String hour;
          

           if ((GPS.hour)<10) 
          {
            hour="0"+String(GPS.hour);
          
          }
          else{hour=String(GPS.hour);
          
          
          }

          if ((GPS.minute)<10) 
          {
            minute="0"+String(GPS.minute);
          
          }
          else{minute=String(GPS.minute);
          
          
          }
        
        }

    
        String dir_name;
        dir_name="/"+year;
        char dirchar[dir_name.length()+1];
        dir_name.toCharArray(dirchar, sizeof(dirchar));
        SD_MMC.mkdir(dirchar);
        //Serial.println(dir_name);
       

        
        dir_name="/"+year+"/"+month;
        char dirchar2[dir_name.length()+1];
        dir_name.toCharArray(dirchar2, sizeof(dirchar2));
        SD_MMC.mkdir(dirchar2);
        //Serial.println(dir_name);

    
        dir_name="/"+year+"/"+month+"/"+day;
        char dirchar3[dir_name.length()+1];
        dir_name.toCharArray(dirchar3, sizeof(dirchar3));
        SD_MMC.mkdir(dirchar3);
        //Serial.println(dir_name);

       
        dir_name="/"+year+"/"+month+"/"+day+"/"+hour;
        char dirchar4[dir_name.length()+1];
        dir_name.toCharArray(dirchar4, sizeof(dirchar4));
        SD_MMC.mkdir(dirchar4);
        //Serial.println(dir_name);


        sample_name=dir_name+"/"+year+month+day+hour+minute+".txt";
        File file = SD_MMC.open(sample_name, FILE_APPEND);
        

        if(ntp) {

           file.println(String(tm.tm_mday)+";"+
                    String(tm.tm_mon + 1)+";"+
                    String(tm.tm_year + 1900)+";"+
                    String(tm.tm_hour)+";"+
                    String(tm.tm_min)+";"+
                    String(tm.tm_sec)+";"+
                    "0"+";"+
                    String(GPS.latitude)+";"+
                    String(GPS.lat)+";"+
                    String(GPS.longitude)+";"+
                    String(GPS.lon)+";"+
                    String(GPS.speed)+";"+
                    String(GPS.altitude)+";"+
                    String(GPS.satellites)+";"+
                    String(GPS.fixquality)+";"+                    
                    String(1000.0f*accData[0][0])+";"+
                    String(1000.0f*accData[0][1])+";"+
                    String(1000.0f*accData[0][2])+";"+
                    String(gyroData[0][0])+";"+ 
                    String(gyroData[0][1])+";"+
                    String(gyroData[0][2])+";"+
                    String(magData[0][0])+";"+
                    String(magData[0][1])+";"+
                    String(magData[0][2])+";"+
                    String(qt[0][0])+";"+
                    String(qt[0][1])+";"+
                    String(qt[0][2])+";"+
                    String(qt[0][3])+";"+
                    String(heading[0])+";"+
                    String(angle[0][1])+";"+
                    String(angle[0][0])+";"+
                    String(temperature[0])+";"+
                    String(pressure[0]));
                    file.close();
       
        }

        else {
        

                      file.println(String(GPS.day)+";"+
                      String(GPS.month)+";"+
                      String(GPS.year)+";"+
                      String(GPS.hour)+";"+
                      String(GPS.minute)+";"+
                      String(GPS.seconds)+";"+
                      String(GPS.milliseconds)+";"+
                      String(GPS.latitude)+";"+
                      String(GPS.lat)+";"+
                      String(GPS.longitude)+";"+
                      String(GPS.lon)+";"+
                      String(GPS.speed)+";"+
                      String(GPS.altitude)+";"+
                      String(GPS.satellites)+";"+
                      String(GPS.fixquality)+";"+                    
                      String(1000.0f*accData[0][0])+";"+
                      String(1000.0f*accData[0][1])+";"+
                      String(1000.0f*accData[0][2])+";"+
                      String(gyroData[0][0])+";"+ 
                      String(gyroData[0][1])+";"+
                      String(gyroData[0][2])+";"+
                      String(magData[0][0])+";"+
                      String(magData[0][1])+";"+
                      String(magData[0][2])+";"+
                      String(qt[0][0])+";"+
                      String(qt[0][1])+";"+
                      String(qt[0][2])+";"+
                      String(qt[0][3])+";"+
                      String(heading[0])+";"+
                      String(angle[0][1])+";"+
                      String(angle[0][0])+";"+
                      String(temperature[0])+";"+
                      String(pressure[0]));
                      file.close();
      

        }


      }
  }

      // delay(200);
    
  
  
}

