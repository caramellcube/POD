
/*
HMC5883L_Example.pde - Example sketch for integration with an HMC5883L triple axis magnetomerwe.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
/*************************************************** 
  This is an example for the TMP006 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit TMP006 Breakout 
  ----> https://www.adafruit.com/products/1296

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <DHT22.h>
#include <stdio.h>
#include <HMC5883L.h>
#include "Adafruit_SI1145.h"
#include "Adafruit_TMP006.h"
#include <Adafruit_ADXL345_U.h>

#define DHT22_PIN 7  // Lets the arduino know which pin the DHT-22 is connected to

int error = 0;
int CaCuSensorVal = 0;
int CaCuMax = 0;
int CaCuMin = 1024;
float CaCuX = 0;
float CaCuY = 0;
float CaCuZ = 0;
int CaCuSerialRead = 0;
int CaCuMode = 0;
long CaCuStartDHT = 0;
long CaCuTimeDHT = 0;
long CaCuStartTMP = 0;
long CaCuTimeTMP = 0;

DHT22 myDHT22(DHT22_PIN);
HMC5883L compass;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_TMP006 tmp006;

void setup(void) // this subroutine runs once when the arduino starts
{
 
  analogReference(EXTERNAL);  // don't forget this part, it stops the arduino from lighting on fire
  
  Serial.begin(9600); // Start the usb interface
  Wire.begin(); // Start the I2C interface.

  compass = HMC5883L(); // Construct a new HMC5883 compass.
  
  error = compass.SetScale(1.3); // Set the scale of the compass.
  //if(error != 0) // If there is an error, print it out.
  //  Serial.println(compass.GetErrorText(error));
  
  //Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  //if(error != 0) // If there is an error, print it out.
  //  Serial.println(compass.GetErrorText(error));
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }  
    if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
    if (! tmp006.begin()) {
    Serial.println("Didn't find tmp006");
    while (1);
  }

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  // I think this is to clear the ADC and give the digital sensors time to warm up
  delay(100);
  CaCuSensorVal = analogRead(A0);
  delay(100);
  CaCuSensorVal = analogRead(A1);
  delay(100);
  CaCuSensorVal = analogRead(A2);
  delay(100);
  CaCuSensorVal = analogRead(A3);
  delay(1600);
}

void CaCuTestDHT() {    
      Serial.print("DHT,");
      // Speed check starts, the DHT can only be read once every 2 seconds 
     CaCuTimeDHT = millis() - CaCuStartDHT;
     if (CaCuTimeDHT < 2010){
       Serial.print("?,?,");  // these keep the .csv file in the right order
     }
     else {
      // DHT code begins      
      DHT22_ERROR_t errorCode; 
      errorCode = myDHT22.readData();
      switch(errorCode)
      {
        case DHT_ERROR_NONE:
          Serial.print(myDHT22.getTemperatureC());
          Serial.print(",");
          Serial.print(myDHT22.getHumidity());
          Serial.print(",");
          break;
        case DHT_ERROR_CHECKSUM:
          Serial.print("check sum error,check sum error,");
          break;
        case DHT_BUS_HUNG:
          Serial.print("BUS Hung,BUS Hung,");
          break;
        case DHT_ERROR_NOT_PRESENT:
          Serial.print("Not Present,Not Present,");
          break;
        case DHT_ERROR_ACK_TOO_LONG:
          Serial.print("ACK time out,ACK time out,");
          break;
        case DHT_ERROR_SYNC_TIMEOUT:
          Serial.print("Sync Timeout,Sync Timeout,");
          break;
        case DHT_ERROR_DATA_TIMEOUT:
          Serial.print("Data Timeout,Data Timeout,");
          break;
        case DHT_ERROR_TOOQUICK:
          Serial.print("Polled to quick,Polled to quick,");
          break;
      }
     CaCuStartDHT = millis();
     } 
}

void CaCuTestMag() {
       // Magnetometer code begins
      MagnetometerRaw raw = compass.ReadRawAxis();
      MagnetometerScaled scaled = compass.ReadScaledAxis();
  
      // Calculate heading when the magnetometer is level, then correct for signs of axis.
      float heading = atan2(scaled.YAxis, scaled.XAxis);
      //float declinationAngle = 0.0457;
      //heading += declinationAngle;
  
      // Correct for when signs are reversed.
      if(heading < 0)
        heading += 2*PI;
    
      // Check for wrap due to addition of declination.
      if(heading > 2*PI)
        heading -= 2*PI;
   
      // Convert radians to degrees for readability.
      float headingDegrees = heading * 180/M_PI; 
      Serial.print("Mag,");
      Serial.print(raw.XAxis);
      Serial.print(",");   
      Serial.print(raw.YAxis);
      Serial.print(",");   
      Serial.print(raw.ZAxis);
      Serial.print(",");
   
      Serial.print(scaled.XAxis);
      Serial.print(",");   
      Serial.print(scaled.YAxis);
      Serial.print(",");   
      Serial.print(scaled.ZAxis);
      Serial.print(",");
   
      Serial.print(heading);
      Serial.print(",");
      Serial.print(headingDegrees);
      Serial.print(",");   
}

void CaCuTestBMP() {
       // bmp code begins 
      /* Get a new sensor event */ 
      sensors_event_t event;
      bmp.getEvent(&event);
 
      /* Display the results (barometric pressure is measure in hPa) */
      if (event.pressure)
      {
        /* Display atmospheric pressue in hPa */
        Serial.print("BMP,");
        Serial.print(event.pressure);
        Serial.print(",");

        /* First we get the current temperature from the BMP085 */
        float temperature;
        bmp.getTemperature(&temperature);
        Serial.print(temperature);
        Serial.print(",");

        /* Then convert the atmospheric pressure, SLP and temp to altitude    */
        /* Update this next line with the current SLP for better results      */
        float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
        Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                            event.pressure,
                                            temperature)); 
        Serial.print(",");
      }
      else
      {
        Serial.print("error, error, error,");
      }  
}

void CaCuTestSI1145() {
    // Si1145 sensor code begins
    Serial.print("Lux,");
    Serial.print(uv.readIR());
    Serial.print(",");
    Serial.print(uv.readVisible());
    Serial.print(",");
    Serial.print(uv.readUV());
    Serial.print(",");  
}

void CaCuTestTmp006() {
      Serial.print("Temp,");
      // Speed check starts, the TMP006 can only be read once every 4 seconds
     CaCuTimeTMP = millis() - CaCuStartTMP;
     if (CaCuTimeTMP < 4010){
       Serial.print("?,?,"); // These keep the .csv file in the right order
     }
     else {
       float objt = tmp006.readObjTempC();
       Serial.print(objt); Serial.print(",");
       float diet = tmp006.readDieTempC();
       Serial.print(diet); Serial.print(",");
       CaCuStartTMP = millis();
     } 
}

void CaCuTestAnalog() {
     // analog sensor code begins
     Serial.print("Analog,");
    // CaCuSensorVal = analogRead(A7);
    // Serial.print(CaCuSensorVal);
    // Serial.print(",");
    // CaCuSensorVal = analogRead(A6);
    // Serial.print(CaCuSensorVal);
    // Serial.print(",");    
    // Pins A4 & A5 are not to be used by analogRead, they handle I2C    
     CaCuSensorVal = analogRead(A3);
     Serial.print(CaCuSensorVal);
     Serial.print(",");
     delay(12);
     CaCuSensorVal = analogRead(A2);
     Serial.print(CaCuSensorVal);
     Serial.print(",");
     delay(12);
     CaCuSensorVal = analogRead(A1);
     Serial.print(CaCuSensorVal);
     Serial.print(",");
     delay(12);
}

void CaCuTestLongSnap() {
  // Some sensors need to take multiple measurements to work
  // This routine reads the long exposure sensors over a fixed time
  // This causes a delay in replying
  CaCuMax = 0;
  CaCuMin = 1024;
  CaCuX = 0;
  CaCuY = 0;
  CaCuZ = 0;
  Serial.print("Long,");  
  for (int x = 1; x < 71 ; x++) {
      sensors_event_t event; 
      accel.getEvent(&event);
      CaCuSensorVal = (event.acceleration.x);
      if (CaCuSensorVal > CaCuX){CaCuX = CaCuSensorVal;}
      CaCuSensorVal = (event.acceleration.y);
      if (CaCuSensorVal > CaCuY){CaCuY = CaCuSensorVal;}
      CaCuSensorVal = (event.acceleration.z);
      if (CaCuSensorVal > CaCuZ) {CaCuZ = CaCuSensorVal;}
      delay(5);
      CaCuSensorVal = analogRead(A0);
      if (CaCuSensorVal > CaCuMax) CaCuMax = CaCuSensorVal;
      if (CaCuSensorVal < CaCuMin) CaCuMin = CaCuSensorVal;
      delay(5);
  }
  Serial.print(CaCuX);  Serial.print(","); 
  Serial.print(CaCuY);  Serial.print(",");
  Serial.print(CaCuZ);  Serial.print(","); 
  CaCuSensorVal = (CaCuMax - CaCuMin);
  Serial.print(CaCuSensorVal);
  Serial.print(",");
  CaCuMax = 0;
  CaCuMin = 1024;
  CaCuX = 0;
  CaCuY = 0;
  CaCuZ = 0;
  delay(10);
}

void CaCuTestLongLog() {
  // This routine reads the long exposure sensors since the last time it was called.
  // between log readings, the sensors run in the background, collecting data.
  Serial.print("Long,");  
  Serial.print(CaCuX);  Serial.print(","); 
  Serial.print(CaCuY);  Serial.print(",");
  Serial.print(CaCuZ);  Serial.print(","); 
  CaCuSensorVal = (CaCuMax - CaCuMin);
  Serial.print(CaCuSensorVal);
  Serial.print(",");
  delay(10);
  CaCuMax = 0;
  CaCuMin = 1024;
  CaCuX = 0;
  CaCuY = 0;
  CaCuZ = 0;
}

void loop(void) 
{
  if (Serial.available() > 0) {
     CaCuSerialRead = Serial.read();
       if (CaCuSerialRead==('0')) { // stop reading long exposure sensors in the background 
           CaCuMode = 0;
           Serial.println("#");
       }     
       if (CaCuSerialRead==('1')) { // take a reading using snapshot mode, over a fixed interval
           CaCuMode = 0;
           Serial.print("s,");
           delay(10);
           CaCuTestMag();
           CaCuTestBMP();
           CaCuTestSI1145();
           CaCuTestAnalog();
           CaCuTestDHT();
           CaCuTestTmp006();
           CaCuTestLongSnap();
           Serial.println("");
       }
       if (CaCuSerialRead==('2')) { // take a reading in logging mode, start reading long exposure sensors in the background
           CaCuMode = 1;
           Serial.print("l,");
           delay(10);
           CaCuTestMag();
           CaCuTestBMP();
           CaCuTestSI1145();
           CaCuTestAnalog();
           CaCuTestDHT();
           CaCuTestTmp006();
           CaCuTestLongLog();       
           Serial.println("");
       }
  }
  if (CaCuMode > 0){ // if the long exposure sensors are to run in the background, then this block of code runs on each loop
      sensors_event_t event; 
      accel.getEvent(&event);
      CaCuSensorVal = (event.acceleration.x);
      if (CaCuSensorVal > CaCuX){CaCuX = CaCuSensorVal;}
      CaCuSensorVal = (event.acceleration.y);
      if (CaCuSensorVal > CaCuY){CaCuY = CaCuSensorVal;}
      CaCuSensorVal = (event.acceleration.z);
      if (CaCuSensorVal > CaCuZ) {CaCuZ = CaCuSensorVal;}
      delay(5);
      CaCuSensorVal = analogRead(A0);
      if (CaCuSensorVal > CaCuMax) CaCuMax = CaCuSensorVal;
      if (CaCuSensorVal < CaCuMin) CaCuMin = CaCuSensorVal;
  } 
delay(10);
}
