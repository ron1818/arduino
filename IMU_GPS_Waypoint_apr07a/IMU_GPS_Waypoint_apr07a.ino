/* ren ye 20160408
    use RTIMULib-Arduino to collect imu data
    use tinyGPS++ to collect GPS data
    use LiquidCrystal-I2C to display data
    https://arduino-info.wikispaces.com/LCD-Blue-I2C#v1
    https://bitbucket.org/fmalpartida/
    use haversine to calculate distance and bearing
    http://www.movable-type.co.uk/scripts/latlong.html
    TODO:
    waypoint infomation read from UART
    data transmission to UART
    combine PWM for rudder and motor control
    PID control
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "RTPressure.h"
#include "CalLib.h"
#include <EEPROM.h>

/***************** overall init ************************/
//  GPSbaud defines the speed to use for gps tx rx
#define GPSbaud 9600
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

/***************** IMU init ****************************/
RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  1000                         // interval between pose displays

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
/***************** IMU init end ************************/

/***************** LCD init ****************************/
// set the LCD address to 0x3f for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
/***************** LCD init end ************************/

/***************** GPS init ****************************/
// software serial
SoftwareSerial GPSSerial(13, 12); // Tx -> 13, Rx -> 12
// gps initialize
Adafruit_GPS GPS(&GPSSerial); //initialize GPS

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false // no need to get raw data
/*
  #define IMU_EN true // enable IMU output
  #define PRE_EN true // enable pressure output
  #define GPS_EN true //enable GPS output
*/

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
/***************** GPS init end ************************/


void setup()
{
  int errcode;
  /************ open serial ports ************/
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  /*******************************************/

  /************** LCD setup *****************/
  lcd.begin(20, 4); // 20 character by 4 lines
  // Quick blink of backlight
  lcd.clear(); // clear screen
  lcd.backlight();
  delay(250);
  lcd.noBacklight();
  delay(250);
  lcd.backlight(); // finish with backlight on

  // display welcome screen
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print("GPS IMU Pilot");
  lcd.setCursor(0, 1);
  lcd.print("Ren Ye, 2016");
  lcd.setCursor(0, 2);
  lcd.print("AdaFruit UltGPS V4");
  lcd.setCursor(0, 3);
  lcd.print("Pololu AltIMU V9");
  delay(1000);
  /*******************************************/

  /************** GPS setup ******************/
  GPS.begin(GPSbaud); // MTK use 9600, airmar and some other use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // output sentence configuration
  /* PMTK_SET_NMEA_OUTPUT_RMCGGA, PMTK_SET_NMEA_OUTPUT_ALLDATA */
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);   // 1 Hz position fit rate
  /* PMTK_SET_NMEA_UPDATE_1HZ, PMTK_SET_NMEA_UPDATE_5HZ, PMTK_SET_NMEA_UPDATE_10HZ */
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  delay(1000);
  // Ask for firmware version, comment for python
  // GPSSerial.println(PMTK_Q_RELEASE);
  /*******************************************/

  /************** IMU setup ******************/
  imu = RTIMU::createIMU(&settings);                 // create the imu object
  //delay(500);
  pressure = RTPressure::createPressure(&settings);  // create the pressure sensor
  //delay(500);
  if (pressure == 0) {
    Serial.println("No Baro sensor has been configured - terminating");
    while (1) ;
  }

  Serial.print("Starting using IMU "); Serial.print(imu->IMUName());
  Serial.print(", Baro "); Serial.println(pressure->pressureName());
  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if ((errcode = pressure->pressureInit()) < 0) {
    Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.

  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  /*******************************************/
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


// uint32_t timer = millis();
void loop()                     // run over and over again
{
  unsigned long now = millis();
  unsigned long delta;
  float latestPressure;
  float latestTemperature;
  int loopCount = 1;

  //Serial.println(imu->IMURead());
  while (imu->IMURead()) { // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)
      continue;

    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
//      Serial.print("Sample rate: "); Serial.print(sampleCount);
//      if (imu->IMUGyroBiasValid())
//        Serial.println(", gyro bias valid");
//      else
//        Serial.println(", calculating gyro bias");

      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      // RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
      RTMath::displayACCNMEA0183((RTVector3&)imu->getAccel());
      Serial.println();
      RTMath::displayRollPitchYawNMEA0183((RTVector3&)fusion.getFusionPose()); // fused output
      
//      if (pressure->pressureRead(latestPressure, latestTemperature)) {
//        Serial.print("pressure: "); Serial.print(latestPressure);
//        Serial.print(", temperature: "); Serial.println(latestTemperature);
//      }
      
      if (GPS.newNMEAreceived())
      {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
      }
    }
    // Serial.println();
  }
}





