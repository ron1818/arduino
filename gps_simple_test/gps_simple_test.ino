#include <SoftwareSerial.h>
String readString;
//#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
//#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
//#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
//#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
//#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
//#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
//#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
//#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
//#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
//#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

//#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
//#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
//#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
//#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
//#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
//#define LOCUS_OVERLAP 0
//#define LOCUS_FULLSTOP 1
//
//#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
//#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
//
//// standby command & boot successful message
//#define PMTK_STANDBY "$PMTK161,0*28"
//#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
//#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
// #define PMTK_Q_RELEASE "$PMTK605*31"
//
//// request for updates on antenna status
//#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
//#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

void GPSsetup(void);
void GPSread(void);
SoftwareSerial GPSSerial(13, 12); // Tx -> 13, Rx -> 12
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  GPSSerial.begin(9600);
  GPSsetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  GPSread();
}

void GPSsetup(){
  GPSSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  GPSSerial.println(PMTK_API_SET_FIX_CTL_1HZ);
  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

void GPSread(){
  while (GPSSerial.available()) {
    delay(3);  //delay to allow buffer to fill
    if (GPSSerial.available() > 0) {
      char c = GPSSerial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    }
  }
  if (readString.length() > 0) {
    Serial.print(readString); //see what was received
    readString = "";
  }
}

