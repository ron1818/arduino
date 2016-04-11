/* Author: Ren Ye
 * Organization: ERI@N

 * This project is to pass NMEA sentences to the computer and later use python to
 * parse it.
 * The sentences includes but is not limited to GPS, motion, weather data

 * Hardware:
 * Pololu AltIMU V4 (motion, temperature and baro)
 * Adafruit Ultimate GPS (NMEA 0183 ready)
 * SparkFun Redboard (Arduino Uno R3 equivalent)

 * Arduino Libraries:
 * Wire: for I2C
 * SoftwareSerial: for serial communication (GPS)
 * LSM303: acceleration and magneto
 * LPS: temperature and baro
 * L3G: gyro
 * AdaFruitGPS: gps sentence parse (optional)

 * Connectivity:
 * SDA -> SDA
 * SCL -> SCL
 * 5V  -> Vin
 * GND -> GND
 * Tx -> P10 
 * Rx -> P9
 */


/* changelog: 2015-09-28
 * created file
 */


// require libraries
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>
#include <LPS.h>
#include <L3G.h>


// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
int GRAVITY=256;  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
float Gyro_Gain_X=0.07; //X axis Gyro gain
float Gyro_Gain_Y=0.07; //Y axis Gyro gain
float Gyro_Gain_Z=0.07; //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
// These numbers can be updated by the calibration function and
// interruption
int M_X_MIN=-2838;
int M_Y_MIN=-3539;
int M_Z_MIN=-4191;
int M_X_MAX=2133;
int M_Y_MAX=970;
int M_Z_MAX=534;

LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

float Kp_ROLLPITCH=0.02;
float Ki_ROLLPITCH=0.00002;
float Kp_YAW=1.2;
float Ki_YAW=0.00002;

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //1: Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //1: Will print the analog raw data
#define PRINT_EULER 1   //1: Will print the Euler angles Roll, Pitch and Yaw

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

const int GPS_interval=2000;
unsigned long previous_GPS_timer=0;   //general purpuse timer
//unsigned long GPS_timer24=0; //Second timer used to print values 

const int IMU_interval=500;
unsigned long previous_IMU_timer=0;   //general purpuse timer
unsigned long IMU_timer24=0; //Second timer used to print values 
unsigned long IMU_timer_old; //, GPS_timer_old;

int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;
int magnetom_x, magnetom_y, magnetom_z;
float c_magnetom_x, c_magnetom_y, c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll, pitch, yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  { 1,0,0  },
  { 0,1,0  },
  { 0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  { 0,0,0  },
  { 0,0,0  },
  { 0,0,0  }
};

// assign pin, no need for sda and scl
int GPS_Tx_Pin=9;
int GPS_Rx_Pin=8;
int statusLED=13; // onboard LED
int calibratePin=3; // calibration pin
volatile int iscalibrate=0; //status

int GPS_baud=9600; 
int Arduino_baud=9600;

// define objects
LPS ps;
L3G gyro;
LSM303 compass;

// software serial, GPS
SoftwareSerial mySerial(GPS_Tx_Pin, GPS_Rx_Pin); // define software serial pin map
Adafruit_GPS GPS(&mySerial); //initialize GPS

// echo gps raw data to the serial output
#define GPSECHO true

void setup() {
  // assign IO
  pinMode(statusLED, OUTPUT);
  pinMode(calibratePin,INPUT);  
  
  // initialize all objects
  // begin with predifined baud rate
  Serial.begin(Arduino_baud);
  // start logging, message
  Serial.println("Start Logging!");

  /******************* GPS ******************/
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(GPS_baud);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version, comment for python
  // mySerial.println(PMTK_Q_RELEASE);

  /****************** IMU ******************/
  digitalWrite(statusLED,HIGH);
  delay(1500);
  // begin I2C
  Wire.begin(); 
  // begin LPS
  Ps_Init(); 
  // begin LSM303
  Compass_Init();
  // begin L3G
  Gyro_Init(); 

  delay(20);

  // initialize compass range
  compass.m_min = (LSM303::vector<int16_t>){M_X_MIN, M_Y_MIN, M_Z_MIN}; //obtained by calibration
  compass.m_max = (LSM303::vector<int16_t>){M_X_MAX, M_Y_MAX, M_Z_MAX}; //obtained by calibration
  
  
  // offset AN, not for built-in function
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  // Serial.println("Offset:");
  // for(int y=0; y<6; y++)
  // Serial.println(AN_OFFSET[y]);
  
  delay(2000);
  digitalWrite(statusLED,LOW); // done, turn off LED
    
  previous_IMU_timer=millis();
  //previous_GPS_timer=millis();
  delay(20);
  counter=0;

  // interrupt calibrate compass
  attachInterrupt(digitalPinToInterrupt(calibratePin), Compass_Calibrate_Interrupt, FALLING);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  /******* LSM303 compass calibration ************/
  while(iscalibrate==1) //calibration phase
  {
    digitalWrite(statusLED,HIGH);
    Compass_Calibrate();
  }
  // calibration finished or not done
  digitalWrite(statusLED,LOW);
  float compass_heading=Compass_Heading_builtin();

  // timer initialization

  unsigned long current_GPS_timer=millis();
  unsigned long current_IMU_timer=millis();

  /********* GPS raw output ********/
  GPS.read();
  // if a sentence is received, we can check the checksum, parse it...

//
//
//  // if millis() or timer wraps around, we'll just reset it
//  if (previous_GPS_timer > current_GPS_timer)  previous_GPS_timer = current_GPS_timer;
//
//  // approximately every 2 seconds or so, print out the current stats
//  if (current_GPS_timer - previous_GPS_timer > GPS_interval) 
//    previous_GPS_timer = current_GPS_timer; // reset the timer

  /******** IMU ***************/
  if(previous_IMU_timer>current_IMU_timer)  previous_IMU_timer=current_IMU_timer; // millis() wraps around, reset
  if(previous_GPS_timer>current_GPS_timer)  previous_GPS_timer=current_GPS_timer; // millis() wraps around, reset


  if((current_IMU_timer-previous_IMU_timer)>20)  // Main loop runs at 1/IMU_interval
  {
    counter++;
    IMU_timer_old = previous_IMU_timer;
    if (current_IMU_timer>IMU_timer_old)
      G_Dt = (current_IMU_timer-IMU_timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    previous_IMU_timer=current_IMU_timer;
    

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    // Serial.println(counter); // for debug
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      	counter=0;
      	Read_Compass();    // Read I2C magnetometer
      	Compass_Heading(); // Calculate magnetic heading  
      }
    // check if it becomes nan
//    if(isnan(Omega_Vector[0])){
//      Serial.println("Omega becomes nan");
//      Omega_P[0]=0;
//      Omega_P[1]=0;
//      Omega_P[2]=0;
//      Omega_I[0]=0;
//      Omega_I[1]=0;
//      Omega_I[2]=0;
//      Omega[0]=0;
//      Omega[1]=0;
//      Omega[2]=0;
//    }
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
  }

  if((current_GPS_timer-previous_GPS_timer)>GPS_interval)  // Main loop runs at 1/IMU_interval
  {
//    Serial.println(counter);
//    Serial.print(Update_Matrix[0][0]);
//    Serial.print(Update_Matrix[0][1]);
//    Serial.println(Update_Matrix[0][2]);
//    Serial.print(Update_Matrix[1][0]);
//    Serial.print(Update_Matrix[1][1]);
//    Serial.println(Update_Matrix[1][2]);
//    Serial.print(Update_Matrix[2][0]);
//    Serial.print(Update_Matrix[2][1]);
//    Serial.println(Update_Matrix[2][2]);
//    Serial.print(Omega_Vector[0]);
//    Serial.print(Omega_Vector[1]);
//    Serial.println(Omega_Vector[2]);
    previous_GPS_timer=current_GPS_timer;
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences! 
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    }
    
    // printdata();
    print_pitch_roll_NMEA();
    print_accelerator_NMEA();
    print_heading_NMEA(compass_heading);
    print_LPS_NMEA();
  }
  
}
