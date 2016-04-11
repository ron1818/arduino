/* 2015/09/16
   Ren Ye

   Heading with calibration by push button
   When push button once, the chip is in calibration phase, LED 13 on
   when push button again, the chip is in heading phase, LED 13 off
   need to use software deboucing!

   Connectivity:
   5V -> 10Kohm -> [2] -> button pin 1
   button p2 -> gnd
*/


// library
#include <Wire.h>
#include <LSM303.h>

int LED = 13; // onboard led
int button = 2; // button to enter and exit calibration mode
volatile boolean iscalibrate = 0; // interrupt, calibrated or not

// for calibration report
char report[80];

// call the object compass
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

/*
boolean debounce(boolean last)
{
  boolean current = digitalRead(button);
  if (last != current)
  {
    delay(debounceDelay);
    current = digitalRead(button);
  }
  return current;
}
*/

void calibrate(){
  // toggle calibrate state, whenever the button is pressed
  if (iscalibrate==0)
    iscalibrate=1;
  else
    iscalibrate=0;
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(button,INPUT);

  digitalWrite(LED, LOW);
  
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  //compass.m_min = (LSM303::vector<int16_t>){-2838, -3539, -4191}; //obtained by calibration
  //compass.m_max = (LSM303::vector<int16_t>){+2133, +970, +534}; // obtained by calibration
  
  // interrupt, for UNO, only use pin 2 or 3
  attachInterrupt(digitalPinToInterrupt(button), calibrate, FALLING);
}

void loop() {
  // checking if it is in interrupting mode
  while(iscalibrate==1) //calibration
  {
    digitalWrite(LED,HIGH);
    
    compass.read();
  
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
    Serial.println(report);
    delay(100);
    compass.m_min=running_min;
    compass.m_max=running_max;
  }

   //normal heading mode
    digitalWrite(LED,LOW);
    
    compass.read();
    float heading = compass.heading();
  
    Serial.println(heading);
    delay(1000);
}
