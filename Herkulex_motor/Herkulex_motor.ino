#include <Herkulex.h>

/*  default motor ID is 219 or 0xDB
 *  default baud for motor is 115200
 *  connect software serial port (Rx:10, Tx:11) to motor
 *  connect 5V, GND to TR324 power pins
 *  connect D2 to TR324 Servo PWM pin
 */


# define MOTORID 219 //motor ID - verify your ID !!!!
# define RX 10 // connect to motor TX
# define TX 11 // connect to motor RX
const int PWM_in=2; // pwm input
// const int NeutralPulse=1450; //neutral position
const int LeftPulse=900; //left most (-90 degree)
const int RightPluse=2000; //right most(+90 degree)
// const int NeutralDegree=0; //neutral degree
const int LeftDegree=-120; //left most (-90 degree)
const int RightDegree=120; //right most(+90 degree)
volatile double pulsewidth; // rise time of PWM signal
int degree;
int ledcolor;
void setup()  
{
  pinMode(PWM_in, INPUT);
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(9600);    // Open serial communications, monitor
  Serial.println("Begin");
  Herkulex.begin(115200,RX,TX); //open motor serial with rx=10 and tx=11 
  Herkulex.reboot(MOTORID); //reboot first motor
  delay(500); 
  Herkulex.initialize(); //initialize motors
  delay(200);  
}

void loop(){
  pulsewidth = pulseIn(PWM_in, HIGH);
  Serial.print("pulse duration: ");
  Serial.println(pulsewidth);
  degree=map(pulsewidth, LeftPulse, RightPluse, LeftDegree, RightDegree);
  degree=constrain(degree,LeftDegree,RightDegree);
  Serial.print("Move Angle (degree): ");
  Serial.println(degree);
  if(degree>0) 
    ledcolor=LED_GREEN;
  else if(degree==0) 
    ledcolor=LED_BLUE;
  else 
    ledcolor=LED_RED;
  Herkulex.moveOneAngle(MOTORID, degree, 300, ledcolor); //move motor with 300 speed 
  delay(300);
  Serial.print("Get servo Angle: ");
  Serial.println(Herkulex.getAngle(MOTORID));
  delay(10);
//  Serial.println("Move Angle: -100 degrees");
//  Herkulex.moveOneAngle(n, -100, 1000, LED_GREEN); //move motor with 300 speed  
//  delay(1200);
//  Serial.print("Get servo Angle:");
//  Serial.println(Herkulex.getAngle(n));
//  Serial.println("Move Angle: 100 degrees");
//  Herkulex.moveOneAngle(n, 100, 1000, LED_BLUE); //move motor with 300 speed  
//  delay(1200);
//  Serial.print("Get servo Angle:");
//  Serial.println(Herkulex.getAngle(n));
//  
}


