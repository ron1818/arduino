/* 
Title:    Demo Sketch for Infineon DC Motor Control Shield
Author:   Frederick Vandenbosch
Version:  1.0
Date:     8 Feb 2015
*/

// Pin assignment for first half-bridge
const int IS_1 = A0; // current sense and diagnostics
const int IN_1 = 3; // input (PWM)
const int INH_1 = 12; // inhibit (low = sleep)

// Pin assignment for second half-bridge
const int IS_2 = A1; // current sense and diagnostics
const int IN_2 = 11; // input (PWM)
const int INH_2 = 13; // inhibit (low = sleep)

void setup() {
  // Set correct input/output state
  pinMode(IS_1, INPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(INH_1, OUTPUT);
  pinMode(IS_2, INPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(INH_2, OUTPUT);
  
  // Set initial output states
  analogWrite(IN_1, 0); // set motor speed to 0
  digitalWrite(INH_1, HIGH); // enable OUT1
  analogWrite(IN_2, 0); // set motor speed to 0
  digitalWrite(INH_2, HIGH); // enable OUT2
}

void loop() {
  /* 
  half bridge mode, two unidirectional motors
  motor 1 connected to OUT1 and GND
  motor 2 connected to OUT2 and GND
  */
  /*forward(127); // motor 1 and motor 2 at half speed
  delay(5000);
  
  forward(255); // motor 1 and motor 2 at full speed
  delay(5000);
  
  left(127); // motor 1 stopped, motor 2 at half speed
  delay(5000);
  
  left(255); // motor 1 stopped, motor 2 at half speed
  delay(5000);
  
  right(127); // motor 1 at half speed, motor 2 stopped
  delay(5000);
  
  right(255); // motor 1 at half speed, motor 2 stopped
  delay(5000);
  
  halt(); // motor 1 and motor 2 stopped
  delay(5000);
  */
  
  /* 
  h-bridge mode, single bidirectional motor
  motor connected to OUT1 and OUT2
  */
  /*left(127); // direction 1 at half speed
  delay(5000);
  
  left(255); // direction 1 at full speed
  delay(5000);
  
  right(127); // direction 2 at half speed
  delay(5000);
  
  right(255); // direction 2 at full speed
  delay(5000);
  
  halt(); // stop motor
  delay(5000);
  */
  for(int i=0; i<=255; i++){
    left(i);
    delay(100);
  }
  delay(1000);
  for(int i=255; i>=0; i--){
    left(i);
    delay(100);
  }
  delay(1000);
  for(int i=0; i<=255; i++){
    right(i);
    delay(100);
  }
  delay(1000);
  for(int i=255; i>=0; i--){
    right(i);
    delay(100);
  }

  halt();
  delay(1000);
  
  
}

void forward(int motorSpeed) { // motorSpeed from 0 to 255
  analogWrite(IN_1, motorSpeed);
  analogWrite(IN_2, motorSpeed);
}

void halt() {
  analogWrite(IN_1, 0);
  analogWrite(IN_2, 0);
}

void left(int motorSpeed) { // in case of two motors: assuming left wheel motor on OUT1 and right wheel motor on OUT2
  analogWrite(IN_1, 0);
  analogWrite(IN_2, motorSpeed);
}

void right(int motorSpeed) { // in case of two motors: assuming left wheel motor on OUT1 and right wheel motor on OUT2
  analogWrite(IN_1, motorSpeed);
  analogWrite(IN_2, 0);
}
