/* 2015/09/21
   Ren Ye

   Use sparkfun's circuit 6 and circuit 7 to get solar insolation and temperature
   display on serial port
   Use Pololu's AltIMU V4 to get temperature and barometer
*/

#include <Wire.h>
#include <LPS.h>

LPS ps;

const int photoPin = 1;
const int temperaturePin = 0;

void setup() {
  // start serial
  Serial.begin(9600);
  Wire.begin();
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();

  //Serial.println("Tvoltage,DegreesC,Pvoltage,IMUtemperature,IMUpressure");

}

void loop() {
  // for temperature and photo resistor, analog
  float Tvoltage, Pvoltage, degreesC, degreesF;

  Tvoltage = getVoltage(temperaturePin);
  Pvoltage = getVoltage(photoPin);
  
  // Now we'll convert the voltage to degrees Celsius.
  // This formula comes from the temperature sensor datasheet:

  degreesC = (Tvoltage - 0.5) * 100.0;
  
  // While we're at it, let's convert degrees Celsius to Fahrenheit.
  // This is the classic C to F conversion formula:
  
  degreesF = degreesC * (9.0/5.0) + 32.0;

  // for temeperature and pressure with IMU and I2C
  float IMUpressure = ps.readPressureMillibars();
  //float altitude = ps.pressureToAltitudeMeters(pressure);
  float IMUtemperature = ps.readTemperatureC();

  
  // print to serial
/*
  Serial.print("T(V/C/F): ");
  Serial.print(Tvoltage);
  Serial.print(", ");
  Serial.print(degreesC);
  Serial.print(", ");
  Serial.print(degreesF);
  Serial.print(", ");
  Serial.print("P(V): ");
  Serial.print(Pvoltage);
  Serial.print(", ");
  Serial.print("Timu(C): ");
  Serial.print(", ");
  Serial.print(IMUtemperature);
  Serial.print(", ");
  Serial.print("Pimu(hPa) ");
  Serial.print(IMUpressure);
  Serial.print("\n");
*/

Serial.print(Tvoltage);
Serial.print(',');
Serial.print(degreesC);
Serial.print(',');
Serial.print(Pvoltage);
Serial.print(',');
Serial.print(IMUtemperature);
Serial.print(',');
Serial.println(IMUpressure);

  delay(1000);
  
}


float getVoltage(int pin)
{
  // This function has one input parameter, the analog pin number
  // to read. You might notice that this function does not have
  // "void" in front of it; this is because it returns a floating-
  // point value, which is the true voltage on that pin (0 to 5V).
  
  // You can write your own functions that take in parameters
  // and return values. Here's how:
  
    // To take in parameters, put their type and name in the
    // parenthesis after the function name (see above). You can
    // have multiple parameters, separated with commas.
    
    // To return a value, put the type BEFORE the function name
    // (see "float", above), and use a return() statement in your code
    // to actually return the value (see below).
  
    // If you don't need to get any parameters, you can just put
    // "()" after the function name.
  
    // If you don't need to return a value, just write "void" before
    // the function name.

  // Here's the return statement for this function. We're doing
  // all the math we need to do within this statement:
  
  return (analogRead(pin) * 0.004882814);
  
  // This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.
}

