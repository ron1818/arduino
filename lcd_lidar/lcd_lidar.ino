/* Lidar Lite and LCD display on arduino Nano,
 * Lidar Lite and LCD both with I2C 
 * Lidar Lite: 
 * 1 -> 5V, 6 -> GND
 * 2 -> SDA (D4)
 * 3 -> SCL (D5)
 * LCD:
 * SDA -> SDA (D4)
 * SCL -> SCL (D5)
 * Vcc -> 5V GND -> GND
 */

#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal_I2C.h>

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x3f (27) for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

LIDARLite myLidarLite;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Used to type in characters
  myLidarLite.begin();
  myLidarLite.beginContinuous();
  lcd.begin(20,4);   // initialize the lcd for 20 chars 4 lines, turn on backlight

  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Distance (cm)");
  
  lcd.setCursor(0,2); //Start at character 4 on line 3
  lcd.print("Velocity (cm/s)");

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(myLidarLite.distance());
  Serial.println(myLidarLite.velocity());

  // when characters arrive over the serial port...
  //if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // clear the screen
    lcd.clear();
    lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Distance (cm)");
  
  lcd.setCursor(0,2); //Start at character 4 on line 3
  lcd.print("Velocity (cm/s)");
    // read all the available characters
    //while (Serial.available() > 0) {
      // display each character to the LCD
      lcd.setCursor(0,1);
      lcd.print(myLidarLite.distance());
      lcd.setCursor(0,3);
      lcd.print(myLidarLite.velocity());
      delay(1000);
    //}
  //}
}
