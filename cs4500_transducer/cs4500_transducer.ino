/* reading airmar CS4500 ultrasonic speed sensor's output,
 * bare -> GND
 * green -> 13V peak, voltage divider (D5)
 * brown, white (yellow) -> temperature resistor (10K ohm @ 25c)
 * 
 * voltage divider:
 * 242K*2 div 242K, 12V peak becomes 4V peak, safe for arduino.
 * 
 * (1) detect the rising edges and count the milliseconds with 10 rising edges,
 * (2) detect the rising edges and count how many within 2 seconds, and convert back to hertz,
 * use option 2 for this project
 * then can get the Hertz
 * 
 * from datasheet:
 * Data Update Rate: 2 seconds
 * Signal Output: Airmar paddlewheel format
 * —5.6 Hz per knot
 * —20,000 pulses per nautical mile
 * Speed Range: 0.1 knot to 40 knots (0.1 MPH to 46 MPH)
 * 1 knot = 0.514 m/s
 * 0 Hz -> 0 m/s
 * 11 Hz -> ~1 m/s
 * 22 Hz -> ~2 m/s
 * etc.
 * 
 * Temperature sensor:
 * Operating Temperature Range: 0°C to 40°C (32°F to 104°F)
 * The resistance measurement on the temperature sensor 
 * (brown & white wires) should be 
 * 10,000 ohm at 25°C (77° F), 
 * 8,000 ohm at 30°C (86°F) 
 * and 20,000 ohm at 10°C (50° F)
 * mapping table:
 * c F ohm
 * 0 32  29,490
 * 5 41  23,460
 * 10  50  18,790
 * 15  59  15,130
 * 20  68  12,260
 * 25  77  10,000
 * 30  86  8,194
 * 35  95  6,752
 * 40  104 5,592
 * 
 * 5V -> 5Kohm--brown -> A0 to read data
 * white (yellow) -> GND
 * 
 * 2016-01-15
 * use AMEBA board to read the data and 
 * (1) display on 2004 LCD and 
 * (2) send over WIFI to Thingspeak
 * (3) save to sd card
 * 
 * additional pin: 
 * A4 (ameba SDA) -> LCD SDA
 * A5 (ameba SCL) -> LCD SCL
 * 5v -> VCC
 * GND -> GND
 */

// libraries
#include <Wire.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
// #include <SD.h>

// initialize LCD
// set the LCD address to 0x3f for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// wifi parameter
char ssid[] = "ERI@N_PI2";     //  your network SSID (name)
char pass[] = "ERI@Neirp";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

// pin definition
const int SpeedPin = 5;
const int TempPin = A0;
const int LEDPin = 13;

// Variables will change:
int RiseEdgeCounter = 0;   // counter for the number of rising edges
int SpeedState = 0;         // current state of the Speed sensor
int lastSpeedState = 0;     // previous state of the Speed sensor
int analog_read = 0;
double ADC_to_temperature(int);
double hertz_to_speed(double);
double mapf(double, double, double, double);

void setup() {
  // put your setup code here, to run once:
  // initialize the button pin as a input:
  pinMode(SpeedPin, INPUT);
  // initialize the LED as an output:
  pinMode(LEDPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);

  // initialize wifi
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  // initialize LCD
  //lcd.begin(20,4);   // initialize the lcd for 20 chars 4 lines, turn on backlight
  //lcd.setCursor(0,0); //Start at character 4 on line 0
  //lcd.write("V (m/s): ");
  //lcd.setCursor(0,2); //Start at character 4 on line 3
  //lcd.write("T (C): ");

  delay(1000);
}

unsigned long previous_timer=millis();

void loop() {
  // put your main code here, to run repeatedly:
  // read the pushbutton input pin:
  SpeedState = digitalRead(SpeedPin);
  //Serial.println(SpeedState);
  unsigned long current_timer=millis();
  
  // compare the buttonState to its previous state
  if (SpeedState != lastSpeedState) {
    // if the state has changed, increment the counter
    if (SpeedState == HIGH) {
      // if the current state is HIGH then the speed signal
      // wend from low to high:
      RiseEdgeCounter++;
    }
  }
  // save the current state as the last state,
  //for next time through the loop
  lastSpeedState = SpeedState;

  // update timer
  if (previous_timer > current_timer)
  {
    previous_timer = current_timer; //wrap millis()
  }

  if (current_timer - previous_timer >= 2000) // duration is 2 sec
  {
    Serial.print("Water Speed (m/s): ");
    Serial.println(hertz_to_speed(RiseEdgeCounter/2.0)); // convert to speed
    Serial.print("Water temprature (c): ");
    Serial.println(ADC_to_temperature(analogRead(TempPin))); // water temperature 
    RiseEdgeCounter = 0; // reset counter
    previous_timer = current_timer; // reset timer
    digitalWrite(LEDPin, HIGH);
  }

  delay(2); // max frequency is 225Hz (4.4ms)
  digitalWrite(LEDPin, LOW);
}

double hertz_to_speed(double hertz){
  double knot;
  
  knot = hertz/5.6;
  return 0.514 * knot;
}

double ADC_to_temperature(int ADC_data){
  double sense_volt;
  double thermistor;
  double temperature_reci;
  double v_ref = 5.0;
  double pullup = 1500.0;

  // sense_volt = map(double(ADC_data), 0.0, 1023.0, 0.0, v_ref);
  // map function fail to give double, use mapf
  sense_volt = mapf(double(ADC_data), 0.0, 1023.0, 0.0, v_ref);
  //sense_volt = double(ADC_data) / 1023.0 * v_ref;
  thermistor = (pullup * sense_volt) / (v_ref - sense_volt);
  //Serial.println(sense_volt);
  // calculated use R and table:
  // 1/T = 1.032e-3 + 2.386e-4 * log(R) + 1.587e-7 * (log(R)) ^ 3
  // T is in kelvin, so need to -273.15 to celsius
  temperature_reci = 1.032e-3 + 2.386e-4 * log(thermistor) + 1.587e-7 * pow(log(thermistor), 3);
  return 1.0/temperature_reci - 273.15;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}
