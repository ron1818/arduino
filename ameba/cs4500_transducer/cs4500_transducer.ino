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
#include <OneWire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
// #include <SD.h>

// initialize LCD
// set the LCD address to 0x3f for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

byte celsius[8] = //icon for temperature
{
  0b01000,
  0b10100,
  0b10100,
  0b01011,
  0b00100,
  0b00100,
  0b00100,
  0b00011
};

byte offline[8]=
{
  0b10001,
  0b01010,
  0b01010,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b00000,
};

byte online[8]=
{
 0b11101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10111,
};
// wifi parameter
char ssid[] = "ERI@N_PI2";     //  your network SSID (name)
char pass[] = "ERI@Neirp";  // your network password
//int keyIndex = 0;             // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
//WiFiServer server(80);

// Initialize Arduino Ethernet Client
WiFiClient client;

// thingspeak parameter
char thingSpeakAddress[] = "api.thingspeak.com"; // channel 78969
String APIKey = "84HQSYGEMQXUZI64";              //enter your channel's Write API Key
const int updateThingSpeakInterval = 20 * 1000;  // 20 second interval at which to update ThingSpeak

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

unsigned long lastConnectionTime = 0;
boolean lastConnected = false;

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
    delay(15000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printWifiStatus();

  // initialize LCD
  lcd.begin(20,4);   // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.createChar(1, celsius); // 1 for celsius
  lcd.createChar(2, online); // 2 for online
  lcd.createChar(3, offline); // 3 for offline

  delay(1000);
}

unsigned long previous_timer=millis();

void loop() {
  String Wspd = "0.1";
  String Wtemp = "26.0";
  
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
    double water_speed = hertz_to_speed(RiseEdgeCounter/2.0);
    Wspd = String(water_speed, DEC);
    double water_temperature = ADC_to_temperature(analogRead(TempPin));
    Wtemp = String(water_temperature, DEC);
    // to serial monitor
    //Serial.print("Water Speed (m/s): ");
    //Serial.println(water_speed); // convert to speed
    //Serial.print("Water temprature (c): ");
    //Serial.println(water_temperature); // water temperature 
    // to lcd
    lcd.clear();
    lcd.setCursor(0,0); //Start at character 4 on line 0
    lcd.write("V (m/s): ");
    lcd.setCursor(0,2); //Start at character 4 on line 3
    lcd.write("T (C): ");
    lcd.setCursor(0,1);
    lcd.print(water_speed);
    lcd.setCursor(0,3);
    lcd.print(water_temperature);
      
    RiseEdgeCounter = 0; // reset counter
    previous_timer = current_timer; // reset timer
    digitalWrite(LEDPin, HIGH);
  }

  delay(2); // max frequency is 225Hz (4.4ms)
  digitalWrite(LEDPin, LOW);

    // Print Update Response to Serial Monitor
    if (client.available()) {
      char c = client.read();
      Serial.print(c);
      client.stop();
      lastConnected = client.connected();
    }
  
  // Disconnect from ThingSpeak
  if (!client.connected() && lastConnected) {
    Serial.println("...disconnected");
    Serial.println();
    client.stop();
  }
  // Update ThingSpeak
  if (!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval)) {
    updateThingSpeak("field1=" + Wspd + "&field2=" + Wtemp);
    Serial.println(Wspd);
    Serial.println(Wtemp);
  }
  //delay(2);
}

double hertz_to_speed(double hertz){
  return 0.514 * hertz/5.6; // 5.6 Hz/Knot, then convert to m/s
}

double ADC_to_temperature(int ADC_data){
  /* calculated use R and table:
   * 1/T = 1.032e-3 + 2.386e-4 * log(R) + 1.587e-7 * (log(R)) ^ 3
   * T is in kelvin, so need to -273.15 to celsius */
  double sense_volt;
  double thermistor;
  double temperature_reci;
  double v_ref = 5.0;
  double pullup = 1500.0;

  // map function fail to give double, use mapf
  sense_volt = mapf(double(ADC_data), 0.0, 1023.0, 0.0, v_ref);
  thermistor = (pullup * sense_volt) / (v_ref - sense_volt);
  temperature_reci = 1.032e-3 + 2.386e-4 * log(thermistor) + 1.587e-7 * pow(log(thermistor), 3);
  return 1.0/temperature_reci - 273.15;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();

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

void updateThingSpeak(String tsData) {
  if (client.connect(thingSpeakAddress, 80)) {
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + APIKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");
    client.print(tsData);
    lastConnectionTime = millis();

    if (client.connected()) {
      Serial.println("Connecting to ThingSpeak...");
      Serial.println();

    }
  }
}
