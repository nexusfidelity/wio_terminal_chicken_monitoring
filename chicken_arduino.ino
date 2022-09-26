#include <Arduino.h>
#include <Wire.h>

//initialize temp sensor
#include <SensirionI2CSht4x.h>
SensirionI2CSht4x sht4x;
float temperature, humidity;

//initialize TFT LCD
#include "TFT_eSPI.h" //include TFT LCD library 
#include "Free_Fonts.h" //include free fonts library 
TFT_eSPI tft;
#define LCD_BACKLIGHT (72Ul) // Control Pin of LCD
int lcd_status = 1;

//ultrasonic variables
#define trig_water D0 //Button to Grove UART Port
#define echo_water D1 //Button to Grove UART Port
long duration, cm, inches, ultrasonic_val;
float gauge, new_gauge, gauge_gap;

//IO
#define servo_left D2 
#define servo_right D3
//#define servo_back D4 //Button to Grove UART Port
#define fan_switch D5 //Button to Grove UART Port

//servo
#include <Servo.h>
Servo left_servo;
Servo right_servo;
Servo back_servo;

//calibrations
int pointer = 0;
int updownpointer = 0;
int pointer_refresher;

//initialize wifi time
#include <rpcWiFi.h>
#include <millisDelay.h>
#include <Wire.h>
#include "RTC_SAMD51.h"
#include "DateTime.h"
// wifi credentials
const char ssid[] = "XXXXXX";
const char password[] = "XXXXXX";
//wifi time variables
millisDelay updateDelay; // the update delay object. used for ntp periodic update.
unsigned int localPort = 2390;      // local port to listen for UDP packets
// switch between local and remote time servers
// comment out to use remote server
//#define USELOCALNTP
#ifdef USELOCALNTP
char timeServer[] = "n.n.n.n"; // local NTP server
#else
char timeServer[] = "time.nist.gov"; // extenral NTP server e.g. time.nist.gov
#endif
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// declare a time object
DateTime now;
// define WiFI client
WiFiClient client;
//The udp library class
WiFiUDP udp;
// localtime
unsigned long devicetime;
RTC_SAMD51 rtc;
// for use by the Adafuit RTClib library
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };


void setup() {

  //serial setup
  Serial.begin(115200);
  Wire.begin();

  //temp sensor
  uint16_t error;
  char errorMessage[256];
  sht4x.begin(Wire);
  uint32_t serialNumber;
  error = sht4x.serialNumber(serialNumber);
  if (error) {
    Serial.print("Error trying to execute serialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("Serial Number: ");
    Serial.println(serialNumber);
  }

  //button
  pinMode(WIO_KEY_C, INPUT_PULLUP);
  pinMode(WIO_5S_UP, INPUT_PULLUP);
  pinMode(WIO_5S_DOWN, INPUT_PULLUP);
  pinMode(WIO_5S_LEFT, INPUT_PULLUP);
  pinMode(WIO_5S_RIGHT, INPUT_PULLUP);

  //IO
  pinMode(trig_water, OUTPUT);
  pinMode(echo_water, INPUT);

  pinMode(fan_switch, OUTPUT);
  
  //servo
  left_servo.attach(servo_left);
  right_servo.attach(servo_right);
//  back_servo.attach(servo_back);

  //time
  // setup network before rtc check
  connectToWiFi(ssid, password);
  // get the time via NTP (udp) call to time server
  // getNTPtime returns epoch UTC time adjusted for timezone but not daylight savings
  // time
  devicetime = getNTPtime();
  // check if rtc present
  if (devicetime == 0) {
    Serial.println("Failed to get time from network time server.");
  }
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1) delay(10); // stop operating
  }
  // get and print the current rtc time
  //now = rtc.now();
  //Serial.print("RTC time is: ");
  //Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  rtc.adjust(DateTime(devicetime));
  //now = rtc.now();
  //Serial.print("Adjusted RTC (boot) time is: ");
  //Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  // start millisdelays timers as required, adjust to suit requirements
  updateDelay.start(6 * 60 * 60 * 1000); // update time via ntp every 12 hrs

  //display LCD
  setup_LCD();
}

void loop() {

  //temp_sensor
  uint16_t error;
  char errorMessage[256];
  error = sht4x.measureHighPrecision(temperature, humidity);
  if (error) {
    //Serial.print("Error trying to execute measureHighPrecision(): ");
    //errorToString(error, errorMessage, 256);
    //Serial.println(errorMessage);
  } else {
    //    Serial.println("Temperature:");
    //    Serial.print(temperature);
    //    Serial.println("Humidity:");
    //    Serial.print(humidity);
  }
  fan(temperature);
  
  //update time via wifi by 6 hours
  if (updateDelay.justFinished()) { // 6 hour loop
    // repeat timer
    updateDelay.repeat(); // repeat
    // update rtc time
    devicetime = getNTPtime();
    if (devicetime == 0) {
      Serial.println("Failed to get time from network time server.");
    }
    else {
      rtc.adjust(DateTime(devicetime));
      Serial.println("");
      Serial.println("rtc time updated.");
      // get and print the adjusted rtc time
      now = rtc.now();
      Serial.print("Adjusted RTC time is: ");
      Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
    }
  }
  //buttons
  if (digitalRead(WIO_KEY_C) == LOW) {
    if (lcd_status == 1) {
      digitalWrite(LCD_BACKLIGHT, LOW);
      lcd_status = 0;
    }
    else if (lcd_status == 0) {
      digitalWrite(LCD_BACKLIGHT, HIGH);
      lcd_status = 1;
    }
  }
  else if (digitalRead(WIO_5S_LEFT) == LOW) {
    pointer = pointer - 1;
    if (pointer < 0) {
      pointer = 4;
    }
  }
  else if (digitalRead(WIO_5S_RIGHT) == LOW) {
    pointer = pointer + 1;
    if (pointer > 4) {
      pointer = 0;
    }
  }
  else if (digitalRead(WIO_5S_UP) == LOW) {
    updownpointer = 1;
  }
  else if (digitalRead(WIO_5S_DOWN) == LOW) {
    updownpointer = 2;
  }
  
  delay(200);
  //servo functions
  check_calibration(pointer);
  drive_servo(pointer, updownpointer);
  
  delay(1000);
  left_servo.write(90);
  right_servo.write(90);
  
  updownpointer = 0;

  //ultrasonic
  ultrasonic();
  
  draw_temp(temperature, humidity);
  draw_time();
}

void fan(float temperature) {
  if (int(temperature) > 30) {
    digitalWrite(fan_switch, HIGH);
  }
  else{
    digitalWrite(fan_switch, LOW);
  }
}

void check_calibration(int pointer) {
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  if (pointer_refresher != pointer) {
    tft.fillRect(165, 70, 150, 20, TFT_WHITE); //A 25x100 black rectangle starting from (110, 70)
  }
  pointer_refresher = pointer;
  if (pointer == 0) {
    tft.drawString("<NONE>", 165, 70); //draw text string
  }
  else if (pointer == 1) {
    tft.drawString("Blinds <All>", 165, 70); //draw text string
  }
  else if (pointer == 2) {
    tft.drawString("Blinds <Left>", 165, 70); //draw text string
  }
  else if (pointer == 3) {
    tft.drawString("Blinds <Right>", 165, 70); //draw text string
  }
  else if (pointer == 4) {
    tft.drawString("Blinds <Back>", 165, 70); //draw text string
  }
}

void drive_servo(int pointer, int updownpointer) {
if ((pointer == 1) && (updownpointer == 1)){
    }
  if ((pointer == 1) && (updownpointer == 2)){
    }
  if ((pointer == 2) && (updownpointer == 1)) {
    left_servo.write(45);
  }
  if ((pointer == 2) && (updownpointer == 2)) {
    left_servo.write(135);
  }
  if ((pointer == 3) && (updownpointer == 1)) {
    right_servo.write(45);
  }
  if ((pointer == 3) && (updownpointer == 2)) {
    right_servo.write(135);
  }
  if ((pointer == 4) && (updownpointer == 1)) {
    
  }
  if ((pointer == 4) && (updownpointer == 2)) {
    
  }
}

void setup_LCD() {
  tft.begin(); //start TFT LCD
  tft.setRotation(3); //set screen rotation
  tft.fillScreen(TFT_WHITE); //fill background
  tft.drawLine(0, 120, 320, 120, TFT_BLACK); //drawing a horizontal black line from (0,120) to (240,160)
  tft.drawLine(160, 0, 160, 240, TFT_BLACK); //drawing a vertical black line
  //setup text
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("temperature", 10, 10); //draw text string
  tft.drawString("humidity", 10, 50); //draw text string
  tft.drawString("time", 165, 10); //draw text string
  tft.drawString("functions", 165, 50); //draw text string
}

//
// wifi time functions
//
void connectToWiFi(const char* ssid, const char* pwd) {
  //Serial.println("Connecting to WiFi network: " + String(ssid));
  // delete old config
  WiFi.disconnect(true);
  //Serial.println("Waiting for WIFI connection...");
  //Initiate connection
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  //Serial.println("Connected.");
  //printWifiStatus();
}

unsigned long getNTPtime() {
  // module returns a unsigned long time valus as secs since Jan 1, 1970
  // unix time or 0 if a problem encounted
  //only send data when connected
  if (WiFi.status() == WL_CONNECTED) {
    //initializes the UDP state
    //This initializes the transfer buffer
    udp.begin(WiFi.localIP(), localPort);

    sendNTPpacket(timeServer); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);
    if (udp.parsePacket()) {
      //Serial.println("udp packet received");
      //Serial.println("");
      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, extract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // adjust time for timezone offset in secs +/- from UTC
      // WA time offset from UTC is +8 hours (28,800 secs)
      // + East of GMT
      // - West of GMT
      long tzOffset = 28800UL;
      // WA local time
      unsigned long adjustedTime;
      return adjustedTime = epoch + tzOffset;
    }
    else {
      // were not able to parse the udp packet successfully
      // clear down the udp connection
      udp.stop();
      return 0; // zero indicates a failure
    }
    // not calling ntp time frequently, stop releases resources
    udp.stop();
  }
  else {
    // network not connected
    return 0;
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char* address) {
  // set all bytes in the buffer to 0
  for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
    packetBuffer[i] = 0;
  }
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.println("");
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
  Serial.println("");
}

void draw_time() {
  now = rtc.now();
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(TFT_BLACK, TFT_WHITE); //set text color
  tft.drawString(String(now.twelveHour(), DEC), 165, 30); //draw text string
  tft.drawString(":", 185, 30); //draw text string
  tft.drawString(String(now.minute(), DEC), 190, 30); //draw text string
  

  if (now.isPM() == 0) {
    tft.drawString("AM", 210, 30); //draw text string
  }
  else {
    tft.drawString("PM", 210, 30); //draw text string
  }
}
//
// wifi time functions
//

void ultrasonic() {
  //ultraonic section
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trig_water, LOW);
  delayMicroseconds(5);
  digitalWrite(trig_water, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_water, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echo_water, INPUT);
  duration = pulseIn(echo_water, HIGH);

  // Convert the time into a distance
  //  inches = (duration / 2) / 74; // Divide by 74 or multiply by 0.0135
  cm = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343
  //return cm;
  //  Serial.print(inches);
  //  Serial.print("in, ");
  //Serial.print(cm);
  //Serial.print("cm");
  //Serial.println();

  //LCD Drawing for water gauge
  gauge = 100 * (1 - (float(cm) / 25));
  //Serial.println(gauge);
  gauge_gap = 100 - gauge;

  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(TFT_BLACK); //set text color
  tft.drawString("water", 30, 165); //draw text string
  tft.drawString(String(int(gauge)), 30, 185);
  tft.drawString("%", 60, 185);
  tft.drawRect(100, 130, 25, 100, TFT_BLUE); //A 25x100 black rectangle starting from (110, 70)
  tft.fillRect(100, 130 + gauge_gap, 25, gauge, TFT_BLUE); //A 25x100 black rectangle starting from (110, 70)
  if (gauge != new_gauge) {
    tft.fillRect(0, 121, 160, 120, TFT_WHITE); //A 25x100 black rectangle starting from (110, 70)
  }
  new_gauge = gauge;
  
}

void draw_temp(float temperature, float humidity) {
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.drawString(String(int(temperature)), 10, 30);
  tft.drawString("C", 30, 30);
  tft.drawString(String(int(humidity)), 10, 70);
}
