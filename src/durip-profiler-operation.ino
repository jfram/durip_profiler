/*
  DURIP profiler operation by Jonathan Fram. Based on Blink without Delay
  See SoftwareSerialExampleDURIP.ino
*/

//****INCLUDED LIBRARIES*********
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

// Constants
const int waterDepth = 25;
const int speed = 1; // m/s
const int cpm = 1000; // counts / meter
const int pinBrake = 2;  // toggle the Brake
const int pinMotor = 3;  // toggle the Motor
const long interval = 10*1000;  // interval at which to profile (milliseconds)
const int chipSelect = 4; //10; // for logging to the SD card

// Variables
DateTime now = DateTime(2023, 7, 9, 12,0, 0); 
String fileName="start2.txt"; 
File dataFile; 
RTC_DS3231 rtc; // RTC_PCF8523 rtc; // real time clock board type
String str="";
String cmd="";
SoftwareSerial mySerial(9,8); // RX, TX to the winch
int cnt=0;
int SDworks=1; // set to zero during testing to reduce wear and tear on the SD card
int RTCworks=0; 

void setup() {
  // set the digital pin as output:
  pinMode(pinBrake, OUTPUT);
  pinMode(pinMotor, OUTPUT);
  
  Serial.begin(9600); // #ifndef ESP8266
  while (!Serial); {  // this is to the main IO serial monitor.
   ; 
  } // wait for serial port to connect. Needed for native USB  // #endif
  Serial.println("Arduino serial port is ready");

  // set time to computer time , pcf8523 in RTClib. Does not need to be redone because board has a battery. Just verify occassionally.
  if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) { // ! rtc.initialized() || 
    Serial.print("RTC is NOT initialized, let's set the time!");
    Serial.println();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(2000);
  } // rtc.start();
  Serial.println("Realtime clock is ready.");

  mySerial.begin(9600);
  while (!mySerial) {
    ;
  }
  Serial.println("SoftwareSerial port is ready.");

  Serial.println("Initializing SD card... ");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed, or not present");
    SDworks=0;
  }
  Serial.println("SD card initialized.");
  
  // turn relays off
  digitalWrite(pinBrake, LOW);
  digitalWrite(pinMotor, LOW);
  Serial.println("setup function completed."); Serial.println("");
  delay(4*1000); // pause before looping. Time before first profile. 
}

void loop() {  // each profile is a loop
  Serial.print("elapsed sec: "); Serial.println(millis()/1000); // how long the script has been running
  delay(3600*1000);
  if (RTCworks>0) {
    now = rtc.now();
    // open text file with datayyyymmddHHMMSS.txt as the file name (datalogger example, chipSelect=10)
    // too long. fileName = String("data"+String(now.year())+str2digits(now.month())+str2digits(now.day())+str2digits(now.hour())+str2digits(now.minute())+str2digits(now.second())+".txt");
    fileName = String(str2digits(now.month())+str2digits(now.day())+str2digits(now.hour())+str2digits(now.minute())+".txt"); 
    }

  // open text file with mmddHHMM.txt as the file name
  File dataFile = SD.open(fileName, FILE_WRITE); 
  if (dataFile) {
    Serial.print("Opened "); Serial.println(fileName); dataFile.print("Opened "); dataFile.println(fileName);
  }   
  if (!dataFile) {
    Serial.print("Error opening "); Serial.println(fileName); dataFile.print("Error opening "); dataFile.println(fileName);
  }   
  if (RTCworks>0) {now=rtc.now(); Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL)); dataFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));}
  Serial.println("Turn on winch.");  dataFile.println("Turn on winch.");
  digitalWrite(pinBrake, LOW); // this will be setup to release the brake
  Serial.print("Brake state: ");   Serial.println(digitalRead(pinBrake)); 
  dataFile.print("Brake state: "); dataFile.println(digitalRead(pinBrake)); // to keep track of brake on. See SD DataLogger example
  digitalWrite(pinMotor,LOW); // power up winch power relay
  Serial.print("Motor state: ");     Serial.println(digitalRead(pinMotor)); 
  dataFile.println("Motor state: "); dataFile.println(digitalRead(pinMotor)); // after this commands go to the winch, so monitor to SD
  delay(4*1000); // pause to let the motor turn on

  // set winch parameters
  cmd="g r0x32"; 
  mySerial.print(cmd+"\n"); 
  str=mySerial.readString();
  Serial.print("Sent: ");   Serial.println(cmd);   dataFile.print("Sent: "); dataFile.println(cmd);    
  Serial.print("Response: "); Serial.println(str); dataFile.print("Response: "); dataFile.println(str); 

  // cmd="s r0x24 21";
  // go up at high speed using position mode (21)
  //  experiment with speed
  //  experiment with saving
  // save status as climb (Calculate seconds to move. make a for loop so sample every second while moving)

  //  if(mySerial.available()>0) {
  //   Serial.println("available");
  //   readline = mySerial.readString(); dataFile.println(readline);  Serial.println(readline);   
  //    mySerial.println("s r0x24 21");     dataFile.println("s r0x24 21");
  //    mySerial.println("s r0xca 400000"); dataFile.println("s r0xca 400000");
  //    mySerial.println("t 1"); delay(10); dataFile.println("t 1");
  //    now=rtc.now(); dataFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  //    mySerial.println("s r0xca 100000"); dataFile.println("s r0xca 100000");
  //    mySerial.println("t 1"); delay(10); dataFile.println("t 1");
  //    mySerial.println("s r0x24 30");     dataFile.println("s r0x24 30");   
  //    now=rtc.now(); dataFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  //  }

  // slowly unspool at surface
  // get status as unspool
  // reel in quickly
  // get status as reel
  // park slowly
  // get status as park

  // complete profile 
  if (RTCworks>0) {now=rtc.now();} 
  str=now.timestamp(DateTime::TIMESTAMP_FULL); 
  Serial.println("Turn off winch."); dataFile.println("Turn off winch.");
  digitalWrite(pinMotor,HIGH); // power down winch power relay
  Serial.print("Motor state: ");     Serial.println(digitalRead(pinMotor)); 
  dataFile.print("Motor state: "); dataFile.println(digitalRead(pinMotor)); // after this commands go to the winch, so monitor to SD
  digitalWrite(pinBrake,HIGH); // re-engage brake
  Serial.print("Brake state: ");     Serial.println(digitalRead(pinBrake)); 
  dataFile.print("Brake state: "); dataFile.println(digitalRead(pinBrake)); // to keep track of brake on. See SD DataLogger example
  Serial.print("Ended profile: "); dataFile.print("Ended profile: "); 
  cnt=cnt+1; Serial.println(cnt); dataFile.println(cnt);
  dataFile.close(); // close data file
  if (!dataFile) {
    Serial.println("Closed "+fileName); dataFile.println("Closed "+fileName);
  }   
  // Serial.println("past close.");   dataFile.println("past close.");
  Serial.println("");
  delay(interval);  // park until next profile. Eventually, 60 minutes - time to profile. 
  // it appends if the file name is not changed.
}

//******FUNCTIONS**************

// Function that adds leading Zeros
String str2digits(int number) {
  String s;
  s = "";
  if (number >= 0 && number < 10) {
    s="0";
  }
  s = s+String(number); 
  return s;
}

// Function that sends a string and receives a response. NOT COMPLETED YET
String commandwinch(String str) {
  if (SDworks>0) { 
    dataFile.println(str); 
  }
  Serial.println(str); 
  mySerial.print(str+"\n"); 
  str = mySerial.readString(); 
  return str;
}