/*
  DURIP profiler operation by Andrew Scherer and Jon Fram.
*/

#include <RTClib.h>
#include <SPI.h>
#include <SD.h>


// include water depth, velocity, between profile times, and surface wait time as parameters
// slow down last 2 meters on the way back down 
// up speed = 1m/s
// down speed = 0.15 m/s
// last 2 meters less than that
//
//
//
//


DateTime now = DateTime(); 
char fileName[20]; 
File dataFile; 

const byte waterDepth = 25;
const byte speed = 10;               // m/s
const int cpm = 1000;                // counts / meter
const int accel = cpm*speed;         // 1 m/s^2 accel/decel rate
const int upVel = cpm*speed;         // 1 m/s upwards velocity
const int downVel = cpm*speed;       // 0.5 m/s downwards velocity
const long interval = 10*1000;       // interval at which to profile (milliseconds)
long curPos= 0;
long botPos = -20000;
long surPos = 10000;
long moveStatus;

RTC_DS3231 rtc;                      // real time clock board type

const byte numChars = 32;            // no commands or responses should be longer than numChars
char cmd[numChars];
char res[numChars];

const byte pinBrake = 9;            // toggle the Brake
const byte pinMotor = 8;            // toggle the Motor

int cnt=0;                           // count profiles
byte SDworks=1;                      // 0 disables SD card
byte RTCworks = 1;                   // 0 disables RTC
const byte RTCsync = 0;              // set to 1 to force RTC to sync to computer clock

const byte SDchipSelect = 53;        // SD card chip select pin


void setup () {

    pinMode(pinBrake, OUTPUT);
    pinMode(pinMotor, OUTPUT);
    // winch off during startup
    digitalWrite(pinBrake, HIGH);
    digitalWrite(pinMotor, HIGH);


    // initialize serial ports 
    Serial.begin(9600);     // for serial with computer
    while (!Serial);
    Serial1.begin(9600);    // for TX to winch
    while(!Serial1);
    Serial2.begin(9600);    // for RX from winch
    while(!Serial2);    

    // clear input buffer 
    while (Serial2.available() > 0) {
        Serial2.read();
    }

    // setup real time clock
    if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
        Serial.flush();
        while (1) delay(10);
    }
    if (rtc.lostPower()) { // ! rtc.initialized() || 
        Serial.println(F("RTC lost power, syncing time..."));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        delay(2000);
    } // rtc.start();
    if (RTCsync) {
        Serial.println(F("Force syncing RTC..."));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        delay(2000);
    }
    Serial.println(F("Realtime clock is ready."));

    // setup sd card
    Serial.println(F("Initializing SD card... "));
    if (!SD.begin(SDchipSelect)) {
        Serial.println(F("SD card failed, or not present"));
        SDworks=0;
    } else {
        Serial.println(F("SD card initialized."));
    }

    // first profile delay 
    delay(1000);
    
}

void loop () {

    // setup file name based on date and time
    if (RTCworks>0) {
        now = rtc.now();
        sprintf_P(fileName, PSTR("%02d%02d%02d%02d.txt"), now.month(), now.day(), now.hour(), now.minute());
        Serial.print(F("Opening: "));
        Serial.println(fileName);
    }

    // open file
    File dataFile = SD.open(fileName, FILE_WRITE); 
    if (dataFile) {
        Serial.print(F("Opened: ")); Serial.println(fileName); 
        dataFile.print(F("Opened: ")); dataFile.println(fileName);
    }   
    if (!dataFile) {
        Serial.print(F("Error opening: ")); Serial.println(fileName);
    }   

    // print timestamp into file and serial monitor
    if (RTCworks>0) {
        now=rtc.now();
        Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        dataFile.println(now.timestamp(DateTime::TIMESTAMP_FULL));
    }


    // power winch motor and brake
    digitalWrite(pinBrake, LOW);
    digitalWrite(pinMotor, LOW);
    Serial.println(F("Winch on."));
    dataFile.println(F("Winch on."));
    delay(10000);

    setUpMove(dataFile);                      // set move parameters

    sprintf_P(cmd, PSTR("t 1\n"));            // initiate move
    commandWinch(cmd, dataFile);              //

    isMoving(dataFile);                       // loop checking if winch is still moving

    moveSuccess(dataFile);                    // check if move was successful 

    getPosition(dataFile);                    // get final position of move, save to curPos

    // winch off at surface
    digitalWrite(pinBrake, HIGH);
    digitalWrite(pinMotor, HIGH);
    Serial.print(F("Winch off.\n"));
    dataFile.print(F("Winch off.\n"));
    
    // surface delay 
    delay(5000);

    // winch on for down move
    digitalWrite(pinBrake, LOW);
    digitalWrite(pinMotor, LOW);
    Serial.print(F("Winch on.\n"));
    dataFile.print(F("Winch on.\n"));
    delay(10000);

    setDownMove(dataFile);                      // set move parameters

    sprintf_P(cmd, PSTR("t 1\n"));            // initiate move
    commandWinch(cmd, dataFile);              //

    isMoving(dataFile);                       // loop checking if winch is still moving

    moveSuccess(dataFile);                    // check if move was successful 

    getPosition(dataFile);                    // get final position of move, save to curPos

    // winch off at bottom
    digitalWrite(pinBrake, HIGH);
    digitalWrite(pinMotor, HIGH);
    Serial.print(F("Winch off.\n"));
    dataFile.print(F("Winch off.\n"));


    dataFile.print(F("Profile: "));
    dataFile.println(cnt);
    cnt ++;

    dataFile.close(); // close data file
    if (!dataFile) {
        Serial.print(F("Closed ")); Serial.println(fileName);
  }   

    // bottom delay
    delay(15000);
    delay(1000);

}

void commandWinch(char *cmd, File &dataFile) {
    // send command to winch
    Serial1.print(cmd);
    if (cmd != "\n") {
        Serial.print(F("Sent: "));  Serial.print(cmd);
        dataFile.print(F("Sent: ")); dataFile.print(cmd);
    }

    delay(1000);
    // receive and save response to char array res
    int idx = 0;
    while (Serial2.available() > 0) {
        res[idx] = Serial2.read();
        idx++;
        // overwrite last character if overflowing res memory
        if (idx >= numChars) {
            idx = numChars - 1;
        }
    }
    res[idx] = '\0';    // set end string bit
    idx = 0;            // reset idx
    // clear input buffer 
    while (Serial2.available() > 0) {
        Serial2.read();
    }
    // write output
    if (cmd != "\n") {
        Serial.print(F("Response: ")); Serial.write(res);
        dataFile.print(F("Response: ")); dataFile.write(res);
    }
    delay(1000);
}


void isMoving (File &dataFile) {
    while (true) {
        // send command as normal and get response (stored as res)
        sprintf_P(cmd, PSTR("g r0xa0\n"));      
        commandWinch(cmd, dataFile);
        
        char *strtokIdx;
        strtokIdx = strtok(res, " ");              // remove "v" or "e" from response
        strtokIdx = strtok(NULL, " ");             // get remaining long from response 
        moveStatus = strtol(strtokIdx, NULL, 10);  // set value to moveStatus
        // break loop if move complete (i.e., moveStatus==0)
        if (moveStatus == 0) {
            Serial.println(F("Move successful."));
            dataFile.println(F("Move successful."));
            break;
        } 
    }
}

void getPosition (File &dataFile) {
    sprintf_P(cmd, PSTR("g r0x32\n"));     // send command requesting winch position
    commandWinch(cmd, dataFile);           // response stored as res

    char *strtokIdx;
    strtokIdx = strtok(res, " ");          // remove "v" or "e" from response
    strtokIdx = strtok(NULL, " ");         // get remaining integer from response 
    curPos += strtol(strtokIdx, NULL, 10);  // save current position
}

void setUpMove (File &dataFile) {
    commandWinch("\n", dataFile);                           // newline to clear winch serial buffer
    sprintf_P(cmd, PSTR("s r0xc8 0\n"));                    // set absolute move, trapezoid profile
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0x24 21\n"));                   // set position mode
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcb %i\n"), upVel);            // set max velocity
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcc %i\n"), accel);            // set acceleration
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcd %i\n"), accel);            // set deceleration
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), surPos);          // set final position of up move 
    commandWinch(cmd, dataFile);
}

void setDownMove (File &dataFile) {
    commandWinch("\n", dataFile);                           // newline to clear winch serial buffer
    sprintf_P(cmd, PSTR("s r0xc8 0\n"));                    // set absolute move, trapezoid profile
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0x24 21\n"));                   // set position mode
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcb %i\n"), downVel);          // set max velocity
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcc %i\n"), accel);            // set acceleration
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xcd %i\n"), accel);            // set deceleration
    commandWinch(cmd, dataFile);
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), botPos);          // set final position of down move 
    commandWinch(cmd, dataFile);
}

void moveSuccess(File &dataFile) {
    sprintf_P(cmd, PSTR("g r0xc9\n"));
    commandWinch(cmd, dataFile);

    char *strtokIdx;
    strtokIdx = strtok(res, " ");                // remove "v" or "e" from response
    strtokIdx = strtok(NULL, " ");               // get remaining integer from response 
    moveStatus = strtol(strtokIdx, NULL, 10);    // save current position
    if (moveStatus == 0) {
        Serial.println(F("Move was successful."));
        dataFile.println(F("Move was successful."));
    } else if (moveStatus == 16385) {
        Serial.println(F("Move was aborted."));
        dataFile.println(F("Move was aborted."));
        // perhaps do something if the move is aborted?
        // attempt another move? 
    } else {
        Serial.println(F("Move status unknown."));
        dataFile.println(F("Move was unknown."));        
    }
}

void decodeResponse() {

    char *strtokIdx;
    strtokIdx = strtok(res, " ");          // remove "v" or "e" from response
    strtokIdx = strtok(NULL, " ");         // get remaining integer from response 
    curPos = strtol(strtokIdx, NULL, 10);  // save current position
    // separate letter and number in winch response
    // if letter is e, there was an error, return that
    // if letter is v, get the response code for other function
}
