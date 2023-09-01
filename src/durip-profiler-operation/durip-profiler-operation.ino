/*
  DURIP profiler operation by Andrew Scherer and Jon Fram.
*/

#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


DateTime now = DateTime(); 
char file_name[20]; 
File data_file; 

const int CPCM = 244;                            // counts/cm
const int SLOW_DEPTH = 2;                       // slowing down for final 2 meters on move down
int water_depth = 25;                             // water depth in meters
long accel = CPCM*100L/10L;                        // 1 m/s^2 accel/decel rate
long up_vel = CPCM*100L*10L;                        // 1 m/s upwards velocity
long down_vel = CPCM*25L*10L;                       // 0.5 m/s downwards velocity
long final_down_vel = CPCM*5L*10L;
long cur_pos = 0;

// bottom position is 2 meters above intended depth
// last two meters will be descended slower
long inter_pos = -water_depth*100L*CPCM+200L*CPCM;
long bot_pos = -water_depth*100L*CPCM;
long sur_pos = water_depth*100L*CPCM;

long move_status;
bool last_move_success = true;

long sur_wait = 10;                                // surface wait time (seconds)
long bot_wait = 10;                                // bottom wait time, time b/t profiles (seconds)
long first_wait = 10;                              // time before first profile (seconds)

RTC_DS3231 rtc;                                   // real time clock board type

const byte NUM_CHARS = 32;                         // maximum number of characters in commands or responses
char cmd[NUM_CHARS];
char res[NUM_CHARS];
char rec[NUM_CHARS];

const byte PIN_BRAKE = 9;                          // toggle the Brake
const byte PIN_MOTOR = 8;                          // toggle the Motor

int cnt=0;                                        // count profiles
byte sd_works=1;                                   // 0 disables SD card
byte rtc_works = 1;                                // 0 disables RTC
byte serial_setup = 1;                             // for setting up values via serial
const byte RTC_SYNC = 0;                           // set to 1 to force RTC to sync to computer clock
const byte SD_CHIP_SELECT = 53;                     // SD card chip select pin
const int RW_ADDRESS = 0;                          // address to read/write user changeable vars to eeprom

void setup () {


    pinMode(PIN_BRAKE, OUTPUT);
    pinMode(PIN_MOTOR, OUTPUT);
    // winch on during setup to ensure it is working
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);


    // initialize serial ports 
    Serial.begin(9600);     // for serial with computer
    while (!Serial);
    Serial3.begin(9600);    // for TX to winch
    while(!Serial3);
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
    if (RTC_SYNC) {
        Serial.println(F("Force syncing RTC..."));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        delay(2000);
    }
    Serial.println(F("Realtime clock is ready."));

    // setup sd card
    Serial.println(F("Initializing SD card... "));
    if (!SD.begin(SD_CHIP_SELECT)) {
        Serial.println(F("SD card failed, or not present"));
        sd_works=0;
    } else {
        Serial.println(F("SD card initialized."));
    }

    readVars();

    Serial.println(F("Hello! Please enter commands to change variables."));
    Serial.println(F("Entering 'help' displays available commands."));
    while (serial_setup != 0) {
        Serial.println("Ready for next serial command.");
        readSerialCommand();
        delay(1000);
    }

    saveVars();

    Serial.println(readVcc());

    if (water_depth < SLOW_DEPTH) {
        // if water depth is less than SLOW_DEPTH, prevent positive inter_pos and move slowly the whole way down
        inter_pos = 0;                  
    }

    // winch off until first profile
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);

    // first profile delay 
    delay(first_wait*1000);
}

void loop () {

    // setup file name based on date and time
    if (rtc_works>0) {
        now = rtc.now();
        sprintf_P(file_name, PSTR("%02d%02d%02d%02d.txt"), now.month(), now.day(), now.hour(), now.minute());
        Serial.print(F("Opening: "));
        Serial.println(file_name);
    }

    // open file
    File data_file = SD.open(file_name, FILE_WRITE); 
    if (data_file) {
        Serial.print(F("Opened: ")); Serial.println(file_name); 
        data_file.print(F("Opened: ")); data_file.println(file_name);
    }   
    if (!data_file) {
        Serial.print(F("Error opening: ")); Serial.println(file_name);
    }   

    // print timestamp into file and serial monitor
    if (rtc_works>0) {
        now=rtc.now();
        Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
    }

    // power winch motor and brake
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);
    Serial.println(F("Winch on."));
    data_file.println(F("Winch on."));
    delay(5000);

    // do the profile up
    moveUp(data_file);                         

    // winch off at surface
    digitalWrite(PIN_BRAKE, HIGH);
    digitalWrite(PIN_MOTOR, HIGH);
    Serial.print(F("Winch off.\n"));
    data_file.print(F("Winch off.\n"));
    
    // surface delay 
    delay(sur_wait*1000);

    // winch on for down move
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);
    Serial.print(F("Winch on.\n"));
    data_file.print(F("Winch on.\n"));
    delay(5000);

    // do the move down, slowing for the last SLOW_DEPTH
    moveDown(data_file, false);                                         
    moveDown(data_file, true);                      

    // winch off at bottom
    digitalWrite(PIN_BRAKE, HIGH);
    digitalWrite(PIN_MOTOR, HIGH);
    Serial.print(F("Winch off.\n"));
    data_file.print(F("Winch off.\n"));

    // log profile count
    data_file.print(F("Profile: "));
    data_file.println(cnt);
    cnt ++;

    // close data file
    data_file.close(); 
    if (!data_file) {
        Serial.print(F("Closed ")); Serial.println(file_name);
    } 

    // bottom delay between profiles
    delay(bot_wait*1000);
    
}

void commandWinch(char *cmd, File &data_file) {
    // send command to winch
    Serial3.print(cmd);
    if (cmd != "\n") {
        Serial.print(F("Sent: "));  Serial.print(cmd);
        data_file.print(F("Sent: ")); data_file.print(cmd);
    }
    delay(500);
    // receive and save response to char array res
    int idx = 0;
    while (Serial2.available() > 0) {
        res[idx] = Serial2.read();
        idx++;
        // overwrite last character if overflowing res memory
        if (idx >= NUM_CHARS) {
            idx = NUM_CHARS - 1;
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
        data_file.print(F("Response: ")); data_file.write(res);
    }
    delay(500);
}

void eventStatusRegister (File &data_file) {
    // send command as normal and get response (stored as res)
    sprintf_P(cmd, PSTR("g r0xa0\n"));      
    commandWinch(cmd, data_file);
    
    char *strtokIdx;
    strtokIdx = strtok(res, " ");              // remove "v" from response
    strtokIdx = strtok(NULL, " ");             // get remaining long from response 
    move_status = strtol(strtokIdx, NULL, 10);  // set value to move_status
    decodeMoveStatus(move_status);
}

void getPosition (File &data_file) {
    sprintf_P(cmd, PSTR("g r0x32\n"));                                  // send command requesting winch position
    commandWinch(cmd, data_file);                                       // response stored as res
    char *strtokIdx;
    strtokIdx = strtok(res, " ");                                       // remove "v" from response
    strtokIdx = strtok(NULL, " ");                                      // get remaining integer from response 
    cur_pos += strtol(strtokIdx, NULL, 10);                             // save current position
}

void moveUp (File &data_file) {
    commandWinch("\n", data_file);                                      // newline to clear winch serial buffer
    sprintf_P(cmd, PSTR("s r0xc8 0\n"));                                // set absolute move, trapezoid profile
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0x24 21\n"));                               // set position mode
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);                       // set acceleration
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);                       // set deceleration
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), up_vel);                      // set max velocity
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), sur_pos);                     // set final position of up move 
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("t 1\n"));                                      // initiate move
    commandWinch(cmd, data_file);                                       //
    long move_time = water_depth/(up_vel/(CPCM*10L))*1000L;             // expected time to complete move up 
    delay(move_time+5000L);
    eventStatusRegister(data_file);                                               
    getPosition(data_file);                                             // get final position of move, save to cur_pos
    Serial.print(F("Winch position: ")); Serial.println(cur_pos);
    data_file.print(F("Winch position: ")); data_file.println(cur_pos);
    bot_pos = -cur_pos;                                                 // always return to original position of 0
}

void moveDown (File &data_file, bool last_meters) {
    commandWinch("\n", data_file);                                      // newline to clear winch serial buffer
    sprintf_P(cmd, PSTR("s r0xc8 0\n"));                                // set absolute move, trapezoid profile
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0x24 21\n"));                               // set position mode
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);                       // set acceleration
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);                       // set deceleration
    commandWinch(cmd, data_file);                                       //
    if (!last_meters) {                                                 // faster speed for most of way down
        sprintf_P(cmd, PSTR("s r0xcb %ld\n"), down_vel);                // set max velocity
        commandWinch(cmd, data_file);                                   //
        sprintf_P(cmd, PSTR("s r0xca %ld\n"), inter_pos);               // set final position of down move 
        commandWinch(cmd, data_file);                                   //
        sprintf_P(cmd, PSTR("t 1\n"));                                  // initiate move
        commandWinch(cmd, data_file);                                   //
        long move_time = (water_depth-2)/(down_vel/(CPCM*10L))*1000L;   // expected time to complete move down 
        delay(move_time);                                               //
    } else if (last_meters) {                                           // slow down in final 2 meters
        sprintf_P(cmd, PSTR("s r0xcb %ld\n"), final_down_vel);          // set max velocity
        commandWinch(cmd, data_file);                                   //
        sprintf_P(cmd, PSTR("s r0xca %ld\n"), bot_pos);                 // final 2 meters of movement
        commandWinch(cmd, data_file);                                   //
        sprintf_P(cmd, PSTR("t 1\n"));                                  // initiate move
        commandWinch(cmd, data_file);                                   //
        long move_time = (2)/(final_down_vel/(CPCM*10L))*1000;          // expected time to complete move down 
        delay(move_time+500L);
    }
    eventStatusRegister(data_file);                                     // loop checking if winch is still moving
    getPosition(data_file);                                             // get final position of move, update cur_pos
    Serial.print(F("Winch position: ")); Serial.println(cur_pos);
    data_file.print(F("Winch position: ")); data_file.println(cur_pos);
    sur_pos = sur_pos - cur_pos;                                        // correct for slight inaccuracies in winch movement
}

void decodeMoveStatus(long val) {
    // change move status to binary for decoding error codes
    Serial.print(F("Decoded Response: 0b"));
    for (int i = NUM_CHARS; i >= 0; i--) {
        Serial.write(bitRead(val, i) ? '1' : '0');
    }
    Serial.println();
}

long readVcc() {
  // https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

byte readSerialCommand () {
    // receive commands from PC to set user modifiable variables
    // wait for serial input
    while (Serial.available() == 0) {
        delay(100);
    }
    // read serial input
    int idx = 0;
    while (Serial.available() > 0) {
        rec[idx] = Serial.read();
        idx++;
        // overwrite last character if overflowing rec memory
        if (idx >= NUM_CHARS) {
            idx = NUM_CHARS - 1;
        }
        delay(50);
    }
    rec[idx] = '\0';    // set end string bit
    idx = 0;            // reset idx
    // decode serial input according to format shown in help()
    char *strtokIdx;
    strtokIdx = strtok(rec, " ");          
    if (strcmp(strtokIdx, "g") == 0) {
        strtokIdx = strtok(NULL, " ");
        getVar(strtokIdx);
    } else if (strcmp(strtokIdx, "s") == 0) {
        // sets val to 0 if incorrectly input - need to handle this somehow
        strtokIdx = strtok(NULL, " ");
        long val = strtol(strtok(NULL, " "), NULL, 10);
        if (val == 0L) {
            Serial.println(F("Invalid value input - no action taken."));
        } else {
            setVar(strtokIdx, val);
        }
    } else if (strcmp(strtokIdx, "help") == 0) {
        help();
    } else if (strcmp(strtokIdx, "exit") == 0) {
        // this option will end setup loop and begin profiling
        exit();  
    } else {
        Serial.println(F("Unrecognized command - no action taken."));
    }
}

void setVar(char *c, long val) {
    // set current user-modifiable variables
    if (strcmp(c, "velup") == 0) {
        up_vel = val*CPCM*10L;
        getVar("velup");
    } else if (strcmp(c, "veldown") == 0) {
        down_vel = val*CPCM*10L;
        getVar("veldown");
    } else if (strcmp(c, "depth") == 0) {
        water_depth = val;
        inter_pos = -water_depth*100L*CPCM+200L*CPCM;
        bot_pos = -water_depth*100L*CPCM;
        sur_pos = water_depth*100L*CPCM;
        getVar("depth");
    } else if (strcmp(c, "surfwait") == 0) {
        sur_wait = val;
        getVar("surfwait");
    } else if (strcmp(c, "bottwait") == 0) {
        sur_wait = val;
        getVar("bottwait");
    } else if (strcmp(c, "firstwait") == 0) {
        first_wait = val;
        getVar("first_wait");
    } else {
        Serial.println(F("Unrecognized variable name - no action taken."));
    }
}

void getVar(char *c) {
    // retrieve current user-modifiable variables
    if (strcmp(c, "velup") == 0) {
        Serial.print(F("Up velocity (cm/s): "));
        Serial.println(up_vel/(CPCM*10L));    
    } else if (strcmp(c, "veldown") == 0) {
        Serial.print(F("Down velocity (cm/s): "));
        Serial.println(down_vel/(CPCM*10L));
    } else if (strcmp(c, "depth") ==0) {
        Serial.print(F("Water depth (m): "));
        Serial.println(water_depth);
    } else if (strcmp(c, "surfwait") == 0) {
        Serial.print(F("Surface wait time (s): "));
        Serial.println(sur_wait);    
    } else if (strcmp(c, "bottwait") == 0) {
        Serial.print(F("Bottom wait time (s): "));
        Serial.println(bot_wait);
    } else if (strcmp(c, "firstwait") == 0) {
        Serial.print(F("First profile wait time (s): "));
        Serial.println(first_wait);
    } else {
        Serial.println(F("Unrecognized variable name - no action taken."));
    }
}

void help() {
    // explains how to use serial interface
    Serial.println(F("Begin line with 'g' to get current values or 's' to set values."));
    Serial.println("");
    Serial.println(F("Enter a space then the variable name you wish to get or set. Options are:"));
    Serial.println(F("'velup': The max velocity of the profiler on its upwards profile in cm/s."));
    Serial.println(F("'veldown': The max velocity of the profiler on its downwards return in cm/s."));
    Serial.println(F("'depth': The water depth in meters."));
    Serial.println(F("'surfwait': The wait time at the surface in seconds."));
    Serial.println(F("'bottwait': The wait time at the bottom in seconds. AKA, the time between profiles."));
    Serial.println(F("'firstwait': The wait time before beginning the first profile, after this setup is finished."));
    Serial.println("");
    Serial.println(F("If setting a value, enter a space and the value you wish to set the variable to."));
    Serial.println("");
    Serial.println(F("Wait for any responses to indicate success."));
    Serial.println(F("The serial monitor will then reply to tell you when you can input the next command."));
    Serial.println("");
    Serial.println(F("When finished, enter 'exit' to finish setup and begin profiling operations."));
} 

void exit() {
    // exits setup loop, begins profiling
    serial_setup = 0;
    Serial.print(F("Beginning profiling in "));
    Serial.print(first_wait);
    Serial.println(F("seconds. Current settings are:"));
    getVar("velup");
    getVar("veldown");
    getVar("depth");
    getVar("surfwait");
    getVar("bottwait");
    getVar("firstwait");
    Serial.println(F("Variables are saved and will remain until changed."));
    Serial.println(F("Good luck profiling! :)"));
}

void saveVars() {
    // saves user modifiable variables to memory
    EEPROM.put(RW_ADDRESS, up_vel);
    EEPROM.put(RW_ADDRESS + 4, down_vel);
    EEPROM.put(RW_ADDRESS + 8, water_depth);
    EEPROM.put(RW_ADDRESS + 12, sur_wait);
    EEPROM.put(RW_ADDRESS + 16, bot_wait);
    EEPROM.put(RW_ADDRESS + 20, first_wait);
}

void readVars() {
    // reads user modifiable variables from memory
    EEPROM.get(RW_ADDRESS, up_vel);
    EEPROM.get(RW_ADDRESS + 4, down_vel);
    EEPROM.get(RW_ADDRESS + 8, water_depth);
    EEPROM.get(RW_ADDRESS + 12, sur_wait);
    EEPROM.get(RW_ADDRESS + 16, bot_wait);
    EEPROM.get(RW_ADDRESS + 20, first_wait);
}