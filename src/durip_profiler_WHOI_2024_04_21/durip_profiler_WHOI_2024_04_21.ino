/*
  rapid profiling system (RPS) operation by Andrew Scherer and Jon Fram.
  2023-09-26 edits from jfram begin. Mostly max/min factor related. The other 90%+ of the code is from Andrew in summer 2023.
  Features designed based on Andrew and jfram's discussions, minimal code by jfram from spring 2023, and feedback from Rocky Geyer via jfram. 
  2024-04-20 major rewrite by jfram. 
  Jim Lerczak, principal investigator
  funded by the Office of Naval Research DURIP program
*/

// turning the brake on/off does the expected thing.
// does powering the brake on/off also power the winch on/off?

// timer. Keep track of elapsed time

#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// global variable setup
DateTime now = DateTime();       //
char file_name[20];              //
File data_file;                  //
                                 //
const long CPCM = 244;           // counts/cm (empirically measured from 100 ft of line payout)
long water_depth = 28;           //* CPCM * 100;// Vineyard Sound 28?                   // water depth in meters. This and other parameters are loaded from EPROM, not here.
long accel = CPCM * 100 / 10;    // 1 m/s^2 accel/decel rate
long up_vel = CPCM * 100 * 10;   // 1 m/s upwards velocity
long down_vel = CPCM * 25 * 10;  // 0.25 m/s initial down velocity
long bot_vel = CPCM * 15 * 10;   // 0.10 m/s final down velocity
long sur_wait = 3;               // surface wait time (seconds)
long bot_wait = 60;              // bottom wait time, time b/t profiles (seconds)
long first_wait = 10;            // time before first profile (seconds)
long cur_pos = 0;                // current position
long reed_time = 0.0;            // seconds from start to end of moveUp
long startnow = 0.0;             // unix time of move up in seconds
long endnow = 0.0;               // unix time of move up in seconds
long testvar = 0;                // 2024-04-22 testing
RTC_DS3231 rtc;                  // real time clock board type
                                 //
const byte NUM_CHARS = 32;       // maximum number of characters in commands or responses
char cmd[NUM_CHARS];             // commands for sending to winch
char res[NUM_CHARS];             // response from winch
char rec[NUM_CHARS];             // serial input from PC
                                 //
const byte PIN_BRAKE = 9;        // toggle the Brake
const byte PIN_MOTOR = 8;        // toggle the Motor
const byte PIN_REED = 7;         // toggle the Reed Switch
                                 //
int cnt = 0;                     // count profiles
int tcnt = 0;                    // telemetry profiles
int waitEveryX = 3;              // how often to surfwait
int maxProfiles = 1200;          // When to stop profiling
bool sd_works = true;            // enables/disables SD card
bool rtc_works = true;           // enables/disables RTC
bool serial_setup = true;        // for setting up values via serial
const bool RTC_SYNC = false;     // set to true to force RTC to sync to compile time
const byte SD_CHIP_SELECT = 53;  // SD card chip select pin
const int RW_ADDRESS = 0;        // address to read/write user modifiable vars to eeprom

void commandWinch(char *cmd, File &data_file) {
  // send command to winch
  Serial3.print(cmd);
  if (cmd != "\n") {
    Serial.print(F("Sent: "));
    Serial.print(cmd);
    data_file.print(F("Sent: "));
    data_file.print(cmd);
  }
  delay(500);  // milliseconds. It is unreliable with less.
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
  res[idx] = '\0';  // set end string bit
  idx = 0;          // reset idx
  // clear input buffer
  while (Serial2.available() > 0) {
    Serial2.read();
  }
  // write output
  if (cmd != "\n") {
    Serial.print(F("Response: "));
    Serial.write(res);
    data_file.print(F("Response: "));
    data_file.write(res);
  }
  delay(500);
}

void commandWinchNoDelay(char *cmd) {
  // send command to winch
  Serial3.print(cmd);
  // if (cmd != "\n") {
  // Serial.print(F("Sent : "));  Serial.print(cmd); // slows us down
  //}
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
  res[idx] = '\0';  // set end string bit
  idx = 0;          // reset idx
  // clear input buffer
  while (Serial2.available() > 0) {
    Serial2.read();
  }
  // write output to the screen, not to the SD card, which is slow.
  if (cmd != "\n") {
    // Serial.print(cmd); Serial.print(F(" ")); Serial.write(res); Serial.println(F(" "));
    Serial.write(res);
    // Serial.println(F("Fast Response : "));
  }
}

void moveUp(File &data_file) {
  Serial.println(F("moveUp start"));
  data_file.println(F("moveUp start"));
  now = rtc.now();
  data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println(F("Brake released."));
  data_file.println(F("Brake released."));
  digitalWrite(PIN_BRAKE, LOW);
  commandWinch("\n", data_file);                                    // newline to clear winch serial buffer. Confirmed that we need this.
  sprintf_P(cmd, PSTR("s r0xc8 256\n"));                            // set relative move, trapezoid profile. Move to setup. Don't change it.
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("s r0x24 21\n"));                             // set position mode. Move to setup. Don't change it.
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);                     // set acceleration. Move to setup. Don't change it.
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);                     // set deceleration. Move to setup. Don't change it.
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("s r0xcb %ld\n"), up_vel);                    // set upward velocity
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("s r0xca %ld\n"), water_depth * CPCM * 100);  // move to surface at water_depth
  commandWinch(cmd, data_file);                                     //
  sprintf_P(cmd, PSTR("t 1\n"));                                    // initiate move
  commandWinch(cmd, data_file);

  // Google says it can read a pin in 4.8 microseconds.
  // looped from 347000 to 988000 in 10 seconds, so 64,100 per second or 15.6 microseconds per loop.
  // Also try writing to disk as one goes up. See how much it slows it down.
  now = rtc.now();
  startnow = now.unixtime();
  long i = 0;
  int j = 0;
  int reedval = 0;
  delay(2000);                   // it take 2 seconds to accelerate. Don't trip the switch in this time 2024-02-20
  digitalWrite(PIN_REED, HIGH);  // changed from LOW to HIGH to make use of pullup resistors
  reedval = digitalRead(PIN_REED);
  long move_time = abs(water_depth * CPCM * 100 * 10 / up_vel);
  move_time = move_time * 1000 + 1000;  // expected time to complete move to surface plus buffer
  Serial.print(move_time);
  Serial.println(F(" move time"));
  tcnt++;  // add 1 to the telemetry spacing counter.
  while (i < move_time) {
    reedval = digitalRead(PIN_REED);
    if (reedval < 1) {
      i = move_time;
      digitalWrite(PIN_BRAKE, HIGH);  //  close brake
      sprintf_P(cmd, PSTR("t 0\n"));  // stop move
      commandWinch(cmd, data_file);
      Serial.println(F("Winch off and brake applied from reed switch."));
      data_file.println(F("Winch off and brake applied from reed switch."));
      if (tcnt >= waitEveryX) {
        // move up 2 sec at down_vel
        Serial.println(F("Brake released during telemetry."));
        data_file.println(F("Brake released during telemetry."));
        digitalWrite(PIN_BRAKE, LOW);
        commandWinch("\n", data_file);                          // newline to clear winch serial buffer. Confirmed that we need this.
        sprintf_P(cmd, PSTR("s r0xc8 256\n"));                  // set relative move, trapezoid profile. Move to setup. Don't change it.
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("s r0x24 21\n"));                   // set position mode. Move to setup. Don't change it.
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);           // set acceleration. Move to setup. Don't change it.
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);           // set deceleration. Move to setup. Don't change it.
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("s r0xcb %ld\n"), down_vel);        // set velocity
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("s r0xca %ld\n"), CPCM * 100 / 2);  //
        commandWinch(cmd, data_file);                           //
        sprintf_P(cmd, PSTR("t 1\n"));                          // initiate move
        commandWinch(cmd, data_file);
        delay(2 * 1000);
        digitalWrite(PIN_BRAKE, HIGH);
        data_file.println(F("Brake applied during telemetry."));
        Serial.println(F("Brake applied during telemetry."));
        tcnt = 0;
        delay(sur_wait * 1000);
      }
    }
    i = i + 20;  // milliseconds
    delay(20);   // 10 to 20 2024-04-19
    j = j + 1;
    if (j > 100) {
      Serial.println(reedval);
      j = 0;
      Serial.println(i);
    }
  }
  now = rtc.now();
  endnow = now.unixtime();
  reed_time = endnow - startnow;
  // Serial.print(startnow); Serial.println(F(" start"));
  // Serial.print(endnow);   Serial.println(F(" end"));

  if (reedval > 0) {                //switch didn't trigger
    digitalWrite(PIN_BRAKE, HIGH);  //   engage brake.
    Serial.println(F("Brake engaged not due to reed switch."));
    data_file.println(F("Brake engaged not due to reed switch."));
  }
  Serial.println(F("Finished moveUp."));
  data_file.println(F("finished moveUp."));
  now = rtc.now();
  data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
}

void moveDown(File &data_file, int moveorpark) {
  Serial.println(F("starting moveDown"));
  data_file.println(F("starting moveDown"));
  now = rtc.now();
  data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  long move_time = 0;
  long i = 0;
  long crnt = 0;
  char *strtokIdx;
  long old_pos = 0;  // redo as an array
  long old_pos0 = 0;
  long old_pos1 = 0;
  long old_pos2 = 0;
  long old_pos3 = 0;
  long old_pos4 = 0;
  long old_pos5 = 0;
  long old_pos6 = 0;
  long old_pos7 = 0;
  long old_pos8 = 0;
  long old_pos9 = 0;
  if (moveorpark > 0) {
    Serial.println(F("Brake off."));
    data_file.println(F("Brake off."));
    digitalWrite(PIN_BRAKE, LOW);
    commandWinch("\n", data_file);                    // newline to clear winch serial buffer. We do need this.
    sprintf_P(cmd, PSTR("s r0xc8 256\n"));            // set relative move, trapezoid profile. Move to setup. Don't change it.
    commandWinch(cmd, data_file);                     //
    sprintf_P(cmd, PSTR("s r0x24 21\n"));             // set position mode. Move to setup. Don't change it.
    commandWinch(cmd, data_file);                     //
    sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);     // set acceleration. Move to setup. Don't change it.
    commandWinch(cmd, data_file);                     //
    sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);     // set deceleration. Move to setup. Don't change it.
    commandWinch(cmd, data_file);                     //
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), down_vel);  // set  velocity
    commandWinch(cmd, data_file);
    if (water_depth > 3) {                                                     //
      sprintf_P(cmd, PSTR("s r0xca %ld\n"), -(water_depth - 3) * CPCM * 100);  // spool down
    } else {
      sprintf_P(cmd, PSTR("s r0xca %ld\n"), 0 * CPCM * 100);  // do not spool down fast in super shallow water
    }
    commandWinch(cmd, data_file);   //
    sprintf_P(cmd, PSTR("t 1\n"));  // initiate move
    commandWinch(cmd, data_file);   //
    // move_time = abs((water_depth - 3) * CPCM * 100 / (down_vel / (10)));  // expected time to complete move down
    move_time = (reed_time * up_vel - 3 * CPCM * 100) / down_vel;
    if (move_time<10){
      move_time = 10;
    }
    //Serial.print(move_time);  Serial.println(F(" move_time "));
    move_time = move_time * 800;  // 800 instead of 1000 because it keeps being too long. Not sure why
    while (i < move_time) {
      sprintf_P(cmd, PSTR("g r0x0c\n"));   // get motor current
      commandWinchNoDelay(cmd);            //
      strtokIdx = strtok(res, " ");        // remove "v" from response
      strtokIdx = strtok(NULL, " ");       // get remaining long from response
      crnt = strtol(strtokIdx, NULL, 10);  // set value to winch current crnt
      delay(20);
      if (crnt < -2000.0) { // was -1300 on 2024-04-22
        sprintf_P(cmd, PSTR("g r0x0c\n"));   // get motor current
        commandWinchNoDelay(cmd);            //
        strtokIdx = strtok(res, " ");        // remove "v" from response
        strtokIdx = strtok(NULL, " ");       // get remaining long from response
        crnt = strtol(strtokIdx, NULL, 10);  // set value to winch current crnt
        if (crnt < -2000.0) {                // are we sure?
          i = move_time;
          sprintf_P(cmd, PSTR("t 0\n"));  // stop move
          commandWinchNoDelay(cmd);
          Serial.print(crnt);
          Serial.println(F(" Winch stopped because of high motor current."));
          data_file.print(crnt);
          data_file.println(F(" Winch stopped because of high motor current."));
        }
      }
      if (-10 < crnt && crnt < 10) {
        sprintf_P(cmd, PSTR("g r0x0c\n"));   // get motor current again to be sure
        commandWinchNoDelay(cmd);            //
        strtokIdx = strtok(res, " ");        // remove "v" from response
        strtokIdx = strtok(NULL, " ");       // get remaining long from response
        crnt = strtol(strtokIdx, NULL, 10);  // set value to winch current crnt
        if (-10 < crnt && crnt < 10) {       // are we sure?
          i = move_time;
          digitalWrite(PIN_BRAKE, HIGH);
          sprintf_P(cmd, PSTR("t 0\n"));  // stop move
          commandWinchNoDelay(cmd);
          Serial.print(crnt);
          Serial.println(F(" Winch stopped because out of gear."));
          data_file.print(crnt);
          data_file.println(F(" Winch stopped because out of gear."));
        }
      }
      i = i + 20;
    }
    digitalWrite(PIN_BRAKE, HIGH);  // timed out or otherwise did not reach the bottom
    Serial.println(F("Winch stopped at end of fast moveDown."));
    data_file.println(F("Winch stopped at end of fast moveDown."));
  }
  digitalWrite(PIN_BRAKE, HIGH);
  Serial.println(F("Brake applied."));
  data_file.println(F("Brake applied."));
  Serial.println(F("ending moveDown"));
  data_file.println(F("ending moveDown"));
  now = rtc.now();
  data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
}

void getVar(char *c, bool all = false) {
  // retrieve current user-modifiable variables
  if (strcmp(c, "upvel") == 0) {
    Serial.print(F("Up velocity (cm/s): "));
    Serial.println(up_vel / (CPCM * 10L));
  } else if (strcmp(c, "downvel") == 0) {
    Serial.print(F("Down velocity (cm/s): "));
    Serial.println(down_vel / (CPCM * 10L));
  } else if (strcmp(c, "depth") == 0) {
    Serial.print(F("Water depth (m): "));
    Serial.println(water_depth);
  } else if (strcmp(c, "surfwait") == 0) {
    Serial.print(F("Surface wait time (min): "));
    Serial.println(sur_wait / 60);
  } else if (strcmp(c, "bottwait") == 0) {
    Serial.print(F("Bottom wait time (min): "));
    Serial.println(bot_wait / 60);
  } else if (strcmp(c, "firstwait") == 0) {
    Serial.print(F("First profile wait time (min): "));
    Serial.println(first_wait / 60);
  } else if (strcmp(c, "bottvel") == 0) {
    Serial.print(F("Bottom velocity (cm/s): "));
    Serial.println(bot_vel / (CPCM * 10L));
  } else if (strcmp(c, "all") == 0) {
    getVar("upvel");
    getVar("downvel");
    getVar("bottvel");
    getVar("depth");
    getVar("surfwait");
    getVar("bottwait");
    getVar("firstwait");
  } else {
    Serial.println(F("Unrecognized variable name - no action taken."));
  }
}

void setVar(char *c, long val) {
  // set current user-modifiable variables
  if (strcmp(c, "upvel") == 0) {
    up_vel = val * CPCM * 10L;
    getVar("upvel");
  } else if (strcmp(c, "downvel") == 0) {
    down_vel = val * CPCM * 10L;
    getVar("downvel");
  } else if (strcmp(c, "depth") == 0) {
    water_depth = val;
    getVar("depth");
  } else if (strcmp(c, "surfwait") == 0) {
    sur_wait = val * 60;
    getVar("surfwait");
  } else if (strcmp(c, "bottwait") == 0) {
    bot_wait = val * 60;
    getVar("bottwait");
  } else if (strcmp(c, "firstwait") == 0) {
    first_wait = val * 60;
    getVar("firstwait");
  } else if (strcmp(c, "bottvel") == 0) {
    bot_vel = val * CPCM * 10L;
    getVar("bottvel");
  } else {
    Serial.println(F("Unrecognized variable name - no action taken."));
  }
}

void help() {
  // explains how to use serial interface
  Serial.println(F("Begin line with 'g' to get current values or 's' to set values."));
  Serial.println(F(""));
  Serial.println(F("Enter a space then the variable name you wish to get or set. Options are:"));
  Serial.println(F("'upvel': The max velocity of the profiler on its upwards profile in cm/s."));
  Serial.println(F("'downvel': The max velocity of the profiler on its downwards return in cm/s."));
  Serial.println(F("'bottvel: The velocity of the profiler on the final two meters of the descent in cm/s."));
  Serial.println(F("'depth': The water depth in meters."));
  Serial.println(F("'surfwait': The wait time at the surface in minutes."));
  Serial.println(F("'bottwait': The wait time at the bottom in minutes. AKA, the time between profiles."));
  Serial.println(F("'firstwait': The wait time before beginning the first profile, after this setup is finished, in minutes."));
  Serial.println(F("'all': Get values for all user modifiable variables (only works with 'g'/get)."));
  Serial.println(F(""));
  Serial.println(F("If setting a value, enter a space and the value you wish to set the variable to."));
  Serial.println(F(""));
  Serial.println(F("Wait for any responses to indicate success."));
  Serial.println(F("The serial monitor will then reply to tell you when you can input the next command."));
  Serial.println(F(""));
  Serial.println(F("When finished, enter 'exit' to finish setup and begin profiling operations.\n\n"));
}

void exit() {
  // exits setup loop, begins profiling
  serial_setup = false;
  Serial.print(F("\n"));
  Serial.print(F("Beginning profiling in "));
  Serial.print(first_wait);
  Serial.print(F(" seconds ("));
  Serial.print(first_wait / 60.0);
  Serial.println(F(" minutes). Current settings are:"));
  getVar("all");
  if (!sd_works) {
    Serial.print(F("SD card failed, or not present."));
  }
  Serial.println(F("Variables are saved and will remain until changed."));
  Serial.println(F("Happy profiling! :)"));
}

void readSerialCommand() {
  // receive commands from PC to set user modifiable variables
  // wait for serial input
  while (Serial.available() == 0) {
    delay(100);
  }
  // read serial input
  int idx = 0;
  while (Serial.available() > 0) {
    rec[idx] = Serial.read();
    Serial.print(rec[idx]);
    if (rec[idx] == '\n' || rec[idx] == '\r') {
      // overwrite (remove) newlines and carriage returns
      idx--;
    }
    idx++;
    // overwrite last character if overflowing rec memory
    if (idx >= NUM_CHARS) {
      idx = NUM_CHARS - 1;
    }
    delay(500);
  }
  Serial.println();
  rec[idx] = '\0';  // set end string bit
  idx = 0;          // reset idx
  long val;
  // decode serial input according to format shown in help()
  char *strtokIdx;
  strtokIdx = strtok(rec, " \n\0");
  if (strcmp(strtokIdx, "g") == 0) {
    strtokIdx = strtok(NULL, " ");
    getVar(strtokIdx);
  } else if (strcmp(strtokIdx, "s") == 0) {
    // strtol sets val to 0 if not able to convert to long
    strtokIdx = strtok(NULL, " ");
    //  if (strcmp(strtokIdx, "factor") == 0  || strcmp(strtokIdx, "maxfactor") == 0 || strcmp(strtokIdx, "minfactor") == 0) {
    if (strcmp(strtokIdx, "factor") == 0) {
      // factor is a float so deal with that (only stores first decimal place)
      double temp_val = strtod(strtok(NULL, " "), NULL);
      val = round(temp_val * 10);
    } else {
      // all other variables should be integers
      val = strtol(strtok(NULL, " "), NULL, 10);
    }
    if (val == 0L) {
      // catch incorrect input (counting 0 as incorrect for simplicity)
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

void saveVars() {
  // saves user modifiable variables to memory
  EEPROM.put(RW_ADDRESS, up_vel);
  EEPROM.put(RW_ADDRESS + 4, down_vel);
  EEPROM.put(RW_ADDRESS + 8, water_depth);
  EEPROM.put(RW_ADDRESS + 12, bot_wait);
  EEPROM.put(RW_ADDRESS + 16, first_wait);
  EEPROM.put(RW_ADDRESS + 20, sur_wait);
  EEPROM.put(RW_ADDRESS + 24, bot_vel);
}

void readVars() {
  // reads user modifiable variables from memory
  EEPROM.get(RW_ADDRESS, up_vel);
  EEPROM.get(RW_ADDRESS + 4, down_vel);
  EEPROM.get(RW_ADDRESS + 8, water_depth);
  EEPROM.get(RW_ADDRESS + 12, bot_wait);
  EEPROM.get(RW_ADDRESS + 16, first_wait);
  EEPROM.get(RW_ADDRESS + 20, sur_wait);
  EEPROM.get(RW_ADDRESS + 24, bot_vel);
}

void setup() {
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_REED, INPUT);
  // if LOW, then winch on. HIGH is winch off.
  digitalWrite(PIN_MOTOR, LOW);
  digitalWrite(PIN_BRAKE, LOW);
  delay(10);  // Hear clicks to confirm winch is working
  digitalWrite(PIN_BRAKE, HIGH);
  digitalWrite(PIN_MOTOR, HIGH);

  // initialize serial ports
  Serial.begin(9600);  // for serial with computer
  while (!Serial)
    ;
  Serial3.begin(9600);  // for TX to winch
  while (!Serial3)
    ;
  Serial2.begin(9600);  // for RX from winch
  while (!Serial2)
    ;

  Serial.print(F("\n\n"));
  // clear input buffer
  while (Serial2.available() > 0) {
    Serial2.read();
  }

  // setup real time clock
  if (!rtc.begin()) {
    Serial.flush();
    rtc_works = false;
    Serial.print(F("Couldn't find RTC. "));
  } else {
    Serial.println(F("Realtime clock is ready."));
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, syncing time..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(2000);
  }
  if (RTC_SYNC) {
    // syncs RTC clock with system. Make sure system is set to UTC and is uploaded quickly after compile!
    Serial.println(F("Force syncing RTC..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(2000);
  }

  // setup sd card
  Serial.println(F("Initializing SD card... "));
  if (!SD.begin(SD_CHIP_SELECT)) {
    sd_works = false;
    Serial.println(F("SD card failed, or not present."));
  } else {
    Serial.println(F("SD card initialized."));
  }

  // read previously saved vars from EEPROM
  readVars();
  // setup from serial input
  Serial.println(F("\nPlease enter commands to get or set variables."));
  Serial.println(F("Enter 'help' to display list of available commands."));
  while (serial_setup) {
    Serial.println(F("Ready for next serial command.\n"));
    readSerialCommand();
    delay(1000);
  }
  // save user input variables for next time
  saveVars();

  // first profile delay
  delay(first_wait * 1000);
}

void loop() {
  // setup file name based on date and time
  if (rtc_works) {
    now = rtc.now();
    sprintf_P(file_name, PSTR("%02d%02d%02d%02d.txt"), now.month(), now.day(), now.hour(), now.minute());
  } else {
    sprintf_P(file_name, PSTR("%02d%02d%02d%02d.txt"), 1, 1, 1, 1);
  }
  Serial.print(F("Opening: "));
  Serial.println(file_name);

  File data_file = SD.open(file_name, FILE_WRITE);
  // open file
  if (sd_works) {
    if (data_file) {
      Serial.print(F("Opened: "));
      Serial.println(file_name);
      data_file.print(F("Opened: "));
      data_file.println(file_name);
    }
    if (!data_file) {
      Serial.print(F("Error opening: "));
      Serial.println(file_name);
    }
  }

  Serial.println(F("Winch on."));
  data_file.println(F("Winch on."));
  digitalWrite(PIN_MOTOR, LOW);

  moveDown(data_file, 1);  // start fast
  moveUp(data_file);       // go up quickly
  moveDown(data_file, 1);  // start fast
  moveDown(data_file, 1);  // start fast
  moveDown(data_file, 1);  // start fast
  moveDown(data_file, 1);  // start fast

  digitalWrite(PIN_MOTOR, HIGH);
  Serial.println(F("Winch off."));
  data_file.println(F("Winch off."));

  // log profile count
  data_file.print(F("Profile: "));
  data_file.println(cnt);
  Serial.print(F("Profile: "));
  Serial.println(cnt);
  cnt++;

  // close data file
  data_file.close();
  if (!data_file) {
    Serial.print(F("Closed: "));
    Serial.println(file_name);
  }

  delay(bot_wait * 1000);
  if (cnt >= maxProfiles) {
    for (int k = 1; k < 3600; k++) {
      delay(24000);
    }
  }
}