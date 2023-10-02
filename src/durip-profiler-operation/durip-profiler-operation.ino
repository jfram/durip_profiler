/*
  rapid profiling system (RPS) operation by Andrew Scherer and Jon Fram.
  2023-09-26 edits from jfram begin. Mostly max/min factor related. The other 90%+ of the code is from Andrew in summer 2023.
  Features designed based on Andrew and jfram's discussions, minimal code by jfram from spring 2023, and feedback from Rocky Geyer via jfram. 
  Jim Lerczak, principal investigator
  funded by the Office of Naval Research DURIP program
*/

#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// global variable setup
DateTime now = DateTime();                                              //  
char file_name[20];                                                     //
File data_file;                                                         //
                                                                        //
const long CPCM = 244;                                                  // counts/cm (empirically measured from 100 ft of line payout)
const long SLOW_DEPTH = 2;                                              // slowing down for final 2 meters on move down
int water_depth = 25;                                                   // water depth in meters. This and other parameters are loaded from EPROM, not here.
int depth_factor = 1*10;                                                // factor to multiply by water depth to get actual line payout, multiplied by 10 to allow for one decimal place
int min_depth_factor = 1.0*10;                                          // factor to multiply by water depth to get actual line payout at slack tide, multiplied by 10 to allow for one decimal place
int max_depth_factor = 1.3*10;                                          // factor to multiply by water depth to get actual line payout at max tide, multiplied by 10 to allow for one decimal place
long accel = CPCM*100/10;                                               // 1 m/s^2 accel/decel rate
long up_vel = CPCM*100*10;                                              // 1 m/s upwards velocity
long min_sur_vel = CPCM*100*10;                                         //
long max_sur_vel = CPCM*100*10;                                         //
long sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0;                 // m/s surface velocity
long down_vel = CPCM*25*10;                                             // 0.25 m/s initial down velocity
long bot_vel = CPCM*5*10;                                               // 0.05 m/s final down velocity
long cur_pos = 0;                                                       // stores position when winch turns off
                                                                        //
long inter_sur_pos = water_depth*100*CPCM;                              // move full speed up to water depth, then slow down at surface
long sur_pos = water_depth*(depth_factor/10.0)*100*CPCM;                //
long inter_bot_pos = -water_depth*depth_factor/10*100*CPCM+SLOW_DEPTH*100*CPCM;// move down at down_vel until reaching SLOW_DEPTH above bottom
long bot_pos = -water_depth*depth_factor/10*100*CPCM;                   // last two meters will be descended slower
                                                                        //
long move_status;                                                       //
bool last_move_success = true;                                          //
                                                                        //
long sur_wait = 10;                                                     // surface wait time (seconds)
long bot_wait = 60;                                                     // bottom wait time, time b/t profiles (seconds)
long first_wait = 20;                                                   // time before first profile (seconds)
                                                                        //
RTC_DS3231 rtc;                                                         // real time clock board type
                                                                        //
const byte NUM_CHARS = 32;                                              // maximum number of characters in commands or responses
char cmd[NUM_CHARS];                                                    // commands for sending to winch
char res[NUM_CHARS];                                                    // response from winch
char rec[NUM_CHARS];                                                    // serial input from PC
                                                                        //
const byte PIN_BRAKE = 9;                                               // toggle the Brake
const byte PIN_MOTOR = 8;                                               // toggle the Motor
                                                                        //
int cnt = 0;                                                            // count profiles
bool sd_works = true;                                                   // enables/disables SD card
bool rtc_works = true;                                                  // enables/disables RTC
bool serial_setup = true;                                               // for setting up values via serial
bool latch_fault = false;                                               // record latch faults (not implemented yet)
const bool RTC_SYNC = false;                                            // set to true to force RTC to sync to compile time 
const byte SD_CHIP_SELECT = 53;                                         // SD card chip select pin
const int RW_ADDRESS = 0;                                               // address to read/write user modifiable vars to eeprom
                                                                        //
bool use_tides = true;                                                  // variable to store tidal lookup success/failure
File tide_file;                                                         // file that will open tides.txt from SD card
int searchidx = 0;                                                      // search index that stores line of previously found tide time
double lower_tide_time;                                                 // high/low tide just before now
double upper_tide_time;                                                 // high/low tide just after now
int tide_success;                                                       // return value to debug failures in tidal lookup

void commandWinch(char *cmd, File &data_file) {
    // send command to winch
    Serial3.print(cmd);
    if (cmd != "\n") {
        Serial.print(F("Sent: "));  Serial.print(cmd);
        data_file.print(F("Sent: ")); data_file.print(cmd);
    }
    delay(500); // milliseconds
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

void getPosition (File &data_file) {
    sprintf_P(cmd, PSTR("g r0x32\n"));                                  // send command requesting winch position
    commandWinch(cmd, data_file);                                       // response stored as res
    char *strtokIdx;                                                    //
    strtokIdx = strtok(res, " ");                                       // remove "v" from response
    strtokIdx = strtok(NULL, " ");                                      // get remaining integer from response 
    cur_pos += strtol(strtokIdx, NULL, 10);                             // save current position
    Serial.print(F("Current position (m): "));                          // print current position
    Serial.println((double)cur_pos/((double)CPCM*100.00));              //
    data_file.print(F("Current position (m): "));                       // log current position
    data_file.println((double)cur_pos/((double)CPCM*100.00));           //
    Serial.print(F("Current position (cnts): "));                       // print current position
    Serial.println(cur_pos);                                            //
    data_file.print(F("Current position (cnts): "));                    // log current position
    data_file.println(cur_pos);                                         //
}

void setBotPos() {
    bot_pos = -cur_pos;                                                 // always return to 0 winch position
    if (abs(bot_pos) <= SLOW_DEPTH*100L*CPCM) {                         //
        inter_bot_pos = 0;                                              // proceed only bot_vel if within SLOW_DEPTH above 0 position
    } else {                                                            //
        inter_bot_pos = -cur_pos + SLOW_DEPTH*100L*CPCM;                // begin at full down_vel if over SLOW_DEPTH above 0 position
    }                                                                   //
}

void decodeMoveStatus(long val) {
    // change move status to binary for decoding error codes
    Serial.print(F("Decoded Response: 0b"));
    for (int i = NUM_CHARS; i >= 0; i--) {
        Serial.write(bitRead(val, i) ? '1' : '0');
    }
    int i = NUM_CHARS;
    res[i] = '\0'; 
    i--;
    while (i >= 0) {
        res[i] = (bitRead(val, i) ? '1' : '0');
        i--;
    }
    Serial.println();
}

void eventStatusRegister (File &data_file) {
    sprintf_P(cmd, PSTR("g r0xa0\n"));                                  // send command as normal and get response (stored as res)
    commandWinch(cmd, data_file);                                       //
    char *strtokIdx;                                                    //
    strtokIdx = strtok(res, " ");                                       // remove "v" from response
    strtokIdx = strtok(NULL, " ");                                      // get remaining long from response 
    move_status = strtol(strtokIdx, NULL, 10);                          // set value to move_status
    decodeMoveStatus(move_status);                                      // translate to binary for error checking
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
    // let out line at 1 m/s to water_depth                             //
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), up_vel);                      // set max velocity
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), inter_sur_pos);               // move to surface at water_depth
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("t 1\n"));                                      // initiate move
    commandWinch(cmd, data_file);                                       //  
    long move_time = abs(inter_sur_pos)/(up_vel/(10))*1000;             // expected time to complete move to surface 
    delay(move_time-2000);                                              // subtract 2 seconds from move time to ensure it isn't waiting at surface for cmds
    // let out line slower to depth_factor*water_depth                  //
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), sur_vel);                     // set max velocity
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), sur_pos);                     // let out line from surface to depth_factor*water_depth
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("t 1\n"));                                      // initiate move
    commandWinch(cmd, data_file);                                       //
    move_time = abs(sur_pos-inter_sur_pos)/(sur_vel/(10))*1000;         // expected time to complete move the rest of the way 
    delay(move_time+5000);                                              // add 5 seconds to make sure move is really complete
    eventStatusRegister(data_file);                                     //        
    getPosition(data_file);                                             // get final position of move, save to cur_pos
    setBotPos();                                                        // always return to original position of 0
}

void moveDown (File &data_file) {
    commandWinch("\n", data_file);                                      // newline to clear winch serial buffer
    sprintf_P(cmd, PSTR("s r0xc8 0\n"));                                // set absolute move, trapezoid profile
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0x24 21\n"));                               // set position mode
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcc %ld\n"), accel);                       // set acceleration
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xcd %ld\n"), accel);                       // set deceleration
    commandWinch(cmd, data_file);                                       //
    // faster speed for most of way down                                //
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), down_vel);                    // set max velocity
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), inter_bot_pos);               // set final position of down move 
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("t 1\n"));                                      // initiate move
    commandWinch(cmd, data_file);                                       //
    long move_time = abs(inter_bot_pos)/(down_vel/(10))*1000;           // expected time to complete move down 
    delay(move_time-2000);                                              //
    eventStatusRegister(data_file);                                     //
    // slow down in final 2 meters
    sprintf_P(cmd, PSTR("s r0xcb %ld\n"), bot_vel);                     // set max velocity
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("s r0xca %ld\n"), bot_pos);                     // final 2 meters of movement
    commandWinch(cmd, data_file);                                       //
    sprintf_P(cmd, PSTR("t 1\n"));                                      // initiate move
    commandWinch(cmd, data_file);                                       //
    move_time = abs(bot_pos - inter_bot_pos)/(bot_vel/(10))*1000;       // expected time to complete move down 
    delay(move_time+5000);                                              //
    eventStatusRegister(data_file);                                     // 
    getPosition(data_file);                                             // get final position of move, update cur_pos                                                             
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

void getVar(char *c, bool all=false) {
    // retrieve current user-modifiable variables
    if (strcmp(c, "upvel") == 0) {
        Serial.print(F("Up velocity (cm/s): "));
        Serial.println(up_vel/(CPCM*10L));    
    } else if (strcmp(c, "downvel") == 0) {
        Serial.print(F("Down velocity (cm/s): "));
        Serial.println(down_vel/(CPCM*10L));
    } else if (strcmp(c, "depth") == 0) {
        Serial.print(F("Water depth (m): "));
        Serial.println(water_depth);
        if (!all) {
            Serial.print(F("Depth factor: "));
            Serial.println((double)depth_factor/10.0);
            Serial.print(F("Total line payout (m): "));
            Serial.println(water_depth*(double)depth_factor/10.0);
        }
    } else if (strcmp(c, "surfwait") == 0) {
        Serial.print(F("Surface wait time (min): "));
        Serial.println(sur_wait/60);    
    } else if (strcmp(c, "bottwait") == 0) {
        Serial.print(F("Bottom wait time (min): "));
        Serial.println(bot_wait/60);
    } else if (strcmp(c, "firstwait") == 0) {
        Serial.print(F("First profile wait time (min): "));
        Serial.println(first_wait/60);
    } else if (strcmp(c, "factor") == 0) {
        if (!all) {
            Serial.print(F("Water depth (m): "));
            Serial.println(water_depth);
        }
        Serial.print(F("Depth factor: "));
        Serial.println((double)depth_factor/10.0);
        Serial.print(F("Total line payout (m): "));
        Serial.println(water_depth*(double)depth_factor/10.0);
    } else if (strcmp(c, "maxfactor") == 0) {
        Serial.print(F("Max depth factor: "));
        Serial.println((double)max_depth_factor/10.0);
    } else if (strcmp(c, "minfactor") == 0) {
        Serial.print(F("Min depth factor: "));
        Serial.println((double)min_depth_factor/10.0);
    } else if (strcmp(c, "bottvel") == 0) {
        Serial.print(F("Bottom velocity (cm/s): "));
        Serial.println(bot_vel/(CPCM*10L));
    } else if (strcmp(c, "minsurfvel") == 0) {
        Serial.print(F("Minimum surface velocity (cm/s): "));
        Serial.println(min_sur_vel/(CPCM*10));
        if (!all) {
            Serial.print(F("Default surface velocity (cm/s): "));
            Serial.println(sur_vel/(CPCM*10));
        }
    } else if (strcmp(c, "maxsurfvel") == 0) {
        Serial.print(F("Maximum surface velocity (cm/s): "));
        Serial.println(max_sur_vel/(CPCM*10));
        if (!all) {
            Serial.print(F("Default surface velocity (cm/s): "));
            Serial.println(sur_vel/(CPCM*10));
        }
    } else if (strcmp(c, "defsurfvel") == 0) {
        Serial.print(F("Default surface velocity (cm/s): "));
        Serial.println(sur_vel/(CPCM*10));
    } else if (strcmp(c, "all") == 0) {
        getVar("upvel");
        getVar("minsurfvel", true);
        getVar("maxsurfvel", true);
        getVar("defsurfvel");
        getVar("downvel");
        getVar("bottvel");
        getVar("depth", true);
        getVar("factor", true);
        getVar("minfactor", true);
        getVar("maxfactor", true);
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
        up_vel = val*CPCM*10L;
        getVar("upvel");
    } else if (strcmp(c, "downvel") == 0) {
        down_vel = val*CPCM*10L;
        getVar("downvel");
    } else if (strcmp(c, "depth") == 0) {
        water_depth = val;
        inter_bot_pos = -water_depth*depth_factor/10L*100L*CPCM+200L*CPCM;
        bot_pos = -water_depth*depth_factor/10L*100L*CPCM;
        inter_sur_pos = water_depth*100L*CPCM;
        sur_pos = water_depth*depth_factor/10L*100L*CPCM;
        getVar("depth", true);
        getVar("factor", true);
    } else if (strcmp(c, "surfwait") == 0) {
        sur_wait = val*60;
        getVar("surfwait");
    } else if (strcmp(c, "bottwait") == 0) {
        bot_wait = val*60;
        getVar("bottwait");
    } else if (strcmp(c, "firstwait") == 0) {
        first_wait = val*60;
        getVar("firstwait");
    } else if (strcmp(c, "factor") == 0) {
        if (val < 10) {
            Serial.println(F("Depth factor must be greater than or equal to one - no changes were made."));
            getVar("depth", true);
            getVar("factor", true);
        } else if (val >= 10) {
            depth_factor = val;
            inter_bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM+200*CPCM;
            bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM;
            inter_sur_pos = water_depth*100*CPCM;
            sur_pos = water_depth*(depth_factor/10.0)*100*CPCM;
            getVar("depth", true);
            getVar("factor", true);
        }
    } else if (strcmp(c, "maxfactor") == 0) {
        max_depth_factor = val;
        getVar("maxfactor");
    } else if (strcmp(c, "minfactor") == 0) {
        min_depth_factor = val;
        getVar("minfactor");
    } else if (strcmp(c, "minsurfvel") == 0) {
        if (val*CPCM*10 > max_sur_vel) {
            Serial.println(F("Minimum surface velocity must be less than maximum surface velocity. No action taken."));
            getVar("maxsurfvel", true);
        } else if (val*CPCM*10 <= max_sur_vel) {
            min_sur_vel = val*CPCM*10L;
            sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0;
        }
        getVar("minsurfvel");
    } else if (strcmp(c, "maxsurfvel") == 0) {
        if (val*CPCM*10 < min_sur_vel) {
            Serial.println(F("Maximum surface velocity must be greater than minimum surface velocity. No action taken."));
            getVar("minsurfvel", true);
        } else if (val*CPCM*10 >= min_sur_vel) {
            max_sur_vel = val*CPCM*10L;
            sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0;     
        }
        getVar("maxsurfvel");
    } else if (strcmp(c, "bottvel") == 0) {
        bot_vel = val*CPCM*10L;
        getVar("bottvel");
    } else {
        Serial.println(F("Unrecognized variable name - no action taken."));
    }
}

void help() {
    // explains how to use serial interface
    Serial.println(F("Begin line with 'g' to get current values or 's' to set values."));
    Serial.println("");
    Serial.println(F("Enter a space then the variable name you wish to get or set. Options are:"));
    Serial.println(F("'upvel': The max velocity of the profiler on its upwards profile in cm/s."));
    Serial.println(F("'maxsurfvel': The maximum velocity in cm/s of line payout after the profiler hits the surface based on set depth."));
    Serial.println(F("'minsurfvel': The minimum velocity in cm/s of line payout after the profiler hits the surface based on set depth."));
    Serial.println(F("'defsurfvel': The default surface velocity in cm/s, taken as the average of maxsurfvel and minsurfvel. Cannot be set directly - must change maxsurfvel or minsurfvel to update defsurfvel."));
    Serial.println(F("'downvel': The max velocity of the profiler on its downwards return in cm/s."));
    Serial.println(F("'bottvel: The velocity of the profiler on the final two meters of the descent in cm/s."));
    Serial.println(F("'depth': The water depth in meters."));
    Serial.println(F("'factor': The water depth is multiplied by this factor. This is how much line is paid out. Must be greater than or equal to one."));
    Serial.println(F("'minfactor': The water depth is multiplied by this factor. This is how much line is paid out at slack tide. Must be greater than or equal to one."));
    Serial.println(F("'maxfactor': The water depth is multiplied by this factor. This is how much line is paid out at max tide. Must be greater than or equal to one."));
    Serial.println(F("'surfwait': The wait time at the surface in minutes."));
    Serial.println(F("'bottwait': The wait time at the bottom in minutes. AKA, the time between profiles."));
    Serial.println(F("'firstwait': The wait time before beginning the first profile, after this setup is finished, in minutes."));
    Serial.println(F("'all': Get values for all user modifiable variables (only works with 'g'/get)."));
    Serial.println("");
    Serial.println(F("If setting a value, enter a space and the value you wish to set the variable to."));
    Serial.println("");
    Serial.println(F("Wait for any responses to indicate success."));
    Serial.println(F("The serial monitor will then reply to tell you when you can input the next command."));
    Serial.println("");
    Serial.println(F("When finished, enter 'exit' to finish setup and begin profiling operations.\n\n"));
} 

void exit() {
    // exits setup loop, begins profiling
    serial_setup = false;
    Serial.print(F("\n"));
    Serial.print(F("Beginning profiling in "));
    Serial.print(first_wait);
    Serial.print(F(" seconds ("));
    Serial.print(first_wait/60.0);
    Serial.println(F(" minutes). Current settings are:"));
    getVar("all");
    if (!sd_works) {
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        depth_factor = (int)(max_depth_factor + min_depth_factor)/2.0;
        Serial.print(F("Since SD card failed, or not present, using default surface velocity (cm/s): "));
        Serial.println(sur_vel/(CPCM*10));
        Serial.print(F("Since SD card failed, or not present, using default depth factor: "));
        Serial.println((double)depth_factor/10);        
    }
    Serial.println(F("Variables are saved and will remain until changed."));
    Serial.println(F("Happy profiling! :)"));
}

void readSerialCommand () {
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
    rec[idx] = '\0';    // set end string bit
    idx = 0;            // reset idx
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
        if (strcmp(strtokIdx, "factor") == 0  || strcmp(strtokIdx, "maxfactor") == 0 || strcmp(strtokIdx, "minfactor") == 0) {
            // factor is a float so deal with that (only stores first decimal place)
            double temp_val = strtod(strtok(NULL, " "), NULL);
            val = round(temp_val*10);
        } else{
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
    EEPROM.put(RW_ADDRESS + 12, depth_factor);
    EEPROM.put(RW_ADDRESS + 16, bot_wait);
    EEPROM.put(RW_ADDRESS + 20, first_wait);
    EEPROM.put(RW_ADDRESS + 24, sur_wait);
    EEPROM.put(RW_ADDRESS + 28, bot_vel);
    EEPROM.put(RW_ADDRESS + 32, min_sur_vel);
    EEPROM.put(RW_ADDRESS + 36, max_sur_vel);
    EEPROM.put(RW_ADDRESS + 40, min_depth_factor);
    EEPROM.put(RW_ADDRESS + 44, max_depth_factor);
    sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
}

void readVars() {
    // reads user modifiable variables from memory
    EEPROM.get(RW_ADDRESS, up_vel);
    EEPROM.get(RW_ADDRESS + 4, down_vel);
    EEPROM.get(RW_ADDRESS + 8, water_depth);
    EEPROM.get(RW_ADDRESS + 12, depth_factor);
    inter_sur_pos = water_depth*100*CPCM;                             
    sur_pos = water_depth*(depth_factor/10.0)*100*CPCM;                   
    inter_bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM+SLOW_DEPTH*100*CPCM;
    bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM;  
    EEPROM.get(RW_ADDRESS + 16, bot_wait);
    EEPROM.get(RW_ADDRESS + 20, first_wait);
    EEPROM.get(RW_ADDRESS + 24, sur_wait);
    EEPROM.get(RW_ADDRESS + 28, bot_vel);
    EEPROM.get(RW_ADDRESS + 32, min_sur_vel);
    EEPROM.get(RW_ADDRESS + 36, max_sur_vel);
    EEPROM.get(RW_ADDRESS + 40, min_depth_factor);
    EEPROM.get(RW_ADDRESS + 44, max_depth_factor);
    sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
}

int findClosestTideTime (long now_time, File &tide_file) {
    /* 
    returns index for first tide time after now and saves search index for future use
    must use format produced by tide_table_downloader to work, assumes constant line length of 12 characters
    returns special values for fault conditions, resets search index and checks again in case RTC can fix itself
    function will break in ~2300 when unix time becomes 10^11
    */
    long lines = tide_file.size()/12;
    double tide_time;
    int i = searchidx;
    tide_file.seek(i*12);
    char buffer[12];
    while (true) {
        // read twelve characters
        for (int j = 0; j < 12; j++) {
            buffer[j] = tide_file.read();
        }
        // stop searching if last entry in buffer is not new line, formatting is incorrect
        if (buffer[11] != '\n') {
            searchidx = 0;
            return -3;
        }
        tide_time = strtod(buffer, NULL);
        // stop searching if strtod returns 0, formatting is incorrect
        if (tide_time == 0) {
            searchidx = 0;
            return -3;
        }
        if (tide_time > now_time) {
            break;
        }
        i++;
        tide_file.seek(i*12);
        // stop searching at end of file as all times are before now, reset to index to zero to try again next profile
        if (i >= lines) {
            searchidx = 0;
            return -1;
        }
    }
    // if search returns zeroeth index all tide times are after now, reset index to 0 and try again next profile
    if (i == 0) {
        searchidx = 0;
        return -2;
    }
    // if value is found successfully, save it, and save search index for next time
    searchidx = i;
    upper_tide_time = tide_time;
    // get the tide time just before now for linear regression
    tide_file.seek((i-1)*12);
    for (int j = 0; j < 12; j++) {
        buffer[j] = tide_file.read();
    }
    lower_tide_time = strtol(buffer, NULL, 10);
    // i will be strictly greater than zero if successful
    return i;
}

void getTidalState (File &tide_file) {
    /*
    set surface velocity based on time between high/low tides
    also handle faults by setting surface velocity to default (avg of max/min)
    */
    double seconds = now.unixtime();
    tide_success = findClosestTideTime(seconds, tide_file);
    // all times are after now
    if (tide_success == -1) {
        use_tides = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        depth_factor = (int)(max_depth_factor + min_depth_factor)/2.0; // FRAM
    // all times are before now (also catches return 0 in case if statement in findClosest doesn't trigger)
    } else if (tide_success == -2 || tide_success == 0) {
        use_tides = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        depth_factor = (int)(max_depth_factor + min_depth_factor)/2.0; // FRAM
    // incorrect tide file format
    } else if (tide_success == -3) {
        use_tides = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        depth_factor = (int)(max_depth_factor + min_depth_factor)/2.0; // FRAM
    // success - compute tidal state based on linear regression between high and low tide times, using abs value function
    } else if (tide_success > 0) {
        double slope = -2.0*(double)(max_sur_vel - min_sur_vel)/(upper_tide_time - lower_tide_time);
        double peak_flow = (upper_tide_time + lower_tide_time)/2.0;
        sur_vel = slope*fabs(seconds - peak_flow) + (double)max_sur_vel;
        slope = -2.0*(double)(max_depth_factor - min_depth_factor)/(upper_tide_time - lower_tide_time); // FRAM
        depth_factor = (int)(slope*fabs(seconds - peak_flow)) + max_depth_factor;  // FRAM
    // catch any other unknown states
    } else {
        use_tides = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        depth_factor = (int)(max_depth_factor + min_depth_factor)/2.0; // FRAM
    }
    inter_bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM+200*CPCM; // FRAM
    bot_pos = -water_depth*(depth_factor/10.0)*100*CPCM; // FRAM
    inter_sur_pos = water_depth*100*CPCM;// FRAM
    sur_pos = water_depth*(depth_factor/10.0)*100*CPCM; // FRAM
}

void printTidalState(File &data_file) {
    // records results of getTidalState since can't have two files open on SD card at same time and prints surface velocity used
    // all times are after now
    if (tide_success == -1) {
        Serial.print(F("All tide table values are before current time. Using default surface velocity (cm/s): "));
        Serial.println(sur_vel/(double)(CPCM*10));
        data_file.print(F("All tide table values are before current time. Using default surface velocity (cm/s): "));
        data_file.println(sur_vel/(double)(CPCM*10));
    // all times are before now (also catches return 0 in case if statement in findClosest doesn't trigger)
    } else if (tide_success == -2 || tide_success == 0) {
        Serial.print(F("All tide table values are after current time. Using default surface velocity (cm/s): "));
        Serial.println(sur_vel/(double)(CPCM*10));
        data_file.print(F("All tide table values are after current time. Using default surface velocity (cm/s): "));
        data_file.println(sur_vel/(double)(CPCM*10));
    // incorrect tide file format
    } else if (tide_success == -3) {
        Serial.print(F("Tide table formatting incorrect. Using default surface velocity (cm/s): "));
        Serial.println(sur_vel/(double)(CPCM*10));
        data_file.print(F("Tide table formatting incorrect. Using default surface velocity (cm/s): "));
        data_file.println(sur_vel/(double)(CPCM*10));
    // success
    } else if (tide_success > 0) {
        Serial.print(F("Tide table lookup succeeded! Using surface velocity (cm/s): "));
        Serial.println(sur_vel/(double)(CPCM*10));
        data_file.print(F("Tide table lookup succeeded! Using surface velocity (cm/s): "));
        data_file.println(sur_vel/(double)(CPCM*10));
    // catch any other unknown states
    } else {
        Serial.print(F("Unknown tide table error. Using default surface velocity."));
        Serial.println(sur_vel/(double)(CPCM*10));
        data_file.print(F("Unknown tide table error. Using default surface velocity."));
        data_file.println(sur_vel/(double)(CPCM*10));
    }
    Serial.print(F("Depth factor: "));
    Serial.println((double)depth_factor/10.0);
    data_file.print(F("Depth factor: "));
    data_file.println((double)depth_factor/10.0);
}

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

    Serial.print(F("\n\n"));
    // clear input buffer 
    while (Serial2.available() > 0) {
        Serial2.read();
    }

    // setup real time clock
    if (! rtc.begin()) {
        Serial.flush();
        rtc_works = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
        Serial.print(F("Couldn't find RTC. Using default surface velocity (cm/s): "));
        Serial.println(sur_vel);
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
        use_tides = false;
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
        Serial.println("Ready for next serial command.\n");
        readSerialCommand();
        delay(1000);
    }
    // save user input variables for next time
    saveVars();

    if (water_depth <= SLOW_DEPTH) {
        // if water depth is less than SLOW_DEPTH, prevent positive inter_bot_pos and move slowly the whole way down
        inter_bot_pos = 0;                  
    }

    // winch off until first profile
    digitalWrite(PIN_BRAKE, HIGH);
    digitalWrite(PIN_MOTOR, HIGH); 

    // first profile delay 
    delay(first_wait*1000);
}

void loop () {

    // setup file name based on date and time
    if (rtc_works) {
        now = rtc.now();
        sprintf_P(file_name, PSTR("%02d%02d%02d%02d.txt"), now.month(), now.day(), now.hour(), now.minute());
        Serial.print(F("Opening: "));
        Serial.println(file_name);
    } else {
        use_tides = false;
        sur_vel = (double)(max_sur_vel + min_sur_vel)/2.0; 
    }
    Serial.print(sd_works);
    if ((use_tides) && (sd_works)) {
        // compute surface velocity based on tides if RTC and SD are working
        File tide_file = SD.open("tides.txt");
        if (tide_file) {
            getTidalState(tide_file);
            tide_file.close();
        } else if (!tide_file) {  
            Serial.print(F("Error opening: ")); Serial.println(tide_file);
        }

    }

    File data_file = SD.open(file_name, FILE_WRITE); 
    // open file
    if (sd_works) {
        if (data_file) {
            Serial.print(F("Opened: ")); Serial.println(file_name); 
            data_file.print(F("Opened: ")); data_file.println(file_name);
        }   
        if (!data_file) {
            Serial.print(F("Error opening: ")); Serial.println(file_name);
        }   
    }

    // print timestamp into file and serial monitor
    if (rtc_works) {
        now=rtc.now();
        Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        data_file.println(now.timestamp(DateTime::TIMESTAMP_FULL));
    } 

    if (use_tides && sd_works) {
        // print tidal success and surface velocity 
        printTidalState(data_file);
    }

    // get voltage supplied to arduino board
    data_file.print(F("Arduino board voltage (V): "));
    data_file.println(readVcc());

    // power winch motor and brake
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);
    Serial.println(F("Winch on."));
    data_file.println(F("Winch on."));
    delay(5000);
    commandWinch("\n", data_file);    

    // do the profile up
    moveUp(data_file);                         

    // winch off at surface
    digitalWrite(PIN_BRAKE, HIGH);
    digitalWrite(PIN_MOTOR, HIGH);
    Serial.println(F("Winch off."));
    data_file.println(F("Winch off."));
    
    // surface delay 
    delay(sur_wait*1000);

    // winch on for down move
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR, LOW);
    Serial.println(F("Winch on."));
    data_file.println(F("Winch on."));
    delay(5000);
    commandWinch("\n", data_file); 

    // do the move down, slowing for the last SLOW_DEPTH
    moveDown(data_file);                    

    // winch off at bottom
    digitalWrite(PIN_BRAKE, HIGH);
    digitalWrite(PIN_MOTOR, HIGH);
    Serial.println(F("Winch off."));
    data_file.println(F("Winch off."));

    // log profile count
    data_file.print(F("Profile: "));
    data_file.println(cnt);
    cnt ++;

    // close data file
    data_file.close(); 
    if (!data_file) {
        Serial.print(F("Closed: ")); Serial.println(file_name);
    } 

    // reset use tides to try again next loop
    use_tides = true; 
    
    // bottom delay between profiles
    delay(bot_wait*1000);    
}
