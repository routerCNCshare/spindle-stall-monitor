// include libraries
#include <Arduino.h>
#include <LiquidCrystal.h>  // LCD display library
#include <Preferences.h>    // preferences library (non-volatile memory to store user variables)

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(18, 5, 19, 21, 22, 23);  // RS=18, Enable=5, D4=19, D5=21, D6=22, D7=23, VSS=GND, VDD > 5V   < ESP32 board #3

// version record
//V15.  Added average rpm calculations to smooth the readings
//V16.  Added basic menu functions
//V17.  Added improved menu functions
//V18.  Tuning for estop, better default settings, better algorithm
//V19.  Improve e-stop logic
//V20.  Menu tidied.  Factory restore feature added.
//V21.  Initial code for storing user variables in non-volatile memory
//V22.  Stable code for storing user variables in non-volatile memory
//V23.  New prototype board, split main and LCD.  New user key values.  More stable VFD readings.
//V24.  Setup to use Hall Effect Sensor (instead of IR sensor)
//V25.  Some further improvements
//V26.  Tidying up the code, cleaning up menu options, and making it e-stop quicker
//V27.  More tidying up the code, better load sensing display, more tuning
//V28.  Key input values set for 5V (rather than USB 4.88V)

// version for display
const int software_version = 29;



///////////////////////////////////////////////////////////////////////////
//              Configure processor interface pins and user keys         //
///////////////////////////////////////////////////////////////////////////
// assign pins
const int IR_SENSOR_PIN = 34;      // define the pin where the IR sensor is connected
const int BUTTON_PIN_A0 = 15;      // read button interface:  nothing = 4096, sel= 3580, left= 2150 , right= 0,  up= 490 , down= 1330
const int BUTTON_TOLERANCE = 100;  // +- range to look for button press
const int VFD_PWM_PIN = 35;        // define pin for reading PFD PWM signal
const int IR_MONITOR_PIN = 32;     // recreate PWM signal as monitor output
const int ESTOP_RELAY_PIN = 4;     // define pin to control e-stop relay
const int ESTOP_LED_PIN = 2;       // define pin for relay LED
const int VFD_RPM_PIN = 25;        // DAC analog output of VFD RPM
const int RPM_PIN = 26;            // DAC analog output of spindle RPM

// declare user variables for key presses
// set for 5V input (scale if supply voltage is less)
double read_user_pin = 0;  // analogue value for key presses
int system_state = 0;      // store system state: 0 = unknown, 1 = idle, 2= accelerating, 3 = decelerating, 4 = running, 8 = feed hold, 9 = e-stop
int KEY_UP = 360;
int KEY_DOWN = 1140;
int KEY_LEFT = 1892;
int KEY_RIGHT = 0;
int KEY_SELECT = 3071;
int KEY_NO = 4095;         // 5V full range






///////////////////////////////////////////////////////////////////////////////////
//                           declare menu structure                              //
///////////////////////////////////////////////////////////////////////////////////
//
struct menu_structure {
  String menu_label;       // field #1 (name)
  int menu_value;          // field #2 (user value)
  int menu_inc_dec;        // field #3 (inc or dec amount)
  int menu_value_max;      // field #4 (max value allowed)
  int menu_value_min;      // field #5 (min value allowed)
  int menu_factory_value;  // field #6 (factory reset value)
};
// Create an array of Menu_Structure defined structures containing factory reset values (text", user value, inc/dec step, max value, min value, factory default)
menu_structure menu[] = {
  // spindle sampling for rpm calculation and follow error (lower value = less follow error allowed = more sensitivity)
  { "1:SampleTime(ms)", 250, 5, 2000, 25, 250 },  // menu item #0
  { "2:FollowError%", 20, 5, 100, 5, 20 },        // menu item #1
  // rpm calculation fine tuning
  { "3:SpinRpmScale", 100, 1, 150, 10, 100 },  // menu item #2
  { "4:VFD RpmScale", 100, 1, 150, 10, 100 },  // menu item #3
  // parameters to determine 'run', 'accel', or 'decel'
  { "5:RpmDecelThresh", -400, 5, -50, -1000, -400 },  // menu item #4
  { "6:RpmAccelThresh", 400, 5, 1000, 50, 400 },      // menu item #5
  { "7: Menu end", 0, 0, 0, 0, 0 }                    // menu item #6
};
const int number_of_menu_rows = sizeof(menu) / sizeof(menu[0]);
// function prototype declaration
void menu_subroutine(menu_structure menu[], int menu_size);
///////////////////////////////////////////////////////////////////////////////////

// Set up preferences library (where user data is stored)
Preferences preferences;





///////////////////////////////////////////////////////////////////////////////////
//                           declare variables                                   //
///////////////////////////////////////////////////////////////////////////////////
// IR sensor pulse counter from spindle
double spindle_pulse_analog = 0;      // read analog state of spindle input from Hall sensor op amp (then invert it)
int spindle_pulse_state = 0;          // read IR_Sensor pulse state
int spindle_last_pulse_state = 0;     // store previous pulse state
unsigned long spindle_last_time = 0;  // store previous spindle time
double spindle_debounce_time = 0;     // spindle pulse debounce time in milliseconds

// spindle rpm calculation
double pulse_count = 0;         // count of IR sensor pulses
unsigned long Master_Time = 0;  // current time in milliseconds for tracking sampling intervals
double rpm = 0;                 // calculated rpm
double rounded_rpm = 0;
double rpm_last_measurement = 0;  // previous rpm measurement to help calculate acceleration
double rpm_last_time = 0;         // last time the RPM was calculated
double min_rpm = 500;             // minimum rpm to calculate system state (assume not running if rpm below this value)
double pulse_per_rpm = 2;         // number of pulses per rpm (rising and falling pulses are both counted, so use 2 for a single white strip)
bool system_active = false;       // boolean flag to show if stall system is active.  Off at low rpm or if spindle has not started yet.

// user defined variables (stored in memory)
double rpm_sample_time = 250;            // how long to sample IR sensor pulses (in milliseconds)
double delta_rpm_stall = 20;             // min % of actual Vs demand rpm.  If actual rpm is less than this % of demand, then may be stalling.
double spindle_scale = 100;              // scaling to fine tune spindle rpm value
double PWM_scale = 100;                  // scale to fine tune PWM to RPM mapping
double acceleration_upper_limit = 400;   // acceleration upper trigger limit
double acceleration_lower_limit = -400;  // acceleration lower trigger limit

// spindle acceleration calculation
double acceleration = 0;  // current acceleration

// VFD PWM variables
int vfd_PWM_signal = 0;            // VFD PWM signal value
int vfd_PWM_signal_last_time = 0;  // store PMW signal from last time
double vfd_PWM_rpm = 0;            // VFD PWM converted to rpm
double rounded_vfd_PWM_rpm = 0;

// PWM duty cycle variables
double highTime = 0;              // counter in microseconds for how long in HIGH state
double lowTime = 0;               // counter in microseconds for how long in HIGH state
double lastTime = 0;              // record last time to work out delta time
double totalTime = 0;             // store total time for sum of high and low times
int PWM_pulse_state = 0;          // 0 = low, 1= high flip flop
double dutyCycle = 0;             // calculate duty cycle 0 - 100%
static double lastPrintTime = 0;  // how long since last display and calculation update
double currentTime = 0;           // store current time at this moment


// load meter variables
double delta_rpm = 0;        // difference between actual rpm and VFD demand rpm
double load_meter_bars = 0;  // how many load bars to display on the LCD (depending on following error %)
byte FullSquare[8] = {       // custom graphic for load bar
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte LoadMeter[8] = {  // custom graphic for load bar outline
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111
};






////////////////////////////////////////////////////////////////////////////
//                              set up systems                            //
////////////////////////////////////////////////////////////////////////////
void setup() {

  // LCD display
  lcd.begin(16, 2);             // columns and rows
  lcd.clear();                  // clear LCD screen and reset cursor to 0,0
  lcd.setCursor(0, 0);          // set cursor top of display
  lcd.print("SpindleGuard");    // display splash message with version number
  lcd.setCursor(0, 1);          // display splash message with version number
  lcd.print("V");               // display splash message with version number
  lcd.print(software_version);  // display splash message with version number
  delay(1000);                  // wait 1 second to allow user to read splash message
  lcd.clear();                  // clear LCD screen and reset cursor to 0,0
  // create a new LCD characters
  lcd.createChar(0, FullSquare);
  lcd.createChar(1, LoadMeter);

  // pin configuration
  pinMode(IR_SENSOR_PIN, INPUT);       // Set up the IR sensor pin as input
  pinMode(IR_MONITOR_PIN, OUTPUT);     // monitor output pin to check PWM triggering correctly
  digitalWrite(IR_MONITOR_PIN, LOW);   // set monitor pin LOW as start point
  pinMode(BUTTON_PIN_A0, INPUT);       // set up the pin to read keypad button presses
  pinMode(VFD_PWM_PIN, INPUT);         // read VFD PWM signal
  pinMode(ESTOP_RELAY_PIN, OUTPUT);    // relay pin defined as output
  digitalWrite(ESTOP_RELAY_PIN, LOW);  // ensure estop relay control pin is not active when starting
  pinMode(ESTOP_LED_PIN, OUTPUT);      // declare pin2 as output for relay LED
  digitalWrite(ESTOP_LED_PIN, LOW);    // ensure relay LED off at start






  ///////////////////////////////////////////////////////////////////////////////////
  // check if factory reset required - is user holding down 'select/menu' key?     //
  ///////////////////////////////////////////////////////////////////////////////////

  preferences.begin("user_data", false);                                                                           // Open Preferences with namespace "user_data"
  read_user_pin = analogRead(BUTTON_PIN_A0);                                                                       // read button press value
  if (((read_user_pin < (KEY_SELECT + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_SELECT - BUTTON_TOLERANCE)))) {  // is user pressing select/menu ?

    // pressed menu/select button so start factory reset process
    lcd.setCursor(0, 0);                                                           // move cursor
    lcd.print("**FACTORY RESET**");                                                // show message
    read_user_pin = analogRead(BUTTON_PIN_A0);                                     // read button state
    while (read_user_pin < KEY_NO) { read_user_pin = analogRead(BUTTON_PIN_A0); }  // wait for button to be released
    for (int i = 0; i <= number_of_menu_rows; i++) {                               // loop through and reset array values to factory
      menu[i].menu_value = menu[i].menu_factory_value;                             // load user values from FACTORY into 'value' field of array structure
    }                                                                              // loop back
    preferences.clear();                                                           // clear all values in current preferences area in non-volatile memory
    saveUserData();                                                                // store FACTORY values into non-volatile memory (overwriting user values)

    // display 'progress bar' (really just showing user that something has happened)
    lcd.setCursor(0, 1);
    for (int i = 1; i <= 15; i++) {
      lcd.write(byte(0));  // Display the full square character
      delay(125);
    }
    lcd.clear();  // clear LCD screen
  } else

    ////////////////////////////////////////////////////
    // otherwise load user stored values from memory  //
    ////////////////////////////////////////////////////
    // check if preferences library exists (1st ever run will not have a preference library stored)
    if (preferences.getInt("key_value_0", -1) != -1) {
      // load preference library into array structure
      lcd.setCursor(0, 0);            // move cursor
      lcd.print("load user values");  // display user message
      loadUserData();                 // load values from non-volatile memory into array structure containing user values

      // display 'progress bar' (really just showing user that something has happened)
      lcd.setCursor(0, 1);
      for (int i = 1; i <= 15; i++) {
        lcd.write(byte(0));  // Display the full square character
        delay(125);
      }
      lcd.clear();  // clear LCD screen
    } else {
      // no preferences library set up so load defaults (actually does nothing, so firmware factory values are retained)
      lcd.setCursor(0, 0);         // move cursor
      lcd.print("load defaults");  // display user message

      // display 'progress bar' (really just showing user that something has happened)
      lcd.setCursor(0, 1);
      for (int i = 1; i <= 15; i++) {
        lcd.write(byte(0));  // Display the full square character
        delay(125);
      }
      lcd.clear();  // clear LCD screen
    }

  // load values from array structure into main variables
  rpm_sample_time = menu[0].menu_value;           // rpm sample time in milliseconds
  delta_rpm_stall = menu[1].menu_value;           // threshold as % of demand rpm for stall monitoring (e.g. 70% means if actual rpm < 70% of demand rpm system might be stalling)
  spindle_scale = menu[2].menu_value;             // scaling factor to adjust spindle rpm calculation
  PWM_scale = menu[3].menu_value;                 // scaling factor to adjust rpm calculation for VFD demand rpm
  acceleration_lower_limit = menu[4].menu_value;  // lower rpm limit to display "accel" on LCD
  acceleration_upper_limit = menu[5].menu_value;  // upper rpm limit to display "decel" on LCD



  /////////////////////////////////////////////////////////////////////////////
  //                       standby until key pressed                         //
  /////////////////////////////////////////////////////////////////////////////
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("*****offline*****");
  lcd.setCursor(0, 1);
  lcd.print(">SELECT");
  read_user_pin = analogRead(BUTTON_PIN_A0);  // read key press value
  while ((read_user_pin > (KEY_SELECT + 20)) || (read_user_pin < (KEY_SELECT - 20))) {
    read_user_pin = analogRead(BUTTON_PIN_A0);
    lcd.setCursor(12, 1);
    lcd.print(read_user_pin);
    lcd.print("   ");
  }             // read button press value
  lcd.clear();  // clear display
  delay(500);   // wait for key to be released
}




/////////////////////////////////////////////////////////////////////////////
//                                main loop                                //
/////////////////////////////////////////////////////////////////////////////
void loop() {

  //////////////////////////////////////////
  //               VFD PWM counter        //
  //////////////////////////////////////////

  // Calculate and print duty cycle every second
  vfd_PWM_signal = digitalRead(VFD_PWM_PIN);
  if (vfd_PWM_signal == HIGH && vfd_PWM_signal_last_time == LOW) {
    // HIGH PULSE, accumulate high time
    PWM_pulse_state = 1;                        // set pulse state to HIGH
    vfd_PWM_signal_last_time = vfd_PWM_signal;  // record pulse state to allow checking for change of state

  } else if (vfd_PWM_signal == LOW && vfd_PWM_signal_last_time == HIGH) {
    // LOW PULSE, accumulate low time
    PWM_pulse_state = 0;                        // set pulse state to LOW
    vfd_PWM_signal_last_time = vfd_PWM_signal;  // record pulse state to allow checking for change of state
  }
  // accumulate either high or low time depending on PWM state
  currentTime = micros();  // store current time
  if (PWM_pulse_state == 1) {
    highTime += currentTime - lastTime;  // accumulate high pulse time since last time
  } else {
    lowTime += currentTime - lastTime;  // accumulate low pulse time since last time
  }
  lastTime = currentTime;  // store new time




  ///////////////////////////////////////////
  //        spindle IR rpm pulse counter   //
  ///////////////////////////////////////////

  spindle_pulse_state = digitalRead(IR_SENSOR_PIN);                                                                      // read IR sensor in
  if ((spindle_pulse_state != spindle_last_pulse_state) && ((millis() - spindle_last_time) >= spindle_debounce_time)) {  // check if pulse changed state and enough time elaped (debounce)
    pulse_count++;                                                                                                       // if so then increment pulse counter
    spindle_last_pulse_state = spindle_pulse_state;                                                                      // store new pulse state
    spindle_last_time = millis();                                                                                        // store new time
  }




  /////////////////////////////////////////////
  //         update actual and demand RPM    //
  /////////////////////////////////////////////
  // if enough sample time elapsed then calculate spindle actual RPM and VFD demand RPM
  Master_Time = millis();                                             // store new time
  if ((Master_Time - rpm_last_time) >= rpm_sample_time) {             // check if enough time elapsed to calculate new rpm
    rpm_last_measurement = rpm;                                       // store previous rpm measurement
    rpm = (pulse_count * 60000) / (pulse_per_rpm * rpm_sample_time);  // calculate new rpm value and convert pulses to RPM (60 seconds)
    pulse_count = 0;                                                  // reset pulse counter
    rpm = rpm * spindle_scale / 100;                                  // scale rpm value for fine tuning
    if ((rpm_last_measurement < 10) && (rpm > 10)) {                  // check if was stationary, but now spindle is moving
      system_active = true;                                           // start monitoring for e-stops
    }
    analogWrite(IR_MONITOR_PIN, map(rpm, 0, 24000, 0, 255));  // set spindle sensor LED to a brightness proportional to rpm

    // calculate VFD demand rpm
    dutyCycle = (highTime / (highTime + lowTime)) * 100.0;                 // Calculate duty cycle percentage
    highTime = 0;                                                          // Reset times for the next measurement (allowing for processing time when sample time reached)
    lowTime = 0;                                                           // Reset times for the next measurement
    lastPrintTime = millis();                                              // record last update time
    vfd_PWM_rpm = map(dutyCycle, 0, ((100 - PWM_scale) + 100), 0, 24000);  // map ESP32 analog value range 0-4095 into rpm range 0-24000

    // check if demand and actual are ~zero
    if ((rpm < 100) && (vfd_PWM_rpm < 100)) {  // check if demand and actual speed are close to zero
      system_active = false;                   // stop monitoring for e-stops (prevents VFD power on spikes causing e-stops)
    }

    // calculate acceleration
    acceleration = (rpm - rpm_last_measurement) / (rpm_sample_time / 1000);  // acceleration = change in rpm since last calculation time per second
    rpm_last_time = Master_Time;

    // calculate delta rpm as a percentage of demand rpm
    delta_rpm = 100 - ((rpm / vfd_PWM_rpm) * 100);
    if (rpm == vfd_PWM_rpm) { delta_rpm = 0; }  // check for special case when actual rpm and demand rpm are both zero
    if (delta_rpm <= 0) { delta_rpm = 0; }      // check if rpm is higher than VFD demand

    // calculate number of bars to display on "load meter"
    load_meter_bars = (delta_rpm / delta_rpm_stall) * 10;

    // round spindle and VFD rpm to nearest 100 for more stable display on LCD
    rounded_vfd_PWM_rpm = (int(vfd_PWM_rpm / 100) * 100);  // round to nearest 100 rpm
    rounded_rpm = (int(rpm / 100) * 100);                  // round spindle rpm to nearest 100 rpm




    ////////////////////////////////////////////
    //             LCD display                //
    ////////////////////////////////////////////
    // display fixed text on LCD
    lcd.setCursor(0, 0);
    lcd.print("SP");
    lcd.setCursor(8, 0);
    lcd.print("VF");

    // display spindle
    lcd.setCursor(2, 0);
    lcd.print(rounded_rpm, 0);
    lcd.print("  ");

    // VFD demand rpm
    lcd.setCursor(10, 0);
    //lcd.print(delta_rpm);
    lcd.print(rounded_vfd_PWM_rpm, 0);
    lcd.print("   ");

    // // load meter bars
    lcd.setCursor(0, 1);
    for (int i = 1; i <= 10; i++) {
      if (i <= load_meter_bars) {
        //lcd.write(byte(0));  // Display the full square character to show follow error%
        lcd.write(byte(0));
      } else {
        //lcd.write(byte(0));  // Display rest of the meter scale
        lcd.write(1);
      }
    }





    ////////////////////////////////////////////
    //          read button presses           //
    ////////////////////////////////////////////
    // read button interface:  nothing = 4096, sel= 3880, left= 2209 , right= 0,  up= 429 , down= 1330
    read_user_pin = analogRead(BUTTON_PIN_A0);  // read button press value
    lcd.setCursor(9, 1);
    if (((read_user_pin < (KEY_SELECT + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_SELECT - BUTTON_TOLERANCE)))) {
      // pressed menu/select button so call menu subroutine
      menu_subroutine(menu, number_of_menu_rows);  // call menu routine and deal with user selections (no monitoring possible in menu mode)

      // upon return from user menu update all user variables
      rpm_sample_time = menu[0].menu_value;           // rpm sample time in milliseconds
      delta_rpm_stall = menu[1].menu_value;           // threshold as % of demand rpm for stall monitoring (e.g. 70% means if actual rpm < 70% of demand rpm system might be stalling)
      spindle_scale = menu[2].menu_value;             // scaling factor to adjust spindle rpm calculation
      PWM_scale = menu[3].menu_value;                 // scaling factor to adjust rpm calculation for VFD demand rpm
      acceleration_lower_limit = menu[4].menu_value;  // lower rpm limit to display "accel" on LCD
      acceleration_upper_limit = menu[5].menu_value;  // upper rpm limit to display "decel" on LCD

      // reset rpm counters
      rpm = 0;          // reset rpm value
      vfd_PWM_rpm = 0;  // reset VFD rpm value
    }




    //////////////////////////////////
    //      check system state      //
    //////////////////////////////////
    // store system state: 0 = unknown, 1 = idle, 2= accelerating, 3 = decelerating, 4 = running, 8 = feed hold, 9 = e-stop
    system_state = 0;  // reset state
    lcd.setCursor(11, 1);

    // stall check
    if (load_meter_bars > 9 && system_active == true) {
      // ** STALL **
      system_state = 9;
      // e-stop
      lcd.print("estop");
      // trigger estop relay
      digitalWrite(ESTOP_RELAY_PIN, HIGH);  // relay on
      digitalWrite(ESTOP_LED_PIN, HIGH);    // LED on
      delay(2000);                          // wait, then reset estop relay (to allow user to reset machine)
      digitalWrite(ESTOP_RELAY_PIN, LOW);   // reset e-stop pin
      digitalWrite(ESTOP_LED_PIN, LOW);     // LED off

    }
    // idle
    else if (rpm < min_rpm) {
      // below min rpm so set flag for idle
      system_state = 1;
      //stall_counter = 0;
      lcd.print("idle ");

    } else if ((rpm > min_rpm) && (acceleration > acceleration_upper_limit)) {
      // accel
      system_state = 2;
      //stall_counter = 0;
      lcd.print("accel");

    } else if ((rpm > min_rpm) && (acceleration < acceleration_lower_limit)) {
      // decel
      system_state = 3;
      //stall_counter = 0;
      lcd.print("decel");

    } else if ((rpm > min_rpm) && (acceleration > acceleration_lower_limit) && (acceleration < acceleration_upper_limit)) {
      // running
      system_state = 4;
      //stall_counter = 0;
      lcd.print("run  ");
    }

    // reset VFD pulse timer and sample timer
    highTime = 0;                  // Reset VFD high pulse time for the next measurement
    lowTime = 0;                   // Reset VFD low pulse time for the next measurement
    currentTime = micros();        // store VFD current time
    lastTime = currentTime;        // store new VFD time
    rpm_last_time = Master_Time;   // reset counter for signal sampling
    spindle_last_time = millis();  // reset spindle rpm timer
  }
}






/////////////////////////////////////////////////////
//                    Subroutines                  //
/////////////////////////////////////////////////////

// Main user menu
void menu_subroutine(menu_structure menu[], int menu_size) {
  lcd.clear();             // clear LCD screen
  delay(250);              // wait to allow key to be released
  int menu_pointer = 0;    // row pointer in menu array
  bool menu_exit = false;  // flag to exit menu

  // main menu
  while (menu_exit != true) {
    lcd.setCursor(0, 0);
    lcd.print(menu[menu_pointer].menu_label);          // show user variable label
    lcd.setCursor(0, 1);                               // move cursor
    lcd.print(menu[menu_pointer].menu_value);          // show user variable value
    lcd.print(" ");                                    // space to clear digits
    lcd.setCursor(10, 1);                              // move cursor
    lcd.print("[");                                    // factory value open [
    lcd.print(menu[menu_pointer].menu_factory_value);  // factory value
    lcd.print("]");                                    // factory value close]

    // read button interface:  nothing = 4096, sel= 3880, left= 2209 , right= 0,  up= 429 , down= 1330
    read_user_pin = analogRead(BUTTON_PIN_A0);  // read button press value
    // check for UP button
    if (((read_user_pin < (KEY_UP + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_UP - BUTTON_TOLERANCE)))) {
      menu[menu_pointer].menu_value = menu[menu_pointer].menu_value + menu[menu_pointer].menu_inc_dec;
      if (menu[menu_pointer].menu_value >= menu[menu_pointer].menu_value_max) { menu[menu_pointer].menu_value = menu[menu_pointer].menu_value_max; }  // ensure value does not exceed max permitted value
      delay(100);
      lcd.clear();
    }
    // check for DOWN button
    if (((read_user_pin < (KEY_DOWN + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_DOWN - BUTTON_TOLERANCE)))) {
      menu[menu_pointer].menu_value = menu[menu_pointer].menu_value - menu[menu_pointer].menu_inc_dec;
      if (menu[menu_pointer].menu_value <= menu[menu_pointer].menu_value_min) { menu[menu_pointer].menu_value = menu[menu_pointer].menu_value_min; }  // ensure value does not exceed max permitted value
      delay(100);
      lcd.clear();
    }
    // check for LEFT button
    if (((read_user_pin < (KEY_LEFT + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_LEFT - BUTTON_TOLERANCE)))) {
      menu_pointer = menu_pointer - 1;                     // increment menu pointer in array to show next menu item and associated values
      if (menu_pointer < 0) { menu_pointer = menu_size; }  // wrap menu around
      delay(250);
      lcd.clear();
    }
    // check fr RIGHT button
    if (((read_user_pin < (KEY_RIGHT + BUTTON_TOLERANCE)) && (read_user_pin >= (KEY_RIGHT)))) {
      menu_pointer = menu_pointer + 1;                      // increment menu pointer in array to show next menu item and associated values
      if (menu_pointer >= menu_size) { menu_pointer = 0; }  // wrap menu around
      delay(250);
      lcd.clear();
    }

    // check for select (menu) button
    if (((read_user_pin < (KEY_SELECT + BUTTON_TOLERANCE)) && (read_user_pin > (KEY_SELECT - BUTTON_TOLERANCE)))) {  // check if menu button pressed (select)
      read_user_pin = analogRead(BUTTON_PIN_A0);                                                                     // read button state
      while (read_user_pin < 4090) { read_user_pin = analogRead(BUTTON_PIN_A0); }                                    // if still pressed wait for button to be released
      menu_exit = true;                                                                                              // set menu exit flag to exit routine
    }
  }

  // save user values into non-volatile memory
  saveUserData();

  // tidy up and exit menu
  lcd.clear();                // clear LCD screen
  lcd.setCursor(0, 0);        // move cursor
  lcd.print("Saving . . .");  // display message

  // display 'progress bar' (really just showing user that something has happened)
  lcd.setCursor(0, 1);
  for (int i = 1; i <= 15; i++) {
    lcd.write(byte(0));  // Display the full square character
    delay(125);
  }
  lcd.clear();  // clear LCD screen
}

// save user data into memory
void saveUserData() {
  // I tried to get this to work in a for loop but gave up and just wrote them all out
  preferences.putInt("key_value_0", menu[0].menu_value);
  preferences.putInt("key_value_1", menu[1].menu_value);
  preferences.putInt("key_value_2", menu[2].menu_value);
  preferences.putInt("key_value_3", menu[3].menu_value);
  preferences.putInt("key_value_4", menu[4].menu_value);
  preferences.putInt("key_value_5", menu[5].menu_value);
  preferences.putInt("key_value_6", menu[6].menu_value);
}

// load user data from memory
void loadUserData() {
  // I tried to get this to work in a for loop but gave up and just wrote them all out
  menu[0].menu_value = preferences.getInt("key_value_0", -1);
  menu[1].menu_value = preferences.getInt("key_value_1", -1);
  menu[2].menu_value = preferences.getInt("key_value_2", 0);
  menu[3].menu_value = preferences.getInt("key_value_3", 0);
  menu[4].menu_value = preferences.getInt("key_value_4", 0);
  menu[5].menu_value = preferences.getInt("key_value_5", 0);
  menu[6].menu_value = preferences.getInt("key_value_6", 0);
}
