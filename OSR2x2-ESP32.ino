// ESP32-OSR2x2-vamrobot-v1.2
// by vamrobot 12-10-2021
// based on OSR-Alpha3_ESP32 by TempestMAx 9-7-21
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control servos and vibration motors
// It can handle:
//   10x linear channels (L0, L1, L2... L9)
//   10x rotation channels (R0, R1, R2... L9) 
//   10x vibration channels (V0, V1, V2... V9)
//   10x auxilliary channels (A0, A1, A2... A9)
// This code is for the ESP32 DevKit v1 and is designed to drive the OSR2x2 stroker robot.
// This firmware also supports 'manual stroker' operation mode of the OSR2x2 robot and the ability to switch from
// T-Code serial input/output process mode (standard) to manual stroker mode and back by pushing a push button.
// On top of that the manual mode can be controlled via up to 6 push buttons, controlling the speeds of the
// L0 (x axis), A3 (compression}, and the A4 (bend) movements.
// Have fun, play safe!
// History:
// Alpha3 - First ESP32 release, 9-7-2021
// OSR2x2-ESP32-vamrobot-v1.2 12-10-2021


// ----------------------------
//   Settings
// ----------------------------

// Device IDs, for external reference
#define FIRMWARE_ID "OSR2x2-ESP32-vamrobot-v1.2.ino"  // Device and firmware version
#define TCODE_VER "TCode v0.3"  // Current version of TCode

// Select startup firmware mode
// 0 for serial T-Code processing (normal) mode overlaid with manually controllable compression (off by default) and bend (off by default) movements
// 1 for auto stroker mode with manually controllable compression (off by default) and bend (off by default) movements
#define FIRMWARE_MODE_INITIAL 0

// Push buttons configuration setup
// PUSH_BUTTONS_CONFIG below has these possible values based on how many push buttons you have
// and based on what you want them each to do:
//
//   Value - Description
//
//   0 - Uses no push buttons, firmware switching disabled, uses only the serial T-Code processing (normal) mode
//
//   1 - Uses 2 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//
//   2 - Uses 3 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//       Button 3 (FirmwareSwitchButton_PIN) - Switches firmware modes
//
//   3 - Uses 4 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//       Button 3 (IncreaseA3SpeedButton_PIN) - Increase A3 (compression) stroker speed in manual mode and enables/disables A3 (compression) if held for 2 seconds
//       Button 4 (DecreaseA3SpeedButton_PIN) - Decrease A3 (compression) stroker speed in manual mode and if the button is held for 2 seconds then
//                                              the Dual Ring Receiver Offset value is reset to OSR2X2_DUAL_RING_RECEIVER_OFFSET
//
//   4 - Uses 5 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//       Button 3 (IncreaseA3SpeedButton_PIN) - Increase A3 (compression) stroker speed in manual mode and enables/disables A3 (compression) if held for 2 seconds
//       Button 4 (DecreaseA3SpeedButton_PIN) - Decrease A3 (compression) stroker speed in manual mode and if the button is held for 2 seconds then
//                                              the Dual Ring Receiver Offset value is reset to OSR2X2_DUAL_RING_RECEIVER_OFFSET
//       Button 5 (FirmwareSwitchButton_PIN) - Switches firmware modes
//
//   5 - Uses 6 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//       Button 3 (IncreaseA3SpeedButton_PIN) - Increase A3 (compression) stroker speed in manual mode and enables/disables A3 (compression) if held for 2 seconds
//       Button 4 (DecreaseA3SpeedButton_PIN) - Decrease A3 (compression) stroker speed in manual mode and if the button is held for 2 seconds then
//                                              the Dual Ring Receiver Offset value is reset to OSR2X2_DUAL_RING_RECEIVER_OFFSET
//       Button 5 (IncreaseA4SpeedButton_PIN) - Increase A4 (bend) stroker speed in manual mode and enables/disables A4 (bend) if held for 2 seconds
//       Button 6 (DecreaseA4SpeedButton_PIN) - Decrease A4 (bend) stroker speed in manual mode
//
//   6 - Uses 7 total push buttons having these functions:
//       Button 1 (IncreaseL0SpeedButton_PIN) - Increase L0 (x axis) stroker speed in manual mode and switches firmware modes if held for 2 seconds
//       Button 2 (DecreaseL0SpeedButton_PIN) - Decrease L0 (x axis) stroker speed in manual mode and switches the two push buttons that function as
//                                              Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset
//                                              if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons
//                                              back to functioning as Increase and Decrease for L0 (x axis).
//       Button 3 (IncreaseA3SpeedButton_PIN) - Increase A3 (compression) stroker speed in manual mode and enables/disables A3 (compression) if held for 2 seconds
//       Button 4 (DecreaseA3SpeedButton_PIN) - Decrease A3 (compression) stroker speed in manual mode and if the button is held for 2 seconds then
//                                              the Dual Ring Receiver Offset value is reset to OSR2X2_DUAL_RING_RECEIVER_OFFSET
//       Button 5 (IncreaseA4SpeedButton_PIN) - Increase A4 (bend) stroker speed in manual mode and enables/disables A4 (bend) if held for 2 seconds
//       Button 6 (DecreaseA4SpeedButton_PIN) - Decrease A4 (bend) stroker speed in manual mode
//       Button 7 (FirmwareSwitchButton_PIN) - Switches firmware modes
#define PUSH_BUTTONS_CONFIG 5

// Master servo update frequency
// Used in all firmware modes to update the servos is a cyclic fasion that can be 'timed' against
#define MasterServoUpdateFrequency 50

// Sets the frequencies for the different manual stroker operations
#define ManualL0FrequencyInitial 0.6 // Initial L0 (x axis) stroke frequency, in strokes per second
#define ManualL0FrequencyStep 0.2 // Step value up/down for the L0 (x axis) stroke frequency, in strokes per second
#define ManualA3FrequencyInitial 1 // Initial A3 (compression) stroke frequency, in strokes per second
#define ManualA3FrequencyStep 0.2 // Step value up/down for the A3 (compression) stroke frequency, in strokes per second
#define ManualA4FrequencyInitial 2 // Initial A4 (bend) stroke frequency, in strokes per second
#define ManualA4FrequencyStep 0.2 // Step value up/down for the A4 (bend) stroke frequency, in strokes per second

// Min and max range values for main OSR2x2 operations
#define StrokeMaxValue 240 // Sets the maximum L0 (x axis) value
#define StrokeMinValue -240 // Sets the minimum L0 (x axis) value
#define RollMaxValue 120 // Sets the maximum R1 (y axis) value
#define RollMinValue -120 // Sets the minimum R1 (y axis) value
#define CompressionMaxValue 40 // Sets the maximum A3 (compression) value
#define CompressionMinValue -20 // Sets the minimum A3 (compression) value
#define BendMaxValue 40 // Sets the maximum A4 (bend) value
#define BendMinValue -40 // Sets the minimum A4 (bend) value

// Set the absolute maximum and minimum positions/ranges for the servos of the OSR2x2
// These values are used in a function run just before setting/moving the servos themselves to ensure
// that the servos never exceed these minimums and maximums.
#define OSR2X2_POSITION_MAX 350
#define OSR2X2_POSITION_MIN -350

// Set the 'average offset' value for the gap between the dual ring receivers in the OSR2x2
// This allows you to bring the dual ring receivers closer (positive value) or
// spread the dual ring receivers apart (negative value)
// An generally acceptable range is from -90 (toward much longer sleeves/onaholes)
// to 30 (slightly smaller than a Quickshot case and helpful if your Quickshot is slipping
// or if you just want a bit of constant compression/squeeze)
// Example numbers to try for different sleeves:
// Quickshot - 0
// Quickshot compressed slightly - 20
// Chinese Quickshot - -20
// TOMAX Lilith Uterus - -90
// Absorption Lilith Six Uterus - -50
#define OSR2X2_DUAL_RING_RECEIVER_OFFSET 0

// Step change value for the Dual Ring Receiver Offset for when using the two push buttons to modify the OSR2X2_DUAL_RING_RECEIVER_OFFSET value
#define OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE 20

// Servo microseconds per radian
// (Standard: 637 μs/rad)
// (LW-20: 700 μs/rad)
#define ms_per_rad 637  // (μs/rad)

// Pin assignments
// T-wist feedback goes on digital pin 2
#define LowerLeftServo_PIN 19    // OSR2 Left Servo, OSR2x2/SR6 Lower Left Servo
#define UpperLeftServo_PIN 4    // OSR2x2/SR6 Upper Left Servo
#define LowerRightServo_PIN 21   // OSR2 Right Servo, OSR2x2/SR6 Lower Right Servo
#define UpperRightServo_PIN 18   // OSR2x2/SR6 Upper Right Servo
#define TwistServo_PIN 27        // Twist Servo
#define ValveServo_PIN 5        // Valve Servo
#define TwistFeedback_PIN 13     // Twist Servo Feedback
#define Vibe0_PIN 22             // Vibration motor 1
#define Vibe1_PIN 23             // Vibration motor 2
#define IncreaseL0SpeedButton_PIN 25 // Push button input that increases the L0 (x axis) stroker speed when in manual stroker mode
#define DecreaseL0SpeedButton_PIN 26 // Push button input that decreases the L0 (x axis) stroker speed when in manual stroker mode
#define IncreaseA3SpeedButton_PIN 32 // Push button input that increases the A3 (compression) stroker speed when in manual stroker mode
#define DecreaseA3SpeedButton_PIN 33 // Push button input that decreases the A3 (compression) stroker speed when in manual stroker mode
#define IncreaseA4SpeedButton_PIN 34 // Push button input that increases the A4 (bend) stroker speed when in manual stroker mode
#define DecreaseA4SpeedButton_PIN 35 // Push button input that decreases the A4 (bend) stroker speed when in manual stroker mode
#define FirmwareSwitchButton_PIN 2 // Push button input that switches the firmware modes

// Arm servo zeros
// Change these to adjust arm positions
// (1500 = centre)
#define LowerLeftServo_ZERO 1470   // OSR2 Left Servo, OSR2x2/SR6 Lower Left Servo
#define UpperLeftServo_ZERO 1470   // OSR2x2/SR6 Upper Left Servo
#define LowerRightServo_ZERO 1550  // OSR2 Right Servo, OSR2x2/SR6 Lower Right Servo
#define UpperRightServo_ZERO 1515  // OSR2x2/SR6 Upper Right Servo
#define TwistServo_ZERO 1500
#define ValveServo_ZERO 1500

// Servo operating frequencies
#define MainServo_Freq 330  // Main Servos
#define TwistServo_Freq 50  // Twist Servo
#define ValveServo_Freq 50  // Valve Servo
#define VibePWM_Freq 8000   // Vibe motor control PWM frequency

// Other functions
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999) 
#define REVERSE_VALVE_SERVO false // (true/false) Reverse T-Valve direction
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define Lube_PIN 23               // Lube manual input button pin (Connect pin to +5V for ON)
#define Lube_SPEED 255            // Lube pump speed (0-255)
#define MIN_SMOOTH_INTERVAL 3     // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL 100   // Maximum auto-smooth ramp interval for live commands (ms)

// T-Code Channels
#define CHANNELS 10                // Number of channels of each type (LRVA)


// ----------------------------
//  Auto Settings
// ----------------------------
// Do not change

// Servo PWM channels
#define LowerLeftServo_PWM 0     // Lower Left Servo
#define UpperLeftServo_PWM 1     // Upper Left Servo
#define LowerRightServo_PWM 2    // Lower Right Servo
#define UpperRightServo_PWM 3    // Upper Right Servo
#define TwistServo_PWM 6         // Twist Servo
#define ValveServo_PWM 7         // Valve Servo
#define TwistFeedback_PWM 8      // Twist Servo
#define Vibe0_PWM 9              // Vibration motor 1
#define Vibe1_PWM 10             // Vibration motor 2

// Servo Pulse intervals
#define MainServo_Int 1000000/MainServo_Freq
#define TwistServo_Int 1000000/TwistServo_Freq
#define ValveServo_Int 1000000/ValveServo_Freq

// Libraries used
#include <EEPROM.h> // Permanent memory


// -----------------------------
// Class to handle each axis
// -----------------------------
class Axis {

  public:
  // Setup function
  Axis() {

    // Set default dynamic parameters
    rampStartTime = 0;
    rampStart = 5000;
    rampStopTime = rampStart;
    rampStop = rampStart;

    // Set Empty Name
    Name = "";
    lastT = 0;

    // Live command auto-smooth
    minInterval = MAX_SMOOTH_INTERVAL;
      
  }

  // Function to set the axis dynamic parameters
  void Set(int x, char ext, long y) {
    unsigned long t = millis(); // This is the time now
    x = constrain(x,0,9999);
    y = constrain(y,0,9999999);
    // Set ramp parameters, based on inputs
    // Live command
    if ( y == 0 || ( ext != 'S' && ext != 'I' ) ) {
      // update auto-smooth regulator
      int lastInterval = t - rampStartTime;
      if ( lastInterval > minInterval && minInterval < MAX_SMOOTH_INTERVAL ) { minInterval += 1; }
      else if ( lastInterval < minInterval && minInterval > MIN_SMOOTH_INTERVAL ) { minInterval -= 1; } 
      // Set ramp parameters
      rampStart = GetPosition();
      rampStopTime = t + minInterval;  
    } 
    // Speed command
    else if ( ext == 'S' ) {
      rampStart = GetPosition();  // Start from current position
      int d = x - rampStart;  // Distance to move
      if (d<0) { d = -d; }
      long dt = d;  // Time interval (time = dist/speed)
      dt *= 100;
      dt /= y; 
      rampStopTime = t + dt;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    // Interval command
    else if ( ext == 'I' ) {
      rampStart = GetPosition();  // Start from current position
      rampStopTime = t + y;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    rampStartTime = t;
    rampStop = x;
    lastT = t;
  }

  // Function to return the current position of this axis
  int GetPosition() {
    int x; // This is the current axis position, 0-9999
    unsigned long t = millis(); 
    if (t > rampStopTime) {
      x = rampStop;
    } else if (t > rampStartTime) { 
      x = map(t,rampStartTime,rampStopTime,rampStart,rampStop);
    } else {
      x = rampStart;
    }
    x = constrain(x,0,9999);
    return x;
  }

  // Function to stop axis movement at current position
  void Stop() {
    unsigned long t = millis(); // This is the time now
    rampStart = GetPosition();
    rampStartTime = t;
    rampStop = rampStart;
    rampStopTime = t;
  }

  // Public variables
  String Name;  // Function name of this axis
  unsigned long lastT;  //

  private:
  
  // Movement positions
  int rampStart;
  unsigned long rampStartTime;
  int rampStop;
  unsigned long rampStopTime;

  // Live command auto-smooth regulator
  int minInterval;

};


// -----------------------------
// Class to manage Toy Comms
// -----------------------------
class TCode {
  
  public:
  // Setup function
  TCode(String firmware, String tcode) {
    firmwareID = firmware;
    tcodeID = tcode;

    // Vibe channels start at 0
    for (int i = 0; i < CHANNELS; i++) { Vibration[i].Set(0,' ',0); }
    
  }

  // Function to name and activate axis
  void RegisterAxis(String ID, String axisName) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Name = axisName; break;
        case 'R': Rotation[channel].Name = axisName; break;
        case 'V': Vibration[channel].Name = axisName; break;
        case 'A': Auxiliary[channel].Name = axisName; break;
      }
    }
  }

  // Function to read off individual bytes as input
  void ByteInput(byte inByte) {
    bufferString += (char)inByte;  // Add new character to string
    
    if (inByte=='\n') {  // Execute string on newline
      bufferString.trim();  // Remove spaces, etc, from buffer
      executeString(bufferString); // Execute string
      bufferString = ""; // Clear input string
    }
  }

  // Function to read off whole strings as input
  void StringInput(String inString) {
    bufferString = inString;  // Replace existing buffer with input string
    bufferString.trim();  // Remove spaces, etc, from buffer
    executeString(bufferString); // Execute string
    bufferString = ""; // Clear input string
  }

  // Function to set an axis
  void AxisInput(String ID, int magnitude, char extension, long extMagnitude) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }
  }

  // Function to read the current position of an axis
  int AxisRead(String ID) {
    int x = 5000; // This is the return variable
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': x = Linear[channel].GetPosition(); break;
        case 'R': x = Rotation[channel].GetPosition(); break;
        case 'V': x = Vibration[channel].GetPosition(); break;
        case 'A': x = Auxiliary[channel].GetPosition(); break;
      }
    }
    return x;
  }

  // Function to query when an axis was last commanded
  unsigned long AxisLast(String ID) {
    unsigned long t = 0; // Return time
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': t = Linear[channel].lastT; break;
        case 'R': t = Rotation[channel].lastT; break;
        case 'V': t = Vibration[channel].lastT; break;
        case 'A': t = Auxiliary[channel].lastT; break;
      }
    }
    return t;
  }

  private:
  // Strings
  String firmwareID;
  String tcodeID;
  String bufferString; // String to hold incomming commands


  // Declare axes
  Axis Linear[CHANNELS];
  Axis Rotation[CHANNELS];
  Axis Vibration[CHANNELS];
  Axis Auxiliary[CHANNELS];

  // Function to divide up and execute input string
  void executeString(String bufferString) {
    int index = bufferString.indexOf(' ');  // Look for spaces in string
    while (index > 0) {
      readCmd(bufferString.substring(0,index));  // Read off first command
      bufferString = bufferString.substring(index+1);  // Remove first command from string
      bufferString.trim();
      index = bufferString.indexOf(' ');  // Look for next space
    }
    readCmd(bufferString);  // Read off last command
  }


  // Function to process the individual commands
  void readCmd(String command) {
    command.toUpperCase();
  
    // Switch between command types
    switch( command.charAt(0) ) {
      // Axis commands
      case 'L':
      case 'R':
      case 'V':
      case 'A':
        axisCmd(command);
      break;
  
      // Device commands
      case 'D':
        deviceCmd(command);
      break;
  
      // Setup commands
      case '$':
        setupCmd(command);
      break; 
    }
  }
  

  // Function to read and interpret axis commands
  void axisCmd(String command) {
  
    char type = command.charAt(0);  // Type of command - LRVA
    boolean valid = true;  // Command validity flag, valid by default
  
    // Check for channel number
    int channel = command.charAt(1) - '0';
    if (channel < 0 || channel >= CHANNELS) {valid = false;}
    channel = constrain(channel,0,CHANNELS);
  
    // Check for an extension
    char extension = ' ';
    int index = command.indexOf('S',2);
    if (index > 0) {
      extension = 'S';
    } else {
      index = command.indexOf('I',2);
      if (index > 0) {
        extension = 'I';
      }
    }
    if (index < 0) { index = command.length(); }
    
    // Get command magnitude
    String magString = command.substring(2,index);
    magString = magString.substring(0,4);
    while (magString.length() < 4) { magString += '0'; }
    int magnitude = magString.toInt();
    if (magnitude == 0 && magString.charAt(0) != '0') { valid = false; } // Invalidate if zero returned, but not a number
  
    // Get extension magnitude
    long extMagnitude = 0;
    if ( extension != ' ') {
      magString = command.substring(index+1);
      magString = magString.substring(0,8);
      extMagnitude = magString.toInt();
    }
    if (extMagnitude == 0) { extension = ' '; }

    // Switch between command types
    if (valid) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }

  }

  // Function to identify and execute device commands
  void deviceCmd(String command) {
    int i;
    // Remove "D"
    command = command.substring(1);

    // Look for device stop command
    if (command.substring(0,4) == "STOP") {
        for (i = 0; i < 10; i++) { Linear[i].Stop(); }
        for (i = 0; i < 10; i++) { Rotation[i].Stop(); }
        for (i = 0; i < 10; i++) { Vibration[i].Set(0,' ',0); }
        for (i = 0; i < 10; i++) { Auxiliary[i].Stop(); }  
    } else {
      // Look for numbered device commands
      int commandNumber = command.toInt();
      if (commandNumber==0 && command.charAt(0)!='0' ) { command = -1; }
      switch( commandNumber ) {
        case 0:
          Serial.println(firmwareID);
        break;
  
        case 1:
          Serial.println(tcodeID);
        break;
  
        case 2:
          for (i = 0; i < 10; i++) { axisRow("L" + String(i), 8*i, Linear[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("R" + String(i), 8*i+80, Rotation[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("V" + String(i), 8*i+160, Vibration[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("A" + String(i), 8*i+240, Auxiliary[i].Name); }             
        break;
      }
    }

    
  }

  // Function to modify axis preference values
  void setupCmd(String command) {
    int minVal,maxVal;
    String minValString,maxValString;
    boolean valid;
    // Axis type
    char type = command.charAt(1); 
    switch (type) {
      case 'L':
      case 'R':
      case 'V':
      case 'A':
      valid = true;
      break;

      default:
      type = ' ';
      valid = false;
      break;
    }
    // Axis channel number
    int channel = (command.substring(2,3)).toInt();
    if (channel == 0 && command.charAt(2) != '0') {
      valid = false;
    }
    // Input numbers
    int index1 = command.indexOf('-');  
    if (index1 !=3) { valid = false; }
    int index2 = command.indexOf('-',index1+1);  // Look for spaces in string
    if (index2 <=3) { valid = false; }
    if (valid) {
      // Min value
      minValString = command.substring(4,index2);
      minValString = minValString.substring(0,4);
      while (minValString.length() < 4) { minValString += '0'; }
      minVal = minValString.toInt();
      if ( minVal == 0 && minValString.charAt(0)!='0' ) { valid = false; }
      // Max value
      maxValString = command.substring(index2+1);
      maxValString = maxValString.substring(0,4);
      while (maxValString.length() < 4) { maxValString += '0'; }
      maxVal = maxValString.toInt();
      if ( maxVal == 0 && maxValString.charAt(0)!='0' ) { valid = false; }     
    }
    // If a valid command, save axis preferences to EEPROM
    if (valid) {
      int memIndex = 0;
      switch (type) {
        case 'L': memIndex = 0; break;
        case 'R': memIndex = 80; break;
        case 'V': memIndex = 160; break;
        case 'A': memIndex = 240; break;
      }
      memIndex += 8*channel;
      minVal = constrain(minVal,0,9999);
      EEPROM.put(memIndex, minVal-1);
      minVal = constrain(maxVal,0,9999);
      EEPROM.put(memIndex+4, maxVal-10000);
      // Output that axis changed successfully
      switch (type) {
        case 'L': axisRow("L" + String(channel), memIndex, Linear[channel].Name); break;
        case 'R': axisRow("R" + String(channel), memIndex, Rotation[channel].Name); break;
        case 'V': axisRow("V" + String(channel), memIndex, Vibration[channel].Name); break;
        case 'A': axisRow("A" + String(channel), memIndex, Auxiliary[channel].Name); break;             
      }
    }
  }
 
  // Function to print the details of an axis
  void axisRow(String axisID, int memIndex, String axisName) {
    int low, high;
    if (axisName != "") {
      EEPROM.get(memIndex,low);
      low = constrain(low,-1,9998);
      EEPROM.get(memIndex + 4,high);
      high = constrain(high,-10000,-1);
      Serial.print(axisID);
      Serial.print(" ");
      Serial.print(low + 1);
      Serial.print(" ");
      Serial.print(high + 10000);
      Serial.print(" ");
      Serial.println(axisName);
    }
  }
    
};

// -----------------------------
// Class to handle push buttons
// -----------------------------
class PushButton {

  public:
  // Setup function
  PushButton(int pin) {

    pushButtonPin = pin;
      
  }

  // Function to determine and return the state of the push button
  int State() {
    // Take the digital readings of the push button
    int pushButtonReading = digitalRead(pushButtonPin);

    // Check if the push button state changed
    if (pushButtonReading != pushButtonStateLast)
    {
      // Reset the last debounce time
      pushButtonLastDebounceTime = millis();
    }

    long elapsedpushButtonTime = millis() - pushButtonLastDebounceTime;

    // Check if the minimum debounce time has been exceeded for the push button
    if (elapsedpushButtonTime > 50)
    {
      // Change the push button's state if it has indeed changed
      if (pushButtonReading != pushButtonState)
      {
        pushButtonState = pushButtonReading;

        // Check if the button has been pressed, ie in a HIGH state
        if (pushButtonState == HIGH)
        {
          // Store the reading for the push button
          pushButtonStateLast = pushButtonReading;
    
          return 1;
        }
        else
        {
          // Store the reading for the push button
          pushButtonStateLast = pushButtonReading;
    
          return 0;
        }
      }
      else
      {
        // Check if the increase push button has been pressed for enough time to switch firmware modes
        if (elapsedpushButtonTime >= 2000 && pushButtonState == HIGH)
        {
          if ((millis() - lastPushButtonHeld) >= 2000)
          {
            // Store the reading for the push button
            pushButtonStateLast = pushButtonReading;
            
            lastPushButtonHeld = millis();
      
            return 2;
          }
        }
      }
    }
    
    // Store the reading for the push button
    pushButtonStateLast = pushButtonReading;
    
    return 0;
  }

  private:

  int pushButtonPin; // The push button's ESP32 pin
  int pushButtonState; // The state of the push button
  int pushButtonStateLast = LOW; // The last state of the push button
  unsigned long pushButtonLastDebounceTime = 0;  // The last time the push button's pin was toggled
  long lastPushButtonHeld = 0;

};

// -----------------------------
// Class to handle manual stroker modes
// -----------------------------
class Manual {

  public:
  // Setup function
  Manual(float frequencyInitial, float frequencyStep) {

    manualValue = 5000;
    manualFrequency = frequencyInitial;
    manualFrequencyStep = frequencyStep;
    manualStepValue = (int) ((20000 * manualFrequency) / MasterServoUpdateFrequency);
    manualDirection = true;
      
  }

  // Function to get the new manual value
  int Value() {
    // Check the current manual direction
    if (manualDirection)
    {
      // Increase the manual value
      manualValue += manualStepValue;

      // If the manual value has reached it's max, go in the opposite direction
      if (manualValue >= 9999)
      {
        manualValue = 9999;
        manualDirection = false;
      }
    }
    else
    {
      // Decrease the manual value
      manualValue -= manualStepValue;

      // If the manual value has reached it's min, go in the opposite direction
      if (manualValue <= 0)
      {
        manualValue = 0;
        manualDirection = true;
      }
    }

    return manualValue;
  }

  void Increase()
  {
    manualFrequency += manualFrequencyStep;
    manualStepValue = (int) ((20000 * manualFrequency) / MasterServoUpdateFrequency);
    Serial.println(manualFrequency);
    Serial.println(manualStepValue);
    Serial.println();
  }

  void Decrease()
  {
    if ((manualFrequency - manualFrequencyStep) > manualFrequencyStep)
    {
      manualFrequency -= manualFrequencyStep;
      manualStepValue = (int) ((20000 * manualFrequency) / MasterServoUpdateFrequency);
      Serial.println(manualFrequency);
      Serial.println(manualStepValue);
      Serial.println();
    }
  }

  private:

  // Manual mode settings
  float manualFrequency;
  float manualFrequencyStep;
  int manualStepValue;
  int manualValue;
  bool manualDirection;

};



// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above
TCode tcode(FIRMWARE_ID, TCODE_VER);

// Declare operating variables
// Position variables
int xLin,yLin,zLin;
// Rotation variables
int xRot,yRot,zRot;
// Compression variable
int xComp;
// Bend variable
int yBend;
// Vibration variables
int vibe0,vibe1;
// Lube variables
int lube;
// Valve variables
int valveCmd,suckCmd;
// Velocity tracker variables, for valve
int xLast;
unsigned long tLast;
float upVel,valvePos;
// Twist position monitor variables
volatile int twistPulseLength = 0;
volatile int twistPulseCycle = 1099;
volatile int twistPulseStart = 0;
float twistServoAngPos = 0.5;
int twistTurns = 0;
float twistPos;
int firmwareMode = 0;
int mainTwoButtonsMode = 0;
int dualRingReceiverOffsetValue = OSR2X2_DUAL_RING_RECEIVER_OFFSET;
bool compressionEnabled = false;
bool bendEnabled = false;
int compressionMax = max(abs(CompressionMinValue), abs(CompressionMaxValue));
int bendMax = max(abs(BendMinValue), abs(BendMaxValue));
int manualModeStrokeMax = StrokeMaxValue - compressionMax - bendMax;
int manualModeStrokeMin = StrokeMinValue + compressionMax + bendMax;
int tCodeModeStrokeMax = StrokeMaxValue - compressionMax - bendMax;
int tCodeModeStrokeMin = StrokeMinValue + compressionMax + bendMax;
int tCodeModeRollMax = RollMaxValue - compressionMax - bendMax;
int tCodeModeRollMin = RollMinValue + compressionMax + bendMax;
// Setup manual L0 (x axis) object
Manual manualL0(ManualL0FrequencyInitial, ManualL0FrequencyStep);
// Setup manual A3 (compression) objectss
Manual manualA3(ManualA3FrequencyInitial, ManualA3FrequencyStep);
// Setup manual A4 (bend) object
Manual manualA4(ManualA4FrequencyInitial, ManualA4FrequencyStep);
// Setup push button objects
PushButton pushButtonIncreaseL0(IncreaseL0SpeedButton_PIN);
PushButton pushButtonDecreaseL0(DecreaseL0SpeedButton_PIN);
PushButton pushButtonIncreaseA3(IncreaseA3SpeedButton_PIN);
PushButton pushButtonDecreaseA3(DecreaseA3SpeedButton_PIN);
PushButton pushButtonIncreaseA4(IncreaseA4SpeedButton_PIN);
PushButton pushButtonDecreaseA4(DecreaseA4SpeedButton_PIN);
PushButton pushButtonFirmwareSwitch(FirmwareSwitchButton_PIN);
long manualModeUpdatePeriod = 1000 / MasterServoUpdateFrequency;
long previousMillis = 0;

// Setup function
// This is run once, when the arduino starts
void setup() {

  // Start serial connection and report status
  Serial.begin(115200);
  tcode.StringInput("D0");
  tcode.StringInput("D1");

  Serial.println("Stroker Robot: OSR2x2");

  // Setup initial firmware mode
  if (PUSH_BUTTONS_CONFIG == 0)
  {
    firmwareMode = 0;
  }
  else
  {
    firmwareMode = FIRMWARE_MODE_INITIAL;
  }
  
  // #ESP32# Enable EEPROM
  EEPROM.begin(320);

  // Register device axes
  tcode.RegisterAxis("L0", "Up");
  tcode.RegisterAxis("R0", "Twist");
  tcode.RegisterAxis("R1", "Roll");
  tcode.RegisterAxis("V0", "Vibe1");
  if (!LUBE_V1) { tcode.RegisterAxis("V1", "Vibe2"); }
  tcode.RegisterAxis("A0", "Valve");
  tcode.RegisterAxis("A1", "Suck");
  tcode.AxisInput("A1",VALVE_DEFAULT,'I',3000);
  if (LUBE_V1) {
    tcode.RegisterAxis("A2", "Lube");
    tcode.AxisInput("A2",0,' ',0);
    pinMode(Lube_PIN,INPUT);
  }
  tcode.RegisterAxis("A3", "Compression");
  tcode.RegisterAxis("A4", "Bend");

  // Setup Servo PWM channels
  // Lower Left Servo
  ledcSetup(LowerLeftServo_PWM,MainServo_Freq,16);
  ledcAttachPin(LowerLeftServo_PIN,LowerLeftServo_PWM);
  // Upper Left Servo
  ledcSetup(UpperLeftServo_PWM,MainServo_Freq,16);
  ledcAttachPin(UpperLeftServo_PIN,UpperLeftServo_PWM);
  // Lower Right Servo
  ledcSetup(LowerRightServo_PWM,MainServo_Freq,16);
  ledcAttachPin(LowerRightServo_PIN,LowerRightServo_PWM);
  // Upper Right Servo
  ledcSetup(UpperRightServo_PWM,MainServo_Freq,16);
  ledcAttachPin(UpperRightServo_PIN,UpperRightServo_PWM);
  // Twist Servo
  ledcSetup(TwistServo_PWM,TwistServo_Freq,16);
  ledcAttachPin(TwistServo_PIN,TwistServo_PWM);
  // Valve Servo
  ledcSetup(ValveServo_PWM,ValveServo_Freq,16);
  ledcAttachPin(ValveServo_PIN,ValveServo_PWM);

  // Set vibration PWM pins
  // Vibe0 Pin
  ledcSetup(Vibe0_PWM,VibePWM_Freq,8);
  ledcAttachPin(Vibe0_PIN,Vibe0_PWM);
  // Vibe1 Pin
  ledcSetup(Vibe1_PWM,VibePWM_Freq,8);
  ledcAttachPin(Vibe1_PIN,Vibe1_PWM); 

  // Initiate position tracking for twist
  pinMode(TwistFeedback_PIN,INPUT);
  attachInterrupt(TwistFeedback_PIN, twistRising, RISING);

  // Setup input push buttons for manual mode and firmware switching
  if (PUSH_BUTTONS_CONFIG == 1)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
  }
  else if (PUSH_BUTTONS_CONFIG == 2)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
    pinMode(FirmwareSwitchButton_PIN, INPUT);
  }
  else if (PUSH_BUTTONS_CONFIG == 3)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
    pinMode(IncreaseA3SpeedButton_PIN, INPUT);
    pinMode(DecreaseA3SpeedButton_PIN, INPUT);
  }
  else if (PUSH_BUTTONS_CONFIG == 4)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
    pinMode(IncreaseA3SpeedButton_PIN, INPUT);
    pinMode(DecreaseA3SpeedButton_PIN, INPUT);
    pinMode(FirmwareSwitchButton_PIN, INPUT);
  }
  else if (PUSH_BUTTONS_CONFIG == 5)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
    pinMode(IncreaseA3SpeedButton_PIN, INPUT);
    pinMode(DecreaseA3SpeedButton_PIN, INPUT);
    pinMode(IncreaseA4SpeedButton_PIN, INPUT);
    pinMode(DecreaseA4SpeedButton_PIN, INPUT);
  }
  else if (PUSH_BUTTONS_CONFIG == 6)
  {
    pinMode(IncreaseL0SpeedButton_PIN, INPUT);
    pinMode(DecreaseL0SpeedButton_PIN, INPUT);
    pinMode(IncreaseA3SpeedButton_PIN, INPUT);
    pinMode(DecreaseA3SpeedButton_PIN, INPUT);
    pinMode(IncreaseA4SpeedButton_PIN, INPUT);
    pinMode(DecreaseA4SpeedButton_PIN, INPUT);
    pinMode(FirmwareSwitchButton_PIN, INPUT);
  }
  
  // Signal done
  Serial.println("Ready!");
}




// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously

void loop() {
  
  if (firmwareMode == 0)
  {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > manualModeUpdatePeriod)
    {
      previousMillis = currentMillis;
      
      ProcessPushButtons();
    
      // Read serial and send to tcode class
      while (Serial.available() > 0) {
        // Send the serial bytes to the t-code object
        tcode.ByteInput(Serial.read());
      }
    
    
      // Collect inputs
      // These functions query the t-code object for the position/level at a specified time
      // Number recieved will be an integer, 0-9999
      xLin = tcode.AxisRead("L0");
      xRot = tcode.AxisRead("R0");
      yRot = tcode.AxisRead("R1");
      zRot = tcode.AxisRead("R2");
      vibe0 = tcode.AxisRead("V0");
      if (!LUBE_V1) { vibe1 = tcode.AxisRead("V1"); }
      valveCmd = tcode.AxisRead("A0");
      suckCmd = tcode.AxisRead("A1");
      if (LUBE_V1) { lube = tcode.AxisRead("A2"); }
      xComp = tcode.AxisRead("A3");
      yBend = tcode.AxisRead("A4");
    
      // If you want to mix your servos differently, enter your code below:
    
      // Calculate twist position
      float dutyCycle = twistPulseLength;
      dutyCycle = dutyCycle/twistPulseCycle;
      float angPos = (dutyCycle - 0.029)/0.942;
      angPos = constrain(angPos,0,1) - 0.5;
      if (angPos - twistServoAngPos < - 0.8) { twistTurns += 1; }
      if (angPos - twistServoAngPos > 0.8) { twistTurns -= 1; }
      twistServoAngPos = angPos;
      twistPos = 1000*(angPos + twistTurns);
    
      // Calculate valve position
      // Track receiver velocity
      unsigned long t = millis();
      float upVelNow;
      if (t > tLast) {
        upVelNow = xLin - xLast;
        upVelNow /= t - tLast;
        upVel = (upVelNow + 9*upVel)/10;
      }
      tLast = t;
      xLast = xLin;
      // Use suck command if most recent
      boolean suck;
      if (tcode.AxisLast("A1") >= tcode.AxisLast("A0")) {
        suck = true;
        valveCmd = suckCmd;
      } else {
        suck = false;
      }
      // Set valve position
      if (suck) {
        if (upVel < -5) {
          valveCmd = 0;  
        } else if ( upVel < 0 ) {
          valveCmd = map(100*upVel,0,-500,suckCmd,0);
        }
      }
      valvePos = (9*valvePos + map(valveCmd,0,9999,0,1000))/10;
        
      // Calculate arm angles
      // Linear scale inputs to servo appropriate numbers
      int stroke,roll,compression,bend;
      
      if (compressionEnabled && bendEnabled)
      {
        stroke = map(xLin, 0, 9999, tCodeModeStrokeMin, tCodeModeStrokeMax);
        roll   = map(yRot, 0, 9999, RollMinValue, RollMaxValue);
        compression = map(xComp, 0, 9999, CompressionMinValue, CompressionMaxValue);
        bend = map(yBend, 0, 9999, BendMinValue, BendMaxValue);
      }
      else if (compressionEnabled)
      {
        stroke = map(xLin, 0, 9999, tCodeModeStrokeMin, tCodeModeStrokeMax);
        roll   = map(yRot, 0, 9999, RollMinValue, RollMaxValue);
        compression = map(xComp, 0, 9999, CompressionMinValue, CompressionMaxValue);
        bend = 0;
      }
      else if (bendEnabled)
      {
        stroke = map(xLin, 0, 9999, tCodeModeStrokeMin, tCodeModeStrokeMax);
        roll   = map(yRot, 0, 9999, RollMinValue, RollMaxValue);
        compression = 0;
        bend = map(yBend, 0, 9999, BendMinValue, BendMaxValue);
      }
      else
      {
        stroke = map(xLin, 0, 9999, tCodeModeStrokeMin, tCodeModeStrokeMax);
        roll   = map(yRot, 0, 9999, RollMinValue, RollMaxValue);
        compression = 0;
        bend = 0;
      }
  
      int compressionOverlay = 0;
      int bendOverlay = 0;
  
      if (PUSH_BUTTONS_CONFIG == 3 || PUSH_BUTTONS_CONFIG == 4)
      {
        if (compressionEnabled)
        {
          compressionOverlay = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
        }
      }
      else if (PUSH_BUTTONS_CONFIG == 5 || PUSH_BUTTONS_CONFIG == 6)
      {
        if (compressionEnabled && bendEnabled)
        {
          compressionOverlay = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
          bendOverlay = map(manualA4.Value(), 0, 9999, BendMinValue, BendMaxValue);
        }
        else if (compressionEnabled)
        {
          compressionOverlay = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
        }
        else if (bendEnabled)
        {
          bendOverlay = map(manualA4.Value(), 0, 9999, BendMinValue, BendMaxValue);
        }
      }
      
      ledcWrite(LowerLeftServo_PWM, map(LowerLeftServo_ZERO + SafeServoRange(stroke + roll + compression + bend + compressionOverlay + bendOverlay + dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(LowerRightServo_PWM, map(LowerRightServo_ZERO + SafeServoRange(0 - stroke + roll - compression + bend - compressionOverlay + bendOverlay - dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(UpperLeftServo_PWM, map(UpperLeftServo_ZERO + SafeServoRange(stroke + roll - compression - bend - compressionOverlay - bendOverlay - dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(UpperRightServo_PWM, map(UpperRightServo_ZERO + SafeServoRange(0 - stroke + roll + compression - bend + compressionOverlay - bendOverlay + dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
    
      // Twist and valve
      int twist,valve;
      twist  = (xRot - map(twistPos,-1500,1500,9999,0))/5;
      twist  = constrain(twist, -750, 750);
      valve  = valvePos - 500;
      valve  = constrain(valve, -500, 500);
      if (REVERSE_VALVE_SERVO) { valve = -valve; }
      // Set Servos
      ledcWrite(TwistServo_PWM, map(TwistServo_ZERO + twist,0,TwistServo_Int,0,65535));
      ledcWrite(ValveServo_PWM, map(ValveServo_ZERO + valve,0,TwistServo_Int,0,65535));
    
      // Done with servo channels
    
      // Output vibration channels
      // These should drive PWM pins connected to vibration motors via MOSFETs or H-bridges.
      if (vibe0 > 0 && vibe0 <= 9999) {
        ledcWrite(Vibe0_PWM, map(vibe0,1,9999,31,255));
      } else {
        ledcWrite(Vibe0_PWM, 0);
      }
      if (!LUBE_V1 && vibe1 > 0 && vibe1 <= 9999) {
        ledcWrite(Vibe1_PWM, map(vibe1,1,9999,31,255));
      } else {
        ledcWrite(Vibe1_PWM, 0);
      }
      // Vibe timeout functions - shuts the vibne channels down if not commanded for a specified interval
      if (millis() - tcode.AxisLast("V0") > VIBE_TIMEOUT) { tcode.AxisInput("V0",0,'I',500); }
      if (!LUBE_V1 && millis() - tcode.AxisLast("V1") > VIBE_TIMEOUT) { tcode.AxisInput("V1",0,'I',500); }
      
      // Done with vibration channels
    
      // Lube functions
      if (LUBE_V1) {
        if (lube > 0 && lube <= 9999) {
          ledcWrite(Vibe1_PWM, map(lube,1,9999,127,255));
        } else if (digitalRead(Lube_PIN) == HIGH) {
          ledcWrite(Vibe1_PWM,Lube_SPEED);
        } else { 
          ledcWrite(Vibe1_PWM,0);
        }
        if (millis() - tcode.AxisLast("A2") > 500) { tcode.AxisInput("A2",0,' ',0); } // Auto cutoff
      }
    
      // Done with lube
    }
  }
  else if (firmwareMode == 1)
  {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > manualModeUpdatePeriod)
    {
      previousMillis = currentMillis;
      
      ProcessPushButtons();

      // Calculate arm angles
      // Linear scale inputs to servo appropriate numbers
      int stroke,roll,compression,bend;

      if (PUSH_BUTTONS_CONFIG == 1 || PUSH_BUTTONS_CONFIG == 2)
      {
        stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
        roll   = 0;
        compression = 0;
        bend = 0;
      }
      else if (PUSH_BUTTONS_CONFIG == 3 || PUSH_BUTTONS_CONFIG == 4)
      {
        if (compressionEnabled)
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
          bend = 0;
        }
        else
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = 0;
          bend = 0;
        }
      }
      else if (PUSH_BUTTONS_CONFIG == 5 || PUSH_BUTTONS_CONFIG == 6)
      {
        if (compressionEnabled && bendEnabled)
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
          bend = map(manualA4.Value(), 0, 9999, BendMinValue, BendMaxValue);
        }
        else if (compressionEnabled)
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = map(manualA3.Value(), 0, 9999, CompressionMinValue, CompressionMaxValue);
          bend = 0;
        }
        else if (bendEnabled)
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = 0;
          bend = map(manualA4.Value(), 0, 9999, BendMinValue, BendMaxValue);
        }
        else
        {
          stroke = map(manualL0.Value(), 0, 9999, manualModeStrokeMin, manualModeStrokeMax);
          roll   = 0;
          compression = 0;
          bend = 0;
        }
      }
      
      ledcWrite(LowerLeftServo_PWM, map(LowerLeftServo_ZERO + SafeServoRange(stroke + roll + compression + bend + dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(LowerRightServo_PWM, map(LowerRightServo_ZERO + SafeServoRange(0 - stroke + roll - compression + bend - dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(UpperLeftServo_PWM, map(UpperLeftServo_ZERO + SafeServoRange(stroke + roll - compression - bend - dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
      ledcWrite(UpperRightServo_PWM, map(UpperRightServo_ZERO + SafeServoRange(0 - stroke + roll + compression - bend + dualRingReceiverOffsetValue), 0, MainServo_Int, 0, 65535));
    }
  }
}

// Push button processing function
void ProcessPushButtons()
{
  if (PUSH_BUTTONS_CONFIG == 1)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    
    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
  }
  else if (PUSH_BUTTONS_CONFIG == 2)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    int pushButtonFirmwareSwitchState = pushButtonDecreaseL0.State();
    
    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
    
    if (pushButtonFirmwareSwitchState == 1) // Check if the button was pressed
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
  }
  else if (PUSH_BUTTONS_CONFIG == 3)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    int pushButtonIncreaseA3State = pushButtonIncreaseA3.State();
    int pushButtonDecreaseA3State = pushButtonDecreaseA3.State();

    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
    
    if (pushButtonIncreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Increase();
    }
    else if (pushButtonIncreaseA3State == 2) // Check if the button was held
    {
      // Toggle the compression mode
      if (compressionEnabled)
      {
        compressionEnabled = false;
      }
      else
      {
        compressionEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Decrease();
    }
    else if (pushButtonDecreaseA3State == 2) // Check if the button was held
    {
      // Resets the Dual Ring Receiver Offset to the original value specified by OSR2X2_DUAL_RING_RECEIVER_OFFSET
      dualRingReceiverOffsetValue = OSR2X2_DUAL_RING_RECEIVER_OFFSET;
    }
  }
  else if (PUSH_BUTTONS_CONFIG == 4)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    int pushButtonIncreaseA3State = pushButtonIncreaseA3.State();
    int pushButtonDecreaseA3State = pushButtonDecreaseA3.State();
    int pushButtonFirmwareSwitchState = pushButtonDecreaseL0.State();
    
    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
    
    if (pushButtonIncreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Increase();
    }
    else if (pushButtonIncreaseA3State == 2) // Check if the button was held
    {
      // Toggle the compression mode
      if (compressionEnabled)
      {
        compressionEnabled = false;
      }
      else
      {
        compressionEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Decrease();
    }
    else if (pushButtonDecreaseA3State == 2) // Check if the button was held
    {
      // Resets the Dual Ring Receiver Offset to the original value specified by OSR2X2_DUAL_RING_RECEIVER_OFFSET
      dualRingReceiverOffsetValue = OSR2X2_DUAL_RING_RECEIVER_OFFSET;
    }
    
    if (pushButtonFirmwareSwitchState == 1) // Check if the button was pressed
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
  }
  else if (PUSH_BUTTONS_CONFIG == 5)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    int pushButtonIncreaseA3State = pushButtonIncreaseA3.State();
    int pushButtonDecreaseA3State = pushButtonDecreaseA3.State();
    int pushButtonIncreaseA4State = pushButtonIncreaseA4.State();
    int pushButtonDecreaseA4State = pushButtonDecreaseA4.State();
    
    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
    
    if (pushButtonIncreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Increase();
    }
    else if (pushButtonIncreaseA3State == 2) // Check if the button was held
    {
      // Toggle the compression mode
      if (compressionEnabled)
      {
        compressionEnabled = false;
      }
      else
      {
        compressionEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Decrease();
    }
    else if (pushButtonDecreaseA3State == 2) // Check if the button was held
    {
      // Resets the Dual Ring Receiver Offset to the original value specified by OSR2X2_DUAL_RING_RECEIVER_OFFSET
      dualRingReceiverOffsetValue = OSR2X2_DUAL_RING_RECEIVER_OFFSET;
    }
    
    if (pushButtonIncreaseA4State == 1) // Check if the button was pressed
    {
      manualA4.Increase();
    }
    else if (pushButtonIncreaseA4State == 2) // Check if the button was held
    {
      // Toggle the bend mode
      if (bendEnabled)
      {
        bendEnabled = false;
      }
      else
      {
        bendEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA4State == 1) // Check if the button was pressed
    {
      manualA4.Decrease();
    }
  }
  else if (PUSH_BUTTONS_CONFIG == 6)
  {
    int pushButtonIncreaseL0State = pushButtonIncreaseL0.State();
    int pushButtonDecreaseL0State = pushButtonDecreaseL0.State();
    int pushButtonIncreaseA3State = pushButtonIncreaseA3.State();
    int pushButtonDecreaseA3State = pushButtonDecreaseA3.State();
    int pushButtonIncreaseA4State = pushButtonIncreaseA4.State();
    int pushButtonDecreaseA4State = pushButtonDecreaseA4.State();
    int pushButtonFirmwareSwitchState = pushButtonDecreaseL0.State();
    
    if (pushButtonIncreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue -= OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Increase();
      }
    }
    else if (pushButtonIncreaseL0State == 2) // Check if the button was held
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
    
    if (pushButtonDecreaseL0State == 1) // Check if the button was pressed
    {
      if (mainTwoButtonsMode)
      {
        dualRingReceiverOffsetValue += OSR2X2_DUAL_RING_RECEIVER_OFFSET_STEP_VALUE;
      }
      else
      {
        manualL0.Decrease();
      }
    }
    else if (pushButtonDecreaseL0State == 2) // Check if the button was held
    {
      // Toggle the functions of the main two buttons between Increase/Decreasing L0 (x axis) and Increase/Decrease the Dual Ring Receiver Offset value
      if (mainTwoButtonsMode)
      {
        mainTwoButtonsMode = 0;
      }
      else
      {
        mainTwoButtonsMode = 1;
      }
    }
    
    if (pushButtonIncreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Increase();
    }
    else if (pushButtonIncreaseA3State == 2) // Check if the button was held
    {
      // Toggle the compression mode
      if (compressionEnabled)
      {
        compressionEnabled = false;
      }
      else
      {
        compressionEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA3State == 1) // Check if the button was pressed
    {
      manualA3.Decrease();
    }
    else if (pushButtonDecreaseA3State == 2) // Check if the button was held
    {
      // Resets the Dual Ring Receiver Offset to the original value specified by OSR2X2_DUAL_RING_RECEIVER_OFFSET
      dualRingReceiverOffsetValue = OSR2X2_DUAL_RING_RECEIVER_OFFSET;
    }
    
    if (pushButtonIncreaseA4State == 1) // Check if the button was pressed
    {
      manualA4.Increase();
    }
    else if (pushButtonIncreaseA4State == 2) // Check if the button was held
    {
      // Toggle the bend mode
      if (bendEnabled)
      {
        bendEnabled = false;
      }
      else
      {
        bendEnabled = true;
      }
    }
    
    if (pushButtonDecreaseA4State == 1) // Check if the button was pressed
    {
      manualA4.Decrease();
    }
    
    if (pushButtonFirmwareSwitchState == 1) // Check if the button was pressed
    {
      // Switch to the next firmware mode
      if ((firmwareMode + 1) > 1)
      {
        firmwareMode = 0;
      }
      else
      {
        firmwareMode += 1;
      }
    }
  }
}

// Ensure that the servo position is in the absolute minimum to maximum acceptable range
int SafeServoRange(int servoPosition) {
  if (servoPosition > OSR2X2_POSITION_MAX)
  {
    return OSR2X2_POSITION_MAX;
  }
  else if (servoPosition < OSR2X2_POSITION_MIN)
  {
    return OSR2X2_POSITION_MIN;
  }
  else
  {
    return servoPosition;
  }
}

// Twist position detection functions
void twistRising() {
  attachInterrupt(TwistFeedback_PIN, twistFalling, FALLING);
  twistPulseCycle = micros()-twistPulseStart;
  twistPulseStart = micros();
}
void twistFalling() {
  attachInterrupt(TwistFeedback_PIN, twistRising, RISING);
  twistPulseLength = micros()-twistPulseStart;
}
