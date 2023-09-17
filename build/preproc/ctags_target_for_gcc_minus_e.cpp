# 1 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */

   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */


   /* L298 Motor driver*/



//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h


/* Serial port baud rate */


/* Maximum PWM signal */



# 80 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2




/* Include definition of serial commands */
# 86 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2

/* Sensor functions */
# 89 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2

/* Include servo support if required */






  /* Motor driver function definitions */
# 99 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2

  /* Encoder driver function definitions */
# 102 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2

  /* PID parameters and functions */
# 105 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 2

  /* Run the PID loop at 30 times per second */


  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / 30 /* Hz*/;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */

  long lastMotorCommand = 2000;


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 
# 143 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3 4
       __null
# 143 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
           ;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case 'b':
    Serial.println(57600);
    break;
  case 'a':
    Serial.println(analogRead(arg1));
    break;
  case 'd':
    Serial.println(digitalRead(arg1));
    break;
  case 'x':
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case 'w':
    if (arg2 == 0) digitalWrite(arg1, 0x0);
    else if (arg2 == 1) digitalWrite(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'c':
    if (arg2 == 0) pinMode(arg1, 0x0);
    else if (arg2 == 1) pinMode(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'p':
    Serial.println(Ping(arg1));
    break;
# 199 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
  case 'e':
    Serial.print(readEncoder(0));
    Serial.print(" ");
    Serial.println(readEncoder(1));
    break;
   case 'r':
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case 'm':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case 'o':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK");
    break;
  case 'u':
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(57600);

// Initialize the motor controller if used */


    //set as inputs
    
# 256 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x0A) + 0x20)) 
# 256 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
        &= ~(1<<
# 256 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                2 
# 256 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin 2*/);
    
# 257 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x0A) + 0x20)) 
# 257 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
        &= ~(1<<
# 257 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                3 
# 257 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin 3*/);
    
# 258 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x07) + 0x20)) 
# 258 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
        &= ~(1<<
# 258 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                4 
# 258 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin A4*/);
    
# 259 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x07) + 0x20)) 
# 259 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
        &= ~(1<<
# 259 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                5 
# 259 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin A5*/);

    //enable pull up resistors
    
# 262 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 262 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
         |= (1<<
# 262 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                2 
# 262 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin 2*/);
    
# 263 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 263 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
         |= (1<<
# 263 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                3 
# 263 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin 3*/);
    
# 264 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x08) + 0x20)) 
# 264 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
         |= (1<<
# 264 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                4 
# 264 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin A4*/);
    
# 265 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)((0x08) + 0x20)) 
# 265 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
         |= (1<<
# 265 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                5 
# 265 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                /*pin A5*/);

    // tell pin change mask to listen to left encoder pins
    
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)(0x6D)) 
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
          |= (1 << 
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                   2 
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                   /*pin 2*/)|(1 << 
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                                         3 
# 268 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                                         /*pin 3*/);
    // tell pin change mask to listen to right encoder pins
    
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)(0x6C)) 
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
          |= (1 << 
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                   4 
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                   /*pin A4*/)|(1 << 
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                                          5 
# 270 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                                          /*pin A5*/);

    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
   (*(volatile uint8_t *)(0x68)) 
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
         |= (1 << 
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                  1
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                       ) | (1 << 
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3
                                 2
# 273 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                                      );

  initMotorController();
  resetPID();


/* Attach servos if used */
# 289 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = 
# 303 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3 4
                                  __null
# 303 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                                      ;
      else if (arg == 2) argv2[index] = 
# 304 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3 4
                                       __null
# 304 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                                           ;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = 
# 313 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino" 3 4
                      __null
# 313 "/home/claudio/robot_arduino/kilobot_arduino/kilobot_arduino.ino"
                          ;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > 2000) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }


// Sweep servos






}
# 1 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
# 31 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; //encoder lookup table

  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  
# 36 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino" 3
 extern "C" void __vector_11 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_11 (void)
# 36 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
                  {
   static uint8_t enc_last=0;

 enc_last <<=2; //shift previous state two places
 enc_last |= (
# 40 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino" 3
             (*(volatile uint8_t *)((0x09) + 0x20)) 
# 40 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
                  & (3 << 2)) >> 2; //read the current state into lowest 2 bits

   left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  
# 46 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino" 3
 extern "C" void __vector_10 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_10 (void)
# 46 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
                  {
        static uint8_t enc_last=0;

 enc_last <<=2; //shift previous state two places
 enc_last |= (
# 50 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino" 3
             (*(volatile uint8_t *)((0x06) + 0x20)) 
# 50 "/home/claudio/robot_arduino/kilobot_arduino/encoder_driver.ino"
                  & (3 << 4)) >> 4; //read the current state into lowest 2 bits

   right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == 0) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == 0){
      left_enc_pos=0L;
      return;
    } else {
      right_enc_pos=0L;
      return;
    }
  }




/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(0);
  resetEncoder(1);
}
# 1 "/home/claudio/robot_arduino/kilobot_arduino/motor_driver.ino"
/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
# 59 "/home/claudio/robot_arduino/kilobot_arduino/motor_driver.ino"
  void initMotorController() {
    digitalWrite(12, 0x1);
    digitalWrite(13, 0x1);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;

    if (i == 0) {
      if (reverse == 0) { analogWrite(10, spd); analogWrite(6, 0); }
      else if (reverse == 1) { analogWrite(6, spd); analogWrite(10, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if (reverse == 0) { analogWrite(9, spd); analogWrite(5, 0); }
      else if (reverse == 1) { analogWrite(5, spd); analogWrite(9, 0); }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(0, leftSpeed);
    setMotorSpeed(1, rightSpeed);
  }
# 1 "/home/claudio/robot_arduino/kilobot_arduino/servos.ino"
/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/
