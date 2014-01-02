#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <RobotOpen.h>

#include "colors.h"

/*Button Map
A - pressurize
B - N/A
X - Arcade
Y - Tank
Left Shoulder - Fire
Left Trigger - N/A
Right Shoulder - Fire
Right Trigger - N/A
Left Stick Button - N/A
Right Stick Button - N/A
Start - Shift up
Select - Shift Down
*/

/* Port Assignments
ROPWMs
ROPWM O - leftDriveFront
ROPWM 1 - leftDriveBack
ROPWM 2 - rightDriveFront
ROPWM 3 - rightDriveBack
ROPWM 4 - lift
RODigitalIOs
RODigitalIO 0, output - compressor0
RODigitalIO 1, output -  compressor1
RODigitalIO 2, input - compressorShutoff
RODigitalIO 3, input - limitSwitchTop
RODigitalIO 4, input - limitSwitchBottom
RODigitalIO 5 output - fire
ROAnaglogs
ROAnalog 0 - pressureSensor
ROSolenoids
ROSolenoid 2 - shiftUp
ROSolenoid 3 - shiftDown
ROSolenoid 7 - statusLED
*/

// #defines
#define PRESSURIZE  (usb1.btnA())
#define ARCADE      (usb1.btnX())
#define TANK        (usb1.btnY())
#define FIRE        (usb1.btnLShoulder() || usb1.btnRShoulder())
#define SHIFT_UP    (usb1.btnStart())
#define SHIFT_DOWN  (usb1.btnSelect())
#define COMP_OFF    compressor0.off(); compressor1.off();
#define COMP_ON     compressor0.on(); compressor1.on();

 
//#define LEDS

#define LIFT_SPEED  (0.3)
#define MAX_PSI     (67.0)
#define MAX_P_V     (5.0)
#define MIN_P_V     (0.0)
#define ON          (0)
#define OFF         (-1)
#define BLINK       (2)
#define BLINKFAST   (4)
#define ERR         (9)

#define THRESHOLD   (3)

// in case we have to invert it
#define SHUTOFF     (compressorShutoff.read())

// I/O Setup 
ROJoystick usb1(1);         // Joystick #1

ROPWM leftDriveFront(0);
ROPWM leftDriveBack(1);
ROPWM rightDriveFront(2);
ROPWM rightDriveBack(3);
ROPWM lift(4);
#ifdef LEDS
char red = 28, green = 29, blue = 30;
#endif

RODigitalIO compressor0(0, OUTPUT);
RODigitalIO compressor1(1, OUTPUT);

//#define COMPRESSORSHUTOFFOVERRIDE
RODigitalIO compressorShutoff(2, INPUT);

RODigitalIO limitSwitchTop(3, INPUT);
RODigitalIO limitSwitchBottom(4, INPUT);
RODigitalIO fire(5, OUTPUT);

ROAnalog pressureSensor(0);

ROSolenoid shiftUp(2);
ROSolenoid shiftDown(3);
ROSolenoid statusLED(7);

ROCharParameter targetPSI("Target Pressure", 0);

#ifdef LEDS
ROLongParameter RGB("RGB", BLACK);
#endif

boolean led = false;
boolean tank = true;
boolean compressing = false;

float rateLED = 0;

unsigned long startTime = 0;

unsigned char count = 0;

// !!! DO NOT MODIFY !!!
void loop()
{
  RobotOpen.syncDS();
}

float pressure()
{
  // Max = 1023
  float percent = pressureSensor.read() / 1023.0;
  // adjust for calibration
  float skewed = ((percent * (MAX_P_V - MIN_P_V)) + MIN_P_V) / 5.0;
  // Max PSI is 87
  float psi = skewed * 87.0;
  float bar = skewed * 6.0;
  // change to bar to output in bars
  float out = psi;
  return out;
}

boolean timesUp()
{
  return (millis() - startTime) >= (1000 / rateLED);
}

//possible states:
// ON, BLINK, OFF, or other number to blink x number of times per second
// If negative, it disables the light
// bps = blinks per second
void statusLEDSet(float bps)
{
  //RODashboard.debug(itoa(bps, "   ", 3));
  rateLED = bps;
  rateLED = constrain(rateLED, -1, 127);
  if (rateLED > 0 && timesUp())
  {
    //RODashboard.debug("blink rateLED>0");
    startTime = millis();
    led = ~led;
  }
  else if (rateLED == ON)
  {
    //RODashboard.debug("on");
    led = 1;
  }
  else if (rateLED == OFF)
  {
    //RODashboard.debug("off");
    led = 0;
  }
}

boolean atPressure()
{
  if (compressor0.read() && (pressure() - targetPSI.get()) > THRESHOLD)  return true;
  else if (!compressor0.read() && (pressure() - targetPSI.get()) > -THRESHOLD)  return true;
  else return false;
}

#ifdef LEDS
void setLEDs(unsigned long rgb)
{
  analogWrite(red, 0xFF & ((0x00FF0000 & rgb) >> 4));
  analogWrite(green, 0xFF & ((0x0000FF00 & rgb) >> 2));
  analogWrite(blue, 0xFF & ((0x000000FF & rgb) >> 0));
  RODashboard.publish("RGB", (long)(rgb & 0xFFFFFF));
  RODashboard.publish("R", (long)(0xFF & (rgb >> 4)));        
  RODashboard.publish("G", (long)(0xFF & (rgb >> 2)));        
  RODashboard.publish("B", (long)(0xFF & (rgb >> 0)));        
}
#endif


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled()
{

#ifdef LEDS
  setLEDs((unsigned long)RGB.get());
#endif

  // drive
  int leftPower = 127;
  int rightPower = 127;
  if (tank)
  {
    leftPower = usb1.leftY();
    rightPower = usb1.rightY();
  }
  else
  {
    leftPower =
      constrain(255 - ((usb1.rightX()) - (usb1.rightY()) + 127), 0, 255);
    rightPower = 255 -
      constrain(255 - ((usb1.rightX()) + (usb1.rightY()) - 127), 0, 255);
  }

  //leftDriveFront.write(usb1.leftY());
  leftDriveFront.write(leftPower);
  leftDriveBack.write(leftPower);
  //rightDriveFront.write(usb1.rightY());
  rightDriveFront.write(255-rightPower);
  rightDriveBack.write(255-rightPower);
  
  //switches between tank and arcade
  if (TANK)
  {
    tank = true;
  }
  if (ARCADE)
  {
    tank = false;
  }
  
  //shifts
  if (SHIFT_UP)
  {
    shiftUp.on();
    shiftDown.off();
  }
  else if (SHIFT_DOWN)
  {
    shiftUp.off();
    shiftDown.on();
  }
  
  //compressor handling
  if (PRESSURIZE || !atPressure())
  {
    if (SHUTOFF)
    {
      RODashboard.debug(
      "Either you have exceeded the maximum tank Pressure or the pressure switch"
        "is not connected (to Digital IO 1)");
    }

#ifndef COMPRESSORSHUTOFFOVERRIDE
    if (!((pressure() >= MAX_PSI) || (SHUTOFF)))
#else
      if (!((pressure() >= MAX_PSI)))
#endif
      {
        if (timesUp() || rateLED != BLINKFAST)
        {
          statusLEDSet(BLINKFAST);
        }
        compressing = true;
        COMP_ON
      }
  }
  else
  {
    compressing = false;
    COMP_OFF
  }
  if (!compressing && rateLED == BLINKFAST)
  {
    statusLEDSet(OFF);
  }
  if (SHUTOFF && (PRESSURIZE || compressing) && rateLED != ERR)
  {
    statusLEDSet(ERR);
  }
  else if (SHUTOFF && (!PRESSURIZE && !compressing) && rateLED == ERR)
  {
    statusLEDSet(OFF);
  }
  
  //buttons and valves to fire
  if (FIRE)
  {
    fire.on();
  }
  else
  {
    fire.off();
  }
  //lifts the barrel
  unsigned char val = 127;
  if (usb1.dPadUp())
    val = (float) (127.0 + (LIFT_SPEED * 128.0));
  if (usb1.dPadDown())
    val = (float) (127.0 - (LIFT_SPEED * 128.0));

  //limit switches at top and bottom to limit movement of the barrel
  if (!limitSwitchTop.read() && val < 0)  
    val = 127;
  if (!limitSwitchBottom.read() && val > 0) 
    val = 127;

  lift.write(255-val);
}

/* This is called while the robot is disabled
 * All outputs are automatically disabled (PWM, Solenoid, Digital Outs)
 */
void disabled()
{
  // safety code
  COMP_OFF;
}

/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks()
{
  if (ROStatus.isEnabled() && rateLED <= 0)
  {
    statusLEDSet(BLINK);
  }
  else if (!ROStatus.isEnabled() && rateLED != OFF)
  {
    statusLEDSet(OFF);
  }
  if (timesUp() && rateLED > 0)
  {
    statusLEDSet(rateLED);
   
  }
  if (led)
    statusLED.on();
  else
    statusLED.off();
    
  RODashboard.publish("Target Pressure", (int)targetPSI.get());
  RODashboard.publish("At Pressure?", atPressure());
  RODashboard.publish("Pressure", (int)pressure());
  RODashboard.publish("Battery Voltage", ROStatus.batteryReading());

  RODashboard.publish("Top Limit", limitSwitchTop.read());
  RODashboard.publish("Bottom Limit", limitSwitchBottom.read());
  
/*
  RODashboard.publish("LeftX", usb1.leftY());
  RODashboard.publish("Status LED", led);
  RODashboard.publish("Compressor Shutoff", SHUTOFF);
  RODashboard.publish("D-Pad & lift", val);
 */
 /* RODashboard.publish("sizeof char", (int)sizeof(char));
  RODashboard.publish("sizeof short", (int)sizeof(short));
  RODashboard.publish("sizeof int", (int)sizeof(int));
  RODashboard.publish("sizeof long", (int)sizeof(long));
  RODashboard.publish("sizeof long long", (int)sizeof(long long));
  */
}

void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
  compressorShutoff.pullUp();
  limitSwitchTop.pullUp();
  limitSwitchBottom.pullUp();
  //setLEDs(CYAN);
}


