#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <RobotOpen.h>

#include "colors.h"

// #defines
#define PRESSURIZE  (usb1.btnA())
#define ARCADE      (usb1.btnX())
#define TANK        (usb1.btnY())
#define FIRE        (usb1.btnLShoulder() || usb1.btnRShoulder())
#define COMP_OFF	compressor0.off(); compressor1.off();
#define COMP_ON  	compressor0.on(); compressor1.on();

//#define LEDS

#define LIFT_SPEED  (0.3)
#define MAX_PSI     (67.0)
#define MAX_P_V     (5.0)
#define MIN_P_V     (0.0)
#define ON          (0)
#define OFF         (-1)
#define BLINK       (2)
#define BLINKFAST   (4)
#define ERR         (7)

#define THRESHOLD   (3)

// in case we have to invert it
#define SHUTOFF     (compressorShutoff.read())

/* I/O Setup */
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

ROAnalog pressureSensor(0);

ROSolenoid fire0(0);
ROSolenoid fire1(1);
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
  return ((pressure() - targetPSI.get()) > -THRESHOLD);
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
//#define SWITCH
void enabled()
{
#ifdef SWITCH
  switch (count%20)
  {
  case 0:
    {
#endif

#ifdef LEDS
      setLEDs((unsigned long)RGB.get());
#endif

#ifdef SWITCH

    }
    break;

  case 1:
    {
#endif
      // Drive
      int leftPower = 0;
      int rightPower = 0;
      if (tank)
      {
        leftPower = usb1.leftY();
        rightPower = usb1.rightY();
      }
      else
      {
        leftPower =
          constrain(255 - (usb1.rightY() - usb1.rightX() + 127), 0, 255);
        rightPower =
          constrain(255 - (usb1.rightY() + usb1.rightX() - 127), 0, 255);
      }

      leftDriveFront.write(leftPower);
      leftDriveBack.write(leftPower);
      rightDriveFront.write(rightPower);
      rightDriveBack.write(rightPower);
      // Button Handling
      if (TANK)
      {
        tank = true;
      }
      if (ARCADE)
      {
        tank = false;
      }
#ifdef SWITCH

    }
    break;

  case 2:
    {
#endif
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
              ;
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
#ifdef SWITCH

    }
    break;

  case 3:
    {
#endif
      if (FIRE)
      {
        fire0.on();
        fire1.on();
      }
      else
      {
        fire0.off();
        fire1.off();
      }
#ifdef SWITCH

    }
    break;

  case 4:
    {
#endif
      unsigned char val = 127;
      if (usb1.dPadUp())
        val = (float) (127.0 + (LIFT_SPEED * 128.0));
      if (usb1.dPadDown())
        val = (float) (127.0 - (LIFT_SPEED * 128.0));

      lift.write(val);
#ifdef SWITCH

    }
    break;

  default:
    {
      count = 0;
    }
    break;
  }
#endif
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
    //RODashboard.debug("blink");
  }
  else if (!ROStatus.isEnabled() && rateLED != OFF)
  {
    statusLEDSet(OFF);
    //RODashboard.debug("off");
  }
  if (timesUp() && rateLED > 0)
  {
    //RODashboard.debug("invert");
    statusLEDSet(rateLED);
    //    flashLED.queue(1000/rateLED);
  }
  if (led)
    statusLED.on();
  else
    statusLED.off();

  //RODashboard.publish("Compressor", compressor0.read());
  //RODashboard.publish("Status LED", led);
  //RODashboard.publish("Compressor Shutoff", SHUTOFF);
  //RODashboard.publish("D-Pad & lift", val);
  RODashboard.publish("Target Pressure", targetPSI.get());
  RODashboard.publish("At Pressure?", atPressure());
  RODashboard.publish("Pressure", pressure());
  RODashboard.publish("Battery Voltage", ROStatus.batteryReading());
  /*        RODashboard.publish("sizeof char", (int)sizeof(char));
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
  //setLEDs(CYAN);

}


