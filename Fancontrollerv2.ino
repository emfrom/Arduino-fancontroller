/***
This is a FAN controller project using a barebones 8MHz ATMEGA328p to control a fan
In my case a flue gas exhaust fan connected to a rocket mass heater. It's a 12v fan and my powersupply is a LiFePO4 battery bank with a voltage between 26v and 29v.

It uses PWM do drive a buckconverter that powers a fan, without feedback since the load is constant and the input varies only about 10%.
Controling the PWM is direct via registers so it can produce a different pwm frequency than analogWrite() is capable of.
The controller is controlled by one switch, one button and a potentiometer. In my setup the switch and the button is the same physical switch which acts as a button in one direction
and a switch in the other one.

Control flow is implemented as a state machine with interrupts from the pins where the button and switch are connected to change state.
It has three states:
Running permanetly, setting the fan speed based on the potentiometer and nothing mroe
Running on timer, setting the fan speed until a timer expires then go to sleep
Sleep mode, powering down the MCU completely and waking on pin interrupt

It implements reading ADC0 and updating the PWM registry using an interrupt service routine. The ADC is in freewheeling mode and it sets an interrupt after each completed measurement.
ADC0 is arduino pin A0 and pin 23 on the ATMEGA, the potentiometer is wired here. The pot ranges from GND to Vcc. 

In the current configuration the button (button side of my switch) is wired to pin 2 on the arduino, i.e. pin 4 on the ATMEGA, and the switch (switch side of my switch) is
wired to arduino pin 3, pin5 on the ATMEGA. These pins are of course the two interrupt pins, INT0 and INT1. This is necessary to be able to wake the MCU from sleep and go back to buisness.

PWM output is OC1A (Arduino pin 9, ATMEGA pin 15)

I've implemented the hardware both as a low side N MOSFET buck converter (this can output a lot of RFI, be warned) and also using the pwm to drive an IR2104 
(( Use the MOSFET driver, you know you want to! ))


*/
#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega328__) && ! defined(__AVR_ATmega168__)

#error This code only works on the ATMega328P or similar

#endif


#include "Arduino.h"
#include <avr/sleep.h>

/*
 * Global constants
 *
 */

//Debugging stuff
//#define DEBUG

#ifdef DEBUG
#define LOG(a) Serial.print((a))
#else
#define LOG(a)
#endif

//Some basic constants if they havent already been defined
#ifndef TRUE
#if true
#define TRUE true
#else
#define TRUE 1
#endif //if true
#endif //ifndef TRUE

#ifndef FALSE
#if !false
#define FALSE false
#else
#define FALSE 0
#endif //ifndef FALSE
#endif //#if !false

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW 0
#endif

//Cpu frequency adjust
//Used everywhere changing the clock speed of the MCU will have an effect (except debugging statements)
//It should be 16 / Current CPU freq in MHz (so 2 for a bare 8MHz ATMEGA, 1 for arduino)
//this is handy when switching to and from an arduino and a barebones MCU without oscillator
//Currently supports only the values 1 and 2 
#define CPUFREQADJUST 2
#if CPUFREQADJUST != 1 && CPUFREQADJUST != 2
#error Unsupported CPUFREQADJUST
#endif

/* 
 * PIN definitions
 *
 */

//This sets the ADC behaviour
//ADC enabled, Start conversion (i.e. free running mode), Automatic sampling, prescaler depending on CPUFREQADJUST, 128 or 64 (in this order)
//
// IMPORTANT The pot is connected on A0 in the current setup
//            Change the ADMUX bits in setup() for other ports 
//
#define ADCSETUP (bit(ADEN) | bit(ADSC) | bit(ADATE) | bit(ADIE) | bit(ADPS2) | bit(ADPS1) | (CPUFREQADJUST == 2 ? 0 : bit(ADPS0)))


// Output pin that the buck converter is attached to
// Dont change these w/o changing both defines AND all the control registers
#define PWM OCR1A
#define PWMPIN 9

//Pin the timer button is connected to
#define TIMERBUTTONPIN 2

//Pin the power switch is attached to
#define POWERSWITCHPIN 3

//Handy macro to avoid using digital read
//PIND is the register which holds all the input values on port D
//The switch shows up as bit 3 in this bitmask (the rightmost bit is called bit 0)
#define READPOWERSWITCH() ((PIND & 0b00001000) ? HIGH : LOW)


//Shutdown pin of the IR2104
//This pin is shutdown bar (low triggers shutdown). In other words, HIGH to enable and LOW to disable (why not just call it Enable???)
#define IR2104SHUTDOWNPIN 4


/* 
 * PWM definitions
 *
 */

// Ticks per PWM cycle
#define PWMCOUNT 512 / CPUFREQADJUST

// Toggle PWM
// mode : 10 phase correct PWM, TOP=ICR1 
#define PWMON (bit(COM1A1) | bit(WGM11)) 
#define PWMOFF 0

// Different states of running
volatile enum state_enum {
  RP,         //Running permanently (ON power switch)
  RT,         //Running on a timer 
  SM          //Sleep mode
} current_state;


/*
 * Sleep support variables and functions
 *
 */

//Sleep time tells the Timer mode when to turn the fan off
volatile unsigned long sleepTime = 0L;

//Since this sketch will run for more than 50 days(yes a hundred as well for the bare mcu), checking for millis overflow is necessary
volatile bool sleepTimeOverflow = FALSE;


//Sets sleeptime and sleepTimeoverflow
//Argument is the millis value at or after which we will go to sleep
//Is only ever called from ISR so it's marked inline (which the compiler will probably ignore and/or consider superflous)
void __inline__ setSleepTime(long timeToSleep) {
   //timer0 millis is what the millis() function returns
   //We can read it directly as we are called from inside an ISR and so we do not need millis() protection against it changing mid read by a timer interrupt
  extern volatile unsigned long timer0_millis;  

  //Set the target time for the MCU to go to sleep
  sleepTime = timer0_millis + timeToSleep / CPUFREQADJUST; 
      
  //Since we might be running until millis overflows we need to check for that
  if(sleepTime < timer0_millis)
    sleepTimeOverflow = TRUE;
}


/*
 * Interrupt Service Routines
 * 
 * These routines manages button presses and switch changes
 * They both update the current running state to reflect the state of the buttons
 *
 * The ADC vector ISR reads the high byte of the ADC (containg the all the high 8 significant bits of the ADC)
 * It then recalculates this as a pwm value and sets PWM
 */

//Timer Button ISR
//This ISR is called on the falling edge of the pin
//This has to be FALLING not LOW otherwise holding the button down will block all interrupts (tried and tested)
//note, FALLING works for waking up the MCU from sleep
void timerButtonISR() {

  //There are no break; in this switch statement so it will run all the way to the end except where obvious
  //the contents of current state will only change where it starts
  //This is faster, good practice for an ISR, but against a number of "rules" and coding standards. 
  switch(current_state) {
    case SM:
      //Sleep mode, this means we just woke up
      //Disable sleep mode
      sleep_disable();

      // IMPORTANT
      // This is a deliberate fall through to the RP case

    case RP:
      //We hit the button while in running permanently mode
      // or we just woke up and we did both at the same time (or bounce or something)
      
      //If the power button is in ON mode (i.e. LOW) do nothing
      if(READPOWERSWITCH() == LOW) {
        //This returns, it's not a break so be careful about adding stuff after the switch statement
        return;
      }
      
      // IMPORTANT
      // This is a deliberate fall through to the RT case

    case RT:
      //If we came here falling through from RP or SM we set the mode to timer
      current_state = RT;
      //If not, we just hit the button again
      //Can be a bounce but we ignore that for speed since adding a few ms to sleeptime wont matter
      setSleepTime(1800000L);
  }
}

//Power Switch 
//This ISR is called when the powerswitch changes state (Interrupt mode CHANGE)
//Note that the powerbutton can only wake the MCU on a FALLING edge or LOW state
//Electrically the powerswitch has to ground the pin if it's ON state and leave it alone (internal pullups) in OFF state
void powerSwitchISR() {

  //Start by checking the switch since this makes the procedure more readable
  if(READPOWERSWITCH() == LOW) 
    switch(current_state) {
      //The switch is low so it just changed from HIGH to LOW
      //We are turning on (or bouncing)
      
      case SM:
      //Sleep mode
      //We just woke up
      sleep_disable();

      // IMPORTANT
      // This is a deliberate fall through to the RT case

      case RT:
      //User threw the power switch on while in timer mode, turn on

      // IMPORTANT
      // This is a deliberate fall through to the RP case

      case RP:
      //Probably a button bounce, ignore

      //Set the state as we can get here from falling through
      current_state = RP;
     }
  else 
    switch(current_state) {
      //The switch is high so it just changed from LOW to HIGH
      //We are on and we are going to sleep

      case RT:
      //Bounce, or user wants to go from timed mode to power off(and can flip switches real fast)

      // IMPORTANT
      // This is a deliberate fall through to the RP case

      case RP:
        //The powerswitch has been turned off
        //We wait a few sec then turn off (in case the user wants to hit the Timer button)
        setSleepTime(2000);
        current_state = RT;

      // IMPORTANT
      // This is a deliberate fall through to the SM case

      case SM:
        //Sleep mode, should not be possible but who knows what can happen with bouncing buttons
        ;
     }  
}

//We store the read from the ADC so we dont need to update if nothing has changed
volatile byte oldADCH = 0;

//Set the fan speed
//The speed is set by reading a potentiometer connected to A0 and using that value to calculate a pwm value (written to the PWM register)
//Electrically the PWM is used to drive a buckconverter using a MOSFET (or an IR2104 and two MOSFETs) 
//((or whatever, use a bjt for the lols))
//This function gets called whenever the ADC has finished a reading
ISR(ADC_vect) {
  //If nothing has changed we just return
  if(ADCH == oldADCH)
    return;

  //Set old
  oldADCH = ADCH;

  //Calculate and set the PWM
  //NOTE: I use a linear function based on my needs (a 12v fan connected to a battery bank that changes voltage from 26 to 29)
  //There is no feedback, so the load as well as the supply voltage has to be reasonably constant
  //PWMCOUNT figures here because the linear relationship calculated returns a fraction from 0.24 to 0.039
  // so multiplying by PWMCOUNT matches the length of the PWM cycle

  //The two options below are equivalent except the first one uses only integer math
  //Not sure which one is fastest but using floats in an ISR feels icky
  PWM = ((51UL*(unsigned long)oldADCH + 3205UL)*(unsigned long)PWMCOUNT) >> 16;
  
  //This is the formula using floats
  //PWM = (unsigned int)((float)PWMCOUNT)*((0.000764f * oldADCH) + 0.0489f);


}

/*
 * Fan support functions
 *
 */

//Turn the fan completly off
void turnFanOff() {
  //Disable PWM
  TCCR1A = PWMOFF;

  //Set PWMPIN low
  digitalWrite(PWMPIN,LOW);

  //Set the IR2104 shutdown pin LOW, this disables the chip
  digitalWrite(IR2104SHUTDOWNPIN, LOW);

}

//Turn the fan on
void turnFanOn() {
  //We want the fan to move so we turn PWM on
  //(It doesn't matter if it already is)
  TCCR1A = PWMON;

  //Set the IR2104 shutdown pin HIGH, this enables the chip
  //(It doesn't matter if it already is)
  digitalWrite(IR2104SHUTDOWNPIN, HIGH);

}


/*
 * State functions
 * 
 * These functions will each run until the current state changes
 *
 */

// Go to sleep
// Turn the fan off and power down the MCU
void stateSM() {
#ifdef DEBUG
  LOG("State: Sleep Mode\n");
  LOG("............. zzz\n");
  
  //Make sure last log messages gets properly sent
  Serial.flush();
#endif
 
   //Turn the fan off
  turnFanOff();
  
   //Turn the ADC off
  ADCSRA = 0;

  //Turn interrups off for the sleep setup
  //I've seen varying recommendations for this but having them off seems to make the most sense
  noInterrupts();

  //Set power down mode, i.e. everything inside the ATMEGA goes off
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
 
  //Turn on sleep mode capability
  sleep_enable();

  //Deactivate brown out detection
  //If I understand this correctly it will turn itself back on when we exit sleep mode
  sleep_bod_disable();

  //Reenable interrupts so we can wake back up
  interrupts();

  //Go to sleep
  sleep_cpu();

  //
  // IMPORTANT
  //
  // MCU is completely powered down here
  // TODO: Check circuit board so that all points (except the buttons and powerrail) are pulled low while we are down
  //

  //Wake up time
  LOG("\nBack from sleep\n");

  //Reenable the ADC by restoring the ADCSRA
  ADCSRA = ADCSETUP;

  //Sleep has already been disabled in the ISRs so we dont do that here
  return;
}

//Running permanetly mode
// It's the state for when the power switch is in the ON state
void stateRP() {
 LOG("State: Running Permanently\n");
 
  //We want the fan running
  turnFanOn();

  while(current_state == RP) {
#ifdef DEBUG
  static unsigned long lastlog = 0;
  unsigned long time = millis();

  //Log the values once per second or so
  if(time > lastlog + 1000) {
    lastlog = time;
    LOG("millis: ");
    LOG(time);
    LOG("\t\tADCH: ");
    LOG(oldADCH); 
    LOG("\t\tPWM: ");
    LOG(PWM);
    LOG("\n");
  }    
#endif
    delay(25);
  }
    
  //We dont do anything on exit with the PWM values as they might be used in another state (RT specifically)
  return;
}

//Running until timer mode
// Runs until the timer goes out, or state changes for other reasons
void stateRT() {
  LOG("State: Running on timer\n");

  //Start our beloved fan
  turnFanOn();

  //If we had an overflow, add a few more seconds so we dont accidentally miss when millis overflows 
  if(sleepTimeOverflow) {
    sleepTime += 2000L;
  }
  
  while(current_state == RT) {
#ifdef DEBUG
  static unsigned long lastlog = 0;
  unsigned long time = millis();

  //Log the values once every sec
  if(time > lastlog + 1000) {
    lastlog = time;

    LOG("millis: ");
    LOG(time);
    LOG("\t\tADCH: ");
    LOG(oldADCH); 
    LOG("\t\tPWM: ");
    LOG(PWM);
    LOG("\tsleeptime: ");
    LOG(sleepTime);
    LOG("\n");
  }
  
#endif

    //If we had an overflow in sleep time check, we wait until millis has overflowed
    if(sleepTimeOverflow) {
      if(millis() < sleepTime)
        sleepTimeOverflow = FALSE;
    }
    //When millis has counted past sleep time we set sleep mode
    else if(millis() > sleepTime) {
      current_state = SM;
    }

    delay(25);
  }

  //We dont do anything on exit with the PWM values as they might be used in another state
  return;
}

/*
 * ARDUINO functions loop and setup
 * 
 * Doing their arduino thing (as usual)
 */

void setup() {
#ifdef DEBUG
  Serial.begin(38400);
#endif
  LOG("Init staring\n");
 
  //Set PWM pin to output for when it is not in PWM mode
  pinMode(PWMPIN, OUTPUT);
  digitalWrite(PWMPIN,LOW);

  //Set the builtin LED to ouput
  pinMode(LED_BUILTIN, OUTPUT);

  //Set the IR2104 sd_bar pin to output
  pinMode(IR2104SHUTDOWNPIN, OUTPUT);

  //Set the non changing PWM control bits
  //PWM mode 10, phase correct PWM with TOP=ICR and no prescaler (clk/1)
  TCCR1B = bit(WGM13) | bit(CS10);

  //Set the top value of the counter (where it turns around in phase correct mode)
  //This value together with the prescaler sets the PWM frequency i.e. PWMfreq = CPUfreq / prescaler / PWMCOUNT         
  ICR1 = PWMCOUNT - 1;                     

  //Setup the ADC (Analog to Digital converter) for free running mode, it just keeps continously measuring
  ADCSRB = 0;

  //Use port A0
  //left adjust the result, this means the highest 8 bits will all be in ADCH in the right order
  //Use AVcc as the reference voltage
  ADMUX = bit(ADLAR) | bit(REFS0);

  //Disable the digital input buffers on analog pins
  DIDR0 = 0x3F;

  //Set the ADCs first control register to enable
  //Tell it to start samplings
  //Also tell it to keep automatically sampling and to generate an interrupt
  ADCSRA = ADCSETUP;
  
  //Set pin mode for the power switch to internal pullup
  pinMode(POWERSWITCHPIN, INPUT_PULLUP);

  //Wait abit so that we dont get a false reading from the power pin
  delay(2);

  //Set the startup mode based on the power switch
  if(READPOWERSWITCH()) {
    //The pin is high, which means the switch is in off mode so we sleep
    LOG("Startup state is: Sleep mode\n");
    current_state = SM;
  }
  else {
    //The pin is low, the switch is in on mode so we start running the fan
    LOG("Startup state is: Running permanently\n");
    current_state = RP; 
  }

  //Attach the timer button interrupt
  //Important note, the MCU can only wake up from a falling or low state
  //This means that the powerbutton has to be wired to go LOW in the on state and HIGH in the off state
  attachInterrupt(digitalPinToInterrupt(POWERSWITCHPIN), powerSwitchISR, CHANGE);
  

  //Set pinmode for int pin to internal pullup 
  pinMode(TIMERBUTTONPIN, INPUT_PULLUP);

  //Attach the timer button interrupt
  attachInterrupt(digitalPinToInterrupt(TIMERBUTTONPIN), timerButtonISR, FALLING);

  LOG("Init complete\n");
  LOG("########################################\n");
}

void loop() {
  //Main loop calls the appropriate state functions based on the current state variable
  //Change of state occurs only inside of the two ISRs timerButtonISR and powerButtonISR

  //We delay for 50ms since our buttons might bounce and this let's them settle
  delay(50 / CPUFREQADJUST);

  switch(current_state) {
    //Running the fan in the on state
    case RP:
      stateRP();
      break;
      ;;

    //Running the fan until our timer expires
    case RT:
      stateRT();
      break;
      ;;

    //We are going to sleep
    case SM:
      stateSM();
      break;
      ;;
  }
}
