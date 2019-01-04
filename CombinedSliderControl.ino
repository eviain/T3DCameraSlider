#include "TimerOne.h"
                                                                                                                 
#include <Encoder.h>

#include <stdint.h>

#define DEBUG
#include <TouchScreen.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Fonts/FreeSans9pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans12pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans18pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans24pt7b.h> // Hardware-specific library
#include <Fonts/FreeMono24pt7b.h> // Hardware-specific library
#include <Fonts/FreeMono18pt7b.h> // Hardware-specific library
#include <Fonts/FreeMonoBold18pt7b.h> // Hardware-specific library
#include <Fonts/FreeMonoBold24pt7b.h> // Hardware-specific library


// These are the pins for the shield!
#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 8   // can be a digital pin
#define XP 9   // can be a digital pin

//IM - Touch Screen presure
#define MINPRESSURE 10
#define MAXPRESSURE 10000

//IM - Define the limits of the touch screen 
#define TS_MINX 204
#define TS_MINY 195
#define TS_MAXX 948
#define TS_MAXY 910

//IM - TRINAMIC 2130 STEPPER DRIVER 
#include <SPI.h> // SPI FOR SPI COMMS to the 2130 driver 
#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>


// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// Assign human-readable names to some common 16-bit color values
// Modify these to match the colorspace of your LCD
#define	WHITE   0x0000 //
#define	YELLOW    0x001F //
#define	CYAN     0xF800 //
#define	PURPLE  0x07E0 //
#define RED    0x07FF //
#define GREEN 0xF81F //
#define BLUE  0xFFE0 //
#define BLACK   0xFFFF //



#define PIN_BATTVOLT 15

#define STEPSMM 160

//IM - Pins for the Encoder with button. If the encoders running backwards swap enc0 and enc1 pins
#define PIN_BUTTON 19
#define PIN_ENC0 21 //encoder pin 1
#define PIN_ENC1 20 //encoder pin 2

//IM - Stepper Driver Enable Pin
#define PIN_ENABLE 40
#define PIN_DIR 42
#define PIN_STEP 41
#define PIN_CS 53
#define MOSI_PIN 51
#define MISO_PIN 50
#define SCK_PIN 52
//IM - Step and direction pin settings for the Stepper driver


#define MAX_SPEED  40 // In timer value
#define MIN_SPEED  1000

#define STALL_VALUE 5 // [-64..63]

#define DEBOUNCE 300

#define MINRUNTIME 10000

//number of battery voltage points
#define LOOKUP 20



Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

long day = 86400000; // 86400000 milliseconds in a day
long hour = 3600000; // 3600000 milliseconds in an hour
long minute = 60000; // 60000 milliseconds in a minute
long second =  1000; // 1000 milliseconds in a second

//IM - Instansiate the encoder (on the rotary button) and tell it what pins we have used
Encoder myEnc(PIN_ENC1, PIN_ENC0);

//Config for the Trinamic 2130 set the pins pins 
TMC2130Stepper driver = TMC2130Stepper(PIN_ENABLE, PIN_DIR, PIN_STEP, PIN_CS);

long oldPosition  = 0;
//long MaxSlide = 73235; //IM - Test value for max slide (aprox 1mtr) 
long MaxSlide = 0; //IM - holds the maximum number of steps for the length of the slide rail. Its set during initialisation proccess.
long calSteps = 0; //IM - holds the value of steps whilst performing the calibration.
long RailLength = 0; //IM - holds the value for the lenght of the rail;
float StepsPerMM = 80.3; //IM - the number of steps on my stepper motor that cover 1mm of rail travel.
volatile long numruns = 0;
volatile long period = 10000;

volatile long carragePos = 0; //the absolute carrage position on the slider.

volatile long lastTriggered = 0;

long nextBattMillis = 0;

long runtime = 0;
long offset = 0;
long sspeed = 0;
long decimals = 0;

//IM - Defines the mid point of the touch screen, this will determin the direction of movment, buy pressing either side of this line. 
int ts_MID = 195;

byte oldDays = 0;
byte oldHours = 0;
byte oldMinutes = 0;
byte oldSeconds = 0;

boolean buttonPressed = false; //Holds the value seeing if the button has been pressed by the user
boolean srunning = false; 
boolean sdir = 0; //Bit Direction of travel of the motor
boolean oldsdir = !sdir; //Bit holding the last known Direction of travel of the motor
boolean needsInit = false;

boolean drawnStatusBlank = 1;
boolean drawnStatus = srunning;

boolean driverEnabled = false; //set the default to the stepper driver disabled.
boolean calibratedSlider = false; //sets true if the slider rail has been calibrated to the length.
boolean stall = false; //set the motor stall value to false.
boolean TMC2130enabled = false;
boolean carrageHomed = false; //has the carraged reached its home position (during calibration)
boolean lcdFlipped = false; //has the LCD been installed physically upsidedown?
boolean haltCarrage = false;
boolean countCarrage = true;
boolean carrageHalted = false;
boolean stallSleep = false; //some times we dont want to detect a motor stall, like right after it just happend.

int stallSleepDuration = 1000; 
int calibrationStep = 0; //IM - stores the value of the calibrations step 1 - 3. 3 is fully calibrated and homed.

float rawVolt;
float volt;

int checkInerval = 50; //use as a delay to count down too.

void catchButton() {
  if (lastTriggered + DEBOUNCE < millis()) {
    buttonPressed = true;
    lastTriggered = millis();
    Serial.print("catchButton: Button Pressed");
  }
}

//enable the stepper driver
void enableTMC2130(){
  Serial.println("enableTMC2130: stepper ENABLED");
  digitalWrite(PIN_ENABLE, LOW);
  TMC2130enabled = true;
}

//dissables the stepper driver
void disableTMC2130(){
  Serial.println("disableTMC2130: stepper dissabled");
  digitalWrite(PIN_ENABLE, HIGH);
  TMC2130enabled = false;
}

//IM - VSense for Trinamic 2130
bool vsense;

//IM - RMS current for Trinamic 2130
uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

//IM - Sets up the screen based on its physical mounted positon. Some screens may be mounted upsidedown for physical reasons
//this flips the image if required so the screen contents appear the right way up for the user. Other thigs get messed up when
//the screen is flipped round so we must account for that tool, setting the LCDFlipped bool helps track. 
void SetScreenOrientation(bool flip){

    if(flip){
      tft.setRotation(3);
      lcdFlipped = true; 
    }
    else{
      tft.setRotation(1);
      lcdFlipped = false;
    }
    
}

//##################################### SETUP #####################################
void setup() {

#ifdef DEBUG
  Serial.begin(250000);
  Serial.println(F("TFT LCD test"));
  Serial.print("TFT size is ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());
#endif // DEBUG

  Serial.println("Setup Started");
  Serial.println("Instansiate TMC2130");
  
  //set TMC2130 config
  driver.begin();
  driver.rms_current(600); // mA
  driver.microsteps(16);
  driver.diag1_stall(1);
  driver.diag1_active_high(1);
  driver.coolstep_min_speed(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sg_stall_value(STALL_VALUE);
  driver.stealthChop(1);
  

  Serial.println("Set stepper interrupt");
  // Set stepper interrupt
  
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  //turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bits for 8 prescaler
  TCCR1B |= (1 << CS11);// | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  //sei();//allow interrupts
  

  //TMC2130 outputs on (LOW active)
  disableTMC2130();

  vsense = driver.vsense();

  //IM - SET THE STATE OF ALL THE PINS WE WILL BE USING
  //IM - Set the pins for the stepper driver
  //pinMode(PIN_STEP, OUTPUT); //step pin
  //pinMode(PIN_DIR, OUTPUT); //direction pin
  //pinMode(PIN_ENABLE, OUTPUT); //enable pin

  //IM - Set the initial pin states
  digitalWrite(PIN_BUTTON, HIGH); //IM - set the initial state of the button pin to high
  digitalWrite(PIN_ENABLE, LOW); //IM - set the initial state enable pin for the stepper driver


  tft.reset();

  uint16_t identifier = tft.readID();
  
  if (identifier == 0x9325) {
#ifdef DEBUG
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if (identifier == 0x9328) {

    Serial.println(F("Found ILI9328 LCD driver"));
  } else if (identifier == 0x7575) {

    Serial.println(F("Found HX8347G LCD driver"));
  } else if (identifier == 0x9341) {

    Serial.println(F("Found ILI9341 LCD driver"));
  } else if (identifier == 0x8357) {

    Serial.println(F("Found HX8357D LCD driver"));
#endif // DEBUG
  } else {
#ifdef DEBUG
    /*Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.print(F("I try use ILI9341 LCD driver "));
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));*/
#endif // DEBUG
    identifier = 0x9341;
  }


  tft.begin(identifier);

  //main timer for the slider application
  Timer1.initialize(period);         // initialize timer1, and set a 1/2 second period
  //  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  Timer1.stop();


  // IM - This is the interupt that will fire when the button is pressed
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), catchButton, FALLING);
  
  //##################################### LCD ORIENTATION #####################################
  //IM - Set the screen orientation. True is normal, False upsideown. 
  SetScreenOrientation(true);
 

  tft.fillScreen(BLACK);
  updateLCDTime(true);
  updateLCDStatus();


  rawVolt = ((float) + analogRead(PIN_BATTVOLT) + analogRead(PIN_BATTVOLT) + analogRead(PIN_BATTVOLT) + analogRead(PIN_BATTVOLT)) / 5;
  volt = toVolt(rawVolt);


  //enable the stepper motor.
  enableTMC2130();


  
}

//IM - Run the stepper motor. 
//IM - this runs every half step
void callback()
{
  
  //IM - If the device has been calibrated we can check we dont hit the end of the rail  
  if(calibratedSlider){
  
  //Run the position as a half time
  if(countCarrage){  
    
      //if the direction is false then we are moving away from the home position towards MaxSlide
      if(sdir == true){
       if(carragePos != MaxSlide){
         //if it does then stop
          carragePos++;
        }
       else{
          haltCarrage = true;
       }
     //otherwise were heading for home watch out for 0
      }else{
       if(carragePos != 0){
          carragePos--;
        }else{
          haltCarrage = true;
        }
      
      }

      countCarrage = false;
    }else{
      countCarrage = true;
    }

    //IM - Unless halt carrage is true run the slider
    if(!haltCarrage){
      numruns++;
      digitalWrite(PIN_STEP, digitalRead(PIN_STEP) ^ 1);
    }else{
      if(!carrageHalted){
        Serial.println("############  Carratge halted! ############");
        //stopSlider();
        catchButton(); //IM - emulate the button being pressed.   
        carrageHalted = true;
      }  
    }

  }else //if its not calibrated we'll just hit the buffers. 
  {
      numruns++;
      digitalWrite(PIN_STEP, digitalRead(PIN_STEP) ^ 1);
  }
}

void stopSlider(){
  Serial.println("############  stopSlider! ############");
  carrageHalted = true;
  srunning = 0;
 
  //disableTMC2130();
  
  Timer1.stop();
  
  updateLCDStatus();
  
}




//cDir is ture for left false for right. Unless the screen has been flipped
void carrageDirection(bool cDir){

 
  if(cDir){
      if(oldsdir != cDir){
        digitalWrite(PIN_DIR, 0);
        sdir = true;
        Serial.print("#################################### Carrage Direction: ####################################");
        Serial.println(cDir);
        oldsdir = cDir;
        updateLcdDirection(cDir);
      }
  }
  else
  {
    if(oldsdir != cDir){
      digitalWrite(PIN_DIR, 1);
      sdir = false;
      Serial.print("#################################### Carrage Direction: ####################################");
      Serial.println(cDir);
      oldsdir = cDir;
      updateLcdDirection(cDir);
    }
  }
}

ISR(TIMER1_COMPA_vect){
  PORTF |= 1 << 0;
  PORTF &= ~(1 << 0);
}


//##################################### LOOP #####################################

void loop() {


  
  //##################################### Check Endstop Initiailisation #####################################
  //if the unit has just been switched on then perform a calibration of the rail.
  if(!calibratedSlider){
    Calibrate();
  }else{
  //if the calibration has already been done then 
    MainLoop();
  }
  

}


//calibrate the slider for MAXSlide.
void Calibrate()
{
    //if the carrage hasnt been homed then home it. 
    if(!carrageHomed){
      HomeCarrage();
    }else{
      if(calibrationStep == 0)
      {
        Serial.println("calibration step 1 complete");
        calibrationStep = 1;
      }
    }

    if(calibrationStep == 1){
      //once the carrage is at home send it all the way to the other end of the rail. 
      if(MaxSlide == 0){
        SeekEndStop();  
      }else{
        Serial.println("calibration step 2 complete");
        calibrationStep = 2;
      }
    }
    if(calibrationStep == 2){
       if(carragePos != 0){
         HomeCarrage();
       }else{
        Serial.println("calibration step 3 complete");
         calibrationStep = 3;
         calibratedSlider = true; 
         carrageDirection(true);
       }
    }
}



void SeekEndStop(){
  //rest the stall value

  //IM - only run this part the first time we start this process. 
  if(!TMC2130enabled)
  {
    Serial.println("########### SeekEndStop ###########");
    resetStallSleep();
    enableTMC2130();
  }
  static int cal_last_time = checkInerval;
   
  int StallVal;
  
  //set the carrage direction toward home
  carrageDirection(true);

  if((cal_last_time--) == 0)//IM - dont run every loop the stepper driver wont like it!
  {
    
    //Serial.println("Calibration Mode");
    uint32_t drv_status = driver.DRV_STATUS();
    StallVal = (drv_status & SG_RESULT_bm)>>SG_RESULT_bp;
    //Serial.println(StallVal);

    if(!stallSleep){
      if(StallVal == 0){
        stall = true;
        Serial.println("SeekEndStop: stall!");
      }
    }
    
   cal_last_time = checkInerval;
  }

  if(stallSleep){
    decrementStallSleep();
  }
  
  if(!stall){
      
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(100);
      digitalWrite(PIN_STEP, LOW);
      delayMicroseconds(100);
      calSteps++;
  }else{
      
      if(TMC2130enabled){
        stallSleep = true;
        MaxSlide = calSteps;
        CalcRailLength();
        Serial.print(" ########### Max Slide Found: ");
        Serial.print(MaxSlide);  
        Serial.println(" ########### ");
        carragePos = MaxSlide; //as were at the end of the rail then set carrage pos to maxslide.
        disableTMC2130();
       
      }
  }
  
}

//IM - Calculates the lenght of the rail based on the number of steps taken before it hit the end (MaxSlide), 
//devided by the number of steps taken in one mm (a fixed value up top calculated for our stepper motor). 
void CalcRailLength(){

   //calculate the rail length in MM
  RailLength = MaxSlide / StepsPerMM;
  Serial.print(" ########### Rail Length: ");
  Serial.print(RailLength);  
  Serial.println(" ########### ");
}


//psudo timer, allows some clock cycles to happen to delay a redetection of a motor stall.
void decrementStallSleep(){
  if(stallSleep){

    Serial.print("Stall Sleep:");
    Serial.println(stallSleepDuration);
    
    stallSleepDuration-- ;
    if(stallSleepDuration == 0){
      Serial.println("STALL SLEEP END");
      stallSleep = false;
      stallSleepDuration = 1000; 
    }
  }
}

//IM - if you know your not stalling out then you can reset the stall sleeper.
void resetStallSleep(){
      stall = false;
      stallSleep = false;
      stallSleepDuration = 1000; 
}

void HomeCarrage(){

  if(!TMC2130enabled){
    Serial.println("########### Home Carrage ###########");
    enableTMC2130();
    resetStallSleep();
    Serial.print("Check Interval: ");
    Serial.println(checkInerval); 
    Serial.print("Stall: ");
    Serial.println(stall);    
  }
  
  static int cal_last_time = checkInerval;
   
  int StallVal;
  
  //set the carrage direction toward home
  carrageDirection(false);

  if((cal_last_time--) == 0)//IM - dont run every loop the stepper driver wont like it!
  {
    //Serial.println("HomeCarrage: here");
    //Serial.println("Calibration Mode");
    uint32_t drv_status = driver.DRV_STATUS();
    StallVal = (drv_status & SG_RESULT_bm)>>SG_RESULT_bp;
    //Serial.println(StallVal);

    if(!stallSleep){ 
      if(StallVal == 0){
        stall = true;
        Serial.println("HomeCarrage: Stall!");
        stallSleep = true;
      }
    }
   cal_last_time = checkInerval;
  }

  if(stallSleep){
    decrementStallSleep();
  }
  
  if(!stall){
     
     digitalWrite(PIN_STEP, HIGH);
     delayMicroseconds(100);
     digitalWrite(PIN_STEP, LOW);
     delayMicroseconds(100);
    
  }else{
    
    stallSleep = true; 
    carrageHomed = true;
    carragePos = 0; //IM - Where home!
    if(TMC2130enabled){
      Serial.println(" ########### Homed ############");
      //disableTMC2130();
    }
  }
  
}

void StartStop(){
    
    Serial.println("if button pressed");
    haltCarrage = false;
         
    //if the slider is running then enable or dissable the stepper.
    
     
    if ((sspeed == 0) && (decimals == 0)) {
      srunning = 0;
      Serial.print("######### StartStop: STOPPED #########");
      
    }
    else {
      Serial.print("######### StartStop: STARTED #########");
      if(carrageHalted == true){
        carrageHalted = false;
      }
      srunning = !srunning;
      setPeriod();
      
    }
    updateLCDStatus();
    buttonPressed = false;
}

void MainLoop(){

  static uint32_t last_time=0;
  uint32_t ms = millis();

  //##################################### ENCODER AND BUTTON START #####################################
  long newPosition = myEnc.read();

  if ((newPosition < 0) && (newPosition < offset)) {
    offset = newPosition;
    
  }
  newPosition = newPosition - offset;

  //IM If the new position of the rotary encoder is different then..
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    updateRuntime();
    //Serial.println(newPosition);
    //Serial.print("\n");
    setPeriod();
  }

  //IM - If the interupt has been triggered by the button being pressed, buttonPressed will be true. 
  if (buttonPressed) {
    StartStop();
  }

  //##################################### ENCODER AND BUTTON END #####################################

  //##################################### TOUCH SCREEN START #####################################
  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  //if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
  if (p.z > ts.pressureThreshhold) {

    //IM - Map the touch screen relitive to it size
    p.x = map(p.x, TS_MAXX, TS_MINX, 0, 320);
    p.y = map(p.y, TS_MAXY, TS_MINY, 0, 240);
    
    //IM - Print the y position value
    //Serial.print("X = "); Serial.print(p.x);
    //Serial.print("\tY = "); Serial.print(p.y);
    //Serial.print("\n");

    //a temporary variable to store the new direction
    int DIR = 0;
    
    //IM - If the new point os less than the mid value then its to the left.
    //IM - Note: this depends on physical screen orientation! if it is upside down change < to >.
    if (p.x < ts_MID) {
      
      
      sdir = 0; //IM - Set the direction bit to 0
      //IM - Write to the pin the direction value
      DIR = 0;
      //Serial.println("left");
    }
    else {
      sdir = 1; //IM - Set the direction bit to 1
      //IM - Write to the pin the direction value
      DIR = 1;
      //Serial.println("right");
    }

    //IM - if its a new direction up date the desplay
    carrageDirection(sdir);
    
    if (sdir != oldsdir) { 
 
      updateLCDStatus();
      
    }
    
    
  }

  //##################################### TOUCH SCREEN END #####################################

  //##################################### BATTERY STATUS CHECK START #####################################
 // if ((long)(millis() - nextBattMillis) >= 0) {
    //update Battery status
 //   nextBattMillis += 10000;
 //   rawVolt = ((float)analogRead(PIN_BATTVOLT)) * 0.02 + rawVolt * 0.98;
 //   volt = toVolt(rawVolt);
 //   
 //   tft.setTextColor(WHITE);
//
 //   if (volt < 8) {
  //    tft.setTextColor(GREEN);
  //  }

  //  if (volt < 7.2) {
  //   tft.setTextColor(YELLOW);
  //  }

  // if (volt < 6.5) {
  //    tft.setTextColor(RED);
  //  }

   // tft.fillRect(0, 110, 150, 22, BLACK);
   // tft.setTextSize(1);
   // tft.setCursor(0, 130);
   // tft.setFont(&FreeSans12pt7b);

   // tft.print(mapFloat(volt, 6, 8.4, 0, 100));   tft.print("% ");
   // tft.print("volt");   tft.print("V ");
   // tft.print("rawVolt");   tft.print("int ");
   // Serial.print("Battery Check:"); Serial.print(volt); Serial.print("; "); Serial.println(millis());

 // }
  //##################################### BATTERY STATUS CHECK END #####################################

  
}


void updateLcdDirection(bool d){
  
    if (!d) {

      tft.fillTriangle(
        320, 200, // peak
        280, 240, // bottom left
        320, 240, // bottom right
        BLACK);


      tft.fillTriangle(
        0, 200, // peak
        0, 240, // bottom left
        40, 240, // bottom right
        tft.color565(255, 0, 50));

    }

    else {
      tft.fillTriangle(
        0, 200, // peak
        0, 240, // bottom left
        40, 240, // bottom right
        BLACK);

      tft.fillTriangle(
        320, 200, // peak
        280, 240, // bottom left
        320, 240, // bottom right
        tft.color565(255, 0, 50));
    }
 
}


void updateLCDStatus() {




  if (runtime < MINRUNTIME) {
    if (!drawnStatusBlank) {

      tft.fillRect(75, 200, 170, 35, BLACK);
      srunning = 0;
      drawnStatus = 0;
      drawnStatusBlank = 1;
    }
    return;

  }
  else if (drawnStatusBlank || (drawnStatus != srunning)) {

    tft.fillRect(75, 200, 170, 35, BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0);

    tft.setFont(&FreeSans18pt7b);

    if (srunning) {

      tft.setCursor(75, 230);
      tft.setTextColor(GREEN);
      tft.print("RUNNING");
      
    }

    else {

      tft.setCursor(100, 230);
      tft.setTextColor(BLUE);
      tft.print("READY");
 
    }
    drawnStatus = srunning;
    drawnStatusBlank = 0;
  }
}

//IM Updates the runtime when the rotary encoder is turned. 
void updateRuntime() {
  if (oldPosition != 0) {
    runtime = 1000 * oldPosition * oldPosition / 4;
  }
  if (needsInit) {
    updateLCDTime(true);
  }
  else {
    updateLCDTime(false);
  }

  updateLCDStatus();
}

//IM Updates the LCD with the new run time
void updateLCDTime(boolean firstrun) {

  int days = runtime / day ;                                //number of days
  int hours = (runtime % day) / hour;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  int minutes = ((runtime % day) % hour) / minute ;         //and so on...
  int seconds = (((runtime % day) % hour) % minute) / second;


  tft.setCursor(0, 20);
  tft.setTextSize(4);
  tft.setFont();

  if (oldPosition == 0) {

    tft.setCursor(42, 70);
    tft.setFont(&FreeMonoBold24pt7b);

    tft.setTextSize(2);

    tft.fillRect(0, 5, 320, 28, BLACK); //clear time
    tft.fillRect(0, 54, 320, 35, BLACK); // clear speed

    tft.setTextColor(RED);
    tft.print("HALT");


    srunning = false;


    tft.setTextSize(1);

    tft.setFont(&FreeMono18pt7b);

    tft.setCursor(0, 100);
    tft.setTextColor(WHITE);
    //   tft.print("0 mm/s");

    sspeed = 0;
    decimals = 0;
    needsInit = true;
  }

  else {

    tft.setTextColor(WHITE);

    if (firstrun) {
      tft.fillRect(42, 24, 230, 30, BLACK); //clear HALT/STOP


    }

    tft.setFont(&FreeMonoBold18pt7b);
    tft.setTextSize(1);


    //DAYS

    if ((days != oldDays) || firstrun) {
      tft.setCursor(0, 30);

      tft.fillRect(0, 5, 42, 28, BLACK);
      if (days < 10) {
        tft.print("0");
      }
      tft.print(days);
      oldDays = days;
    }



    //HOURS

    if (firstrun) {

      tft.print("d");
    }


    tft.setCursor(80, 30);

    if ((hours != oldHours) || firstrun) {
      tft.fillRect(80, 5, 42, 28, BLACK);
      if (hours < 10) {
        tft.print("0");
      }
      tft.print(hours);
      oldHours = hours;
    }


    //MINUTES

    if (firstrun) {

      tft.print("h");
    }


    tft.setCursor(160, 30);

    if ((minutes != oldMinutes) || firstrun) {
      tft.fillRect(160, 5, 42, 28, BLACK);
      if (minutes < 10) {
        tft.print("0");
      }
      tft.print(minutes);
      oldMinutes = minutes;
    }


    //SECONDS


    if (firstrun) {

      tft.print("m");

    }


    tft.setCursor(240, 30);

    if ((seconds != oldSeconds) || firstrun) {
      tft.fillRect(240, 5, 42, 28, BLACK);
      if (seconds < 10) {
        tft.print("0");
      }
      tft.print(seconds);
      oldSeconds = seconds;
    }

    if (firstrun) {

      tft.print("s");

      needsInit = false;
    }


    if ((oldPosition + offset) != myEnc.read()) {
      return;
    }


    tft.fillRect(0, 54, 320, 35, BLACK);
    tft.setCursor(0, 80);
    tft.setTextSize(1);
    tft.setFont(&FreeMono18pt7b);
    tft.setTextColor(WHITE);

    //IM INTERESTING.speed = distance over time
    //sspeed = (1000000) / (runtime);
     sspeed = (RailLength*1000) / (runtime);

    if (runtime < (MINRUNTIME * 2)) {
      tft.setTextColor(YELLOW);
    }
    if (runtime < MINRUNTIME) {

      tft.setTextColor(RED);
    }

    tft.print(sspeed);
    tft.print(".");
    decimals = ((1000000000) / (runtime)) - (((1000000) / (runtime)) * 1000);
    //decimals = ((RailLength*1000000) / (runtime)) - (((RailLength*1000) / (runtime)) * 1000);
    
    if (decimals < 100) {
      if (decimals < 10) {
        tft.print("0");
      }
      tft.print("0");
    }

    tft.print(decimals);
    tft.print(" mm/s");
  }
}

//########################### TIMER PERIOD ###############################
void setPeriod() {
  Serial.println("########################### set period ########################### ");
  if ((runtime < MINRUNTIME) | ! srunning) {
    Serial.print("srunning: ");
    Serial.println(srunning);
    //disableTMC2130();
    Timer1.stop();
    Serial.print("setPeriod: Timer Stopped");
    Serial.print("\n");
    
  }

  else {
    //IM - TIMER CODE TO WORKOUT THE RUNNING TIME 
    if(!TMC2130enabled){
      
      enableTMC2130();
    }
    //calculate the speed in mm/s by deviding the rail lenght by the desired runtime in microseconds
    float ssspeed = (RailLength*1000) / ((float) runtime);
    
    float sps = ssspeed * (StepsPerMM);
    float pperiod = (RailLength*1000) / sps;

    Serial.print("RunTime: ");
    Serial.println(runtime);

    Serial.print("ssspeed: ");
    Serial.println(ssspeed,7);

    Serial.print("RailLength: ");
    Serial.println(RailLength*1000);

    Serial.print("sps: ");
    Serial.println(sps,7);

    pperiod = ((float)runtime)/((float)MaxSlide);
 
    
    Serial.print("setPeriod: ");
      
    Serial.println(pperiod);

    Timer1.setPeriod(pperiod*500);
  }
}

float toVolt(float rawADC) {
  return mapFloat(rawADC, 0, 1023, 0, 10.8); //empirical calibration
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float lookup(float inval, short lut[][2], short clamp){
   float out;
   byte i;
   
   for (i=1; i<LOOKUP; i++)
   {
      if (lut[i][0] > inval)
      {
         return lut[i-1][1] + (inval - lut[i-1][0]) * (lut[i][1] - lut[i-1][1]) / (lut[i][0] - lut[i-1][0]);
      }
   }
   
   if (i == LOOKUP){
       return clamp;
   }
}


