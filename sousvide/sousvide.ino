/*
*
*  SousVideWith8SegmentDisplays
*
*  Adaptative regulation sous-vide cooker algorithm
*
*  See http://www.instructables.com/id/Cheap-and-effective-Sous-Vide-cooker-Arduino-power/ for more info
*
*  Author : Etienne Giust - 2013
*
*  Features
*
*  - Works out of the box : no need for tweaking or tuning, the software adapts itself to the characteristics of your cooker :  whether it is big, small, full of water, half-full, whether room temperature is low or high, it works.
*  - Efficient regulation in the range of 0.5°C
*  - Sound alarm warns when target temperature is reached
*  - Automatic detection of lid opening and closing : regulation does not get mad when temperature probe is taken out of the water (which is a thing you need to do if you want to actually put food in your cooker)
*  - Safety features : 
*     - automatic cut-off after 5 minutes of continuous heating providing no change in temperature
*     - automatic cut-off after 24 hours of operation
*     - automatic cut-off when temperature reaches 95 °C
*     - allows target temperature only in the safe 50°c to 90°C range 
*  - Dead cheap and simple : no expensive LCD or Solid State Relay
*
*  Updates 2014-07-27
*
*   Switched support of LED Matrix -> 2x16 LCD (still cheap enough)
*       Since we have an LCD
*           On Line 0 print: Debug String, Debug Double, opState
*           On Line 1 print: Water Temp, Goal Temp, Heater Temp
*   Use a TMP36 rather than 1 wire sensor (but TMP36 fluctuates a lot so... trying to source a 1 wire sensor)
*   Since rice cookers heat slowly and cool slowly (due to external heater, and lots of metal that retains heat), we have a huge lag between heater + water temperature change:
*       Added a safety to ensure that heater doesn't blow a thermal fuse during ramp up, waiting for water to change temperature (which it doesn't)
*       No sense in slowly increasing temperature until just 65%, just ramp it all the way to 90%
*       Make heater always within a few degrees of targetWaterTemp especially.  We can do this by monitoring the heater temperature which has a faster and larger response to heating.
*           Things about rice cooker heaters to remember
*               Heater element can cool much futher than water
*               Heater element when much cooler than water can no longer "instantly" heat water
*               Heater element when much hotter than water can no longer stop heating water.
*               Heater element when a little hotter than water still heats water.  So don't leave the heater on until it's 50C over the water since then water can't stop heating.
*       Prevent heater from getting much hotter than water since targetWaterTemp will be overshot a lot due to residual heat in heater
*       TODO: Probably need to keep the heater 2-5 degrees hotter than the targetWaterTemp just to keep heat going in.
*/

// ------------------------- LIBRARIES
#include <LedControl.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#include <LiquidCrystal.h>
LiquidCrystal lcd(2, 4, 5, 6, 7, 8);

// ************************************************
// DiSplay Variables and constants
// ************************************************

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT


byte degree[8] = // define the degree symbol 
{ 
  B00110, 
  B01001, 
  B01001, 
  B00110, 
  B00000,
  B00000, 
  B00000, 
  B00000 
}; 



// ------------------------- PARTS NEEDED

// Arduino board
// integrated 8 digits led display with MAX7219 control module (3 wire interface) 
// Pushbutton x 2
// Piezo element 
// Waterproof DS18B20 Digital temperature sensor
// 4.7K ohm resistor 
// 5V Relay module for Arduino, capable to drive AC125/250V at 10A
// Rice Cooker

// ------------------------- PIN LAYOUT
//
// inputs
// Pushbutton + on pin 6 with INPUT_PULLUP mode
// Pushbutton - on pin 5 with INPUT_PULLUP mode
// Temperature sensor on pin 9 (data pin of OneWire sensor)

// outputs
// Relay on pin 8
// Speaker (piezo) on pin 13
// 8 digit LED display  DataIn on pin 12 
// 8 digit LED display  CLK on pin 11 
// 8 digit LED display  LOAD on pin 10 




// ------------------------- CONSTANTS

  #define UP_BTN 10
  #define DOWN_BTN 12
  #define LEFT_BTN 9
  #define RIGHT_BTN 11
  #define SHIFT_BTN 13  
  
  #define WATER_PIN A5
  #define HEATER_PIN A4
  
  
#define HEATER_STOP_C 95.0
#define HEATER_LIMIT_C 110.0
#define MAX_HEATER_LAG 0.0    // Don't allow heater lag for a rice cooker

// 8 segment display drivers
//#define TEMP_DISPLAY_DRIVER 0
#define DISPLAY_LEFT 4  //left 4 digits of display
#define DISPLAY_RIGHT 0  //right 4 digits of display
#define DISPLAY_CENTER 8

//#define REVERSE_DISPLAY 0 //set to 7 if your displays first digit is on the left

// push-buttons
#define BT_TEMP_MORE_PIN 10 //INPUT_PULLUP mode
#define BT_TEMP_LESS_PIN 12 //INPUT_PULLUP mode

// piezo
#define PIEZO_PIN 13

// temperature sensor
#define ONE_WIRE_BUS 1
#define TEMPERATURE_PRECISION 12
#define SAMPLE_DELAY 500  // Set to 5000 for tmp35 since it fluctuates rapidly, otherwise 500 for more stable DS18B20
#define OUTPUT_TO_SERIAL true

// relay
#define RELAY_OUT_PIN 3
#define RELAY_OUT_PIN_ON LOW
#define RELAY_OUT_PIN_OFF HIGH


// First Ramp
#define FIRST_RAMP_CUTOFF_RATIO 0.90

// Security features
#define MIN_TARGET_TEMP 50   /*sufficient for most sous-vide recipes*/
#define MAX_TARGET_TEMP 90   /*sufficient for most sous-vide recipes*/
#define SHUTDOWN_TEMP 95   /*shutdown if temp reaches that temp*/
#define MAX_UPTIME_HOURS 24   /*shutdown after 24 hours of operation*/
#define MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES 5   /*detect when temp sensor is not in the water and prevent overheating*/

// regulation
#define MIN_SWITCHING_TIME 1500  /* Minimum ON duration of the heating element */
#define DROP_DEGREES_FOR_CALC_REGULATION 0.12 /* minimum drop in degrees used to calculate regulation timings (should be small : <0.2 ) */
#define LARGE_TEMP_DIFFERENCE 1  /* for more than "1" degree, use the Large setting (Small otherwise)*/

// ------------------------- DEFINITIONS & INITIALISATIONS
// JC Vars

int isHeatOn = 0;

// buttons
int sw_tempMore;
int sw_tempLess;

// temperatures
double environmentTemp = 0;
double waterTemp = 0;
double targetWaterTemp = 0;

double heaterTemp = 0;
double tempBeforeHeating = 0;

unsigned long loopDelayTime=100;

// security variables
unsigned long  maxUptimeMillis;
unsigned long  tCheckNotHeatingWildly;
unsigned long tcurrent;
unsigned long nextCaculateHeaterTime = 0;
unsigned long nextSensorUpdateTime = 0;

#define CALCULATE_HEATER_FREQUENCY 2000
#define TOTAL_DUTY_DURATION 10000
unsigned long nextButtonReadTime = 0;

// 7-segment and sensor variables

/*
 LedControl :
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have 1 MAX7219.
*/
//LedControl lc=LedControl(12,11,10,1);
//// Set up a oneWire instance and Dallas temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);	
//// variable to store temperature probe address
// arrays to hold device addresses
DeviceAddress waterThermometer, heaterThermometer;


void turnOffRelay(char reason)
{
  lcd.setCursor(13,0);
  lcd.print("OF");

  lcd.setCursor(15,0);
  lcd.print(" ");
  
  lcd.setCursor(15,0);   
  lcd.print(reason);
  
  digitalWrite(RELAY_OUT_PIN,RELAY_OUT_PIN_OFF);
  isHeatOn = false;
}    

void turnOffRelay()
{
  turnOffRelay('.');
}

void checkHeaterSafety()
{ 
    // Safety Check
    if(heaterTemp > HEATER_STOP_C) {
      turnOffRelay('X');
    } 
}


void turnOnRelay()
{
  lcd.setCursor(13,0);
  lcd.print("ON");

  lcd.setCursor(15,0);
  lcd.print(" ");
  
  

  digitalWrite(RELAY_OUT_PIN,RELAY_OUT_PIN_ON);
  if (isHeatOn == false) {
        tempBeforeHeating = waterTemp;
  }
  isHeatOn = true;
  tCheckNotHeatingWildly = millis() + ((unsigned long)60000 * MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES);
}
    

// ------------------------- SETUP

void setup() {
sensors.begin();


  lcd.begin(16, 2);
  lcd.createChar(1, degree); // create degree symbol from the binary

  //   lcd.setBacklight(VIOLET);
  lcd.setCursor(0, 0);
  lcd.print(F("    Booting..."));
  lcd.setCursor(0, 1);
  lcd.print(F("   Rice Vide!"));

  delay(1000);  // Splash screen
  
  lcd.setCursor(0, 0);
  lcd.print(F("Sensor Count..."));
  lcd.setCursor(0, 1);
  lcd.print(sensors.getDeviceCount());

  delay(400);  // Splash screen

  if (!sensors.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  if (!sensors.getAddress(heaterThermometer, 1)) Serial.println("Unable to find address for Device 1"); 
  
  sensors.setResolution(waterThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(heaterThermometer, TEMPERATURE_PRECISION);
  
	/*
	Initialize pushButtons
	*/
	pinMode(BT_TEMP_MORE_PIN, INPUT_PULLUP);
	pinMode(BT_TEMP_LESS_PIN, INPUT_PULLUP);


	//prepare Relay port for writing 
	pinMode(RELAY_OUT_PIN, OUTPUT);  
	digitalWrite(RELAY_OUT_PIN,RELAY_OUT_PIN_OFF);

	/*
	Initialize temperature sensor
	*/
	sensors.begin();
	sensors.getAddress(waterThermometer, 0);  
	sensors.getAddress(heaterThermometer, 1);  
        sensors.requestTemperatures();
	delay(1000);

        waterTemp = getWaterTempC();
	targetWaterTemp = (long) ((int)waterTemp);
     
	maxUptimeMillis = MAX_UPTIME_HOURS * (unsigned long)3600 * (unsigned long)1000;

turnOffRelay();

  lcd.clear();
 
}



/**************************************************************************************/
/*                                                                                    */
/*                                      MAIN LOOP                                     */
/*                                                                                    */
/**************************************************************************************/


unsigned long onUntil = 0;
int dutyCycle = 0;

void loop() {   
  
  tcurrent = millis();


  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.setCursor(2, 0);
  long seconds = tcurrent / 1000;
  lcd.print(seconds, DEC);
    


  displaytargetWaterTemp(targetWaterTemp);
    
  if (tcurrent > nextCaculateHeaterTime) 
  {
    updateSensorsReadings();
    displaywaterTemp(waterTemp);
    displayHeaterTemp(getHeaterTempC());

    nextSensorUpdateTime = tcurrent + 1000;
    
    
    dutyCycle = percentHeaterOn();
    
        
    lcd.setCursor(10, 0);
    lcd.print("   ");

    lcd.setCursor(10, 0);
    lcd.print(dutyCycle, DEC);
  
  
    onUntil = tcurrent + dutyCycle * (TOTAL_DUTY_DURATION / 100);
    if (dutyCycle > 0) 
    {
      turnOnRelay();
    }
    
    nextCaculateHeaterTime = tcurrent + TOTAL_DUTY_DURATION;
  }

  if (tcurrent > onUntil)
  {
      turnOffRelay();
  }
      
  checkHeaterSafety(); // JC: Turn off heater if it's much hotter than water.
  
  // read buttons state
  
  if (tcurrent > nextButtonReadTime)
  {
    readButtonInputs();
    nextButtonReadTime = tcurrent + 100;
  }
}

#define BOOST_UNTIL_DELTA 10
#define MIN_HEATER_LEAD 1
#define MAX_HEATER_LEAD 20

int percentHeaterOn() {
  int result = 0;
  
  // WAY TOO COLD
  if (waterTemp < targetWaterTemp - BOOST_UNTIL_DELTA)
  {
    // Boost until waterTemp is close
    result = 100;
  } 
  
  // TOO HOT
  else if (waterTemp > targetWaterTemp) 
  {
    if (waterTemp < targetWaterTemp + 1.0) {

      // Don't turn of if water is overtemp, unless heater is behind:
      if (heaterTemp < targetWaterTemp)
      {
        // Don't let the heater fall below targetWaterTemp
        double percentAllowableLagHeat = 1.0 - (waterTemp - targetWaterTemp);
        result = 20 * percentAllowableLagHeat;  
      }
      else 
      {
        result = 0;
      }
    } else {
      // At this point too far over target, it is unacceptable to continue heating 
      result = 0;
    } 
    
  }
  
  // A LITTLE TOO COLD
  else 
  {
    // At this point, water is below target, but not by much
    
    double waterTempLag = targetWaterTemp - waterTemp;
    double heaterTempLead = heaterTemp - targetWaterTemp;
    
    double allowableLead = min(MAX_HEATER_LEAD, waterTempLag + MIN_HEATER_LEAD);
    
    // Do not allow a HUGE heater lead
    if (heaterTempLead < allowableLead)
    { 
      double percentBoostNeeded = (waterTempLag * 1.0) / (BOOST_UNTIL_DELTA * 1.0);
      result = percentBoostNeeded * 100;
      
      if (result < 30) result = 30;
    } 
    else 
    {
      result = 0;
    }
  }
  
  // Safety Check
  if (result) {
    if (heaterTemp > waterTemp + MAX_HEATER_LEAD) {
      result = 0;
    }
  }

  return result;
}


    
// Security checks    
void checkShutdownConditions(){
  boolean doShutdown = false;
  
  // check for too long uptime
  if ( (long) (millis() - maxUptimeMillis) >= 0)
  {
    displayMsg("uptime", maxUptimeMillis);
    doShutdown = true;
  }
  
  // check for too high temperature
  if (heaterTemp > HEATER_LIMIT_C)
  {
    displayMsg("heatSafe", heaterTemp);
    doShutdown = true;
  }
  
  
  if (waterTemp > SHUTDOWN_TEMP)
  {
    displayMsg("h2oSafe", waterTemp);
    doShutdown = true;
  }
  
  // check for too long heating time with no temperature increase (temp probe can't be not trusted anymore so stop the device)
  if (tCheckNotHeatingWildly > 0 && isHeatOn && ( (long) (millis() - tCheckNotHeatingWildly) >= 0))
  {
    if (waterTemp <= tempBeforeHeating)
    {
      // temperature did not increase even if we kept on heating during MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES
      displayMsg("MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES !");
      doShutdown = true;
    }
    // plan next check
    tempBeforeHeating = waterTemp;
    tCheckNotHeatingWildly = millis() + ((unsigned long)60000 * MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES);
  }
  
  if (doShutdown == true)
  {
    shutdownDevice();
  }
}


void shutdownDevice() 
{
   digitalWrite(RELAY_OUT_PIN,RELAY_OUT_PIN_OFF);
   isHeatOn = false;

//    eraseDisplay();
//    displaywaterTemp(0);
//    displaytargetWaterTemp(0);

    if (OUTPUT_TO_SERIAL) {      
        lcd.setCursor(0,1);
        lcd.print("----SHUTDOWN----");
    }
    // turn off relay !

    while(1)
    {
      delay(30000);
    }
}

void readButtonInputs()
{ 
  // read buttons
  sw_tempMore = digitalRead(BT_TEMP_MORE_PIN);
  sw_tempLess = digitalRead(BT_TEMP_LESS_PIN);

  
  // process inputs
  if (sw_tempMore == LOW) { 
    targetWaterTemp= min(targetWaterTemp + 0.5, MAX_TARGET_TEMP);    
  }
  if (sw_tempLess == LOW) {
    targetWaterTemp-=0.5; 
    if (targetWaterTemp < 0) targetWaterTemp = 0;
  }
}



/**************************************************************************************/
/*                                                                                    */
/*                                    UTILITIES                                       */
/*                                                                                    */
/**************************************************************************************/


float ds18b20TemperatureC(DeviceAddress deviceAddress)
{
    float tempC = sensors.getTempC(deviceAddress);
    return tempC;
}


double getWaterTempC()
{
  waterTemp = ds18b20TemperatureC(waterThermometer);
  displaywaterTemp(waterTemp);
  return waterTemp;
}



double getHeaterTempC()
{
  heaterTemp = ds18b20TemperatureC(heaterThermometer);
  displayHeaterTemp(heaterTemp);
  return heaterTemp;
}


void updateSensorsReadings()
{
  sensors.requestTemperatures();
  getWaterTempC();
  getHeaterTempC();
}



void eraseDisplay() {       
    lcd.clear();
}

void displayMsg(char *msg) {
    lcd.setCursor(0,0);
    lcd.print("                 ");
    lcd.setCursor(0,0);
    lcd.print(msg);
}

void displayMsg(char *prefix, double d) {
    lcd.setCursor(0,0);
    lcd.print("                 ");
    lcd.setCursor(0,0);
    lcd.print(prefix);
    lcd.setCursor(8,0);
    lcd.print(d);
}

void displayTemp(float temp, int display_number)
{ 
//  0123456789ABCDEF
//  01.34*67.9*BC.DE*
  int temp100 = temp * 100;
  double tempdot100 = temp100/100.0;
  
  if (display_number == DISPLAY_LEFT) {
    lcd.setCursor(0,1);
    lcd.print(tempdot100);
    lcd.setCursor(5,1);
    lcd.write(1);
  }
  if (display_number == DISPLAY_CENTER) {
    lcd.setCursor(6,1);
    lcd.print(tempdot100);
    lcd.setCursor(10,1);
    lcd.write(1);
  }
  if (display_number == DISPLAY_RIGHT) {
    lcd.setCursor(11,1);
    lcd.print(tempdot100);
//    lcd.setCursor(16,1);
//    lcd.write(1);
  }
}

void displaywaterTemp(float temp)
{
  displayTemp(temp, DISPLAY_LEFT);
}
void displaytargetWaterTemp(float temp)
{
  displayTemp(temp, DISPLAY_CENTER);
}
void displayHeaterTemp(float temp)
{
  displayTemp(temp, DISPLAY_RIGHT);
}

// ------------------------- other UTILITIES

void soundAlarm()
{
  //displayMsg("ALERT");
  for(int index=0;index<3;index++) {
    tone(PIEZO_PIN, 650, 1000);
    //displayMsg("BIIP");
    delay(2000);
  }  
}


