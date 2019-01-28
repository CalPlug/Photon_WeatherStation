  
//===========================================================================
// Description:
//	Photon weather station includes temperature, humidity, pressure, rain in inches,
// wind speed, direction of wind, and ionizing radiation data from UV and geiger counter sensors.
// The Geiger Counter that was interfaced via UART is by Mighty Ohm:  http://mightyohm.com/blog/products/geiger-counter/
// Based on the example code from Haodong Liang who used ThingSpeak, but modified for it also to be 
// used for CloudMQTT. Link to his project: https://www.hackster.io/hliang/thingspeak-weather-station-data-analysis-2877b0.
// The SI1145 driver was developed by Limor Fried of Adafruit:  https://learn.adafruit.com/adafruit-si1145-breakout-board-uv-ir-visible-sensor/
// The TCA9548a driver was developed by Limor Fried of Adafruit:  https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test?view=all
//  with porting to Photon by Rickkas7: https://github.com/rickkas7/TCA9548A-RK  
// The weather shild base project was developed by N. Seidle of SparkFun.
//brownout protection by JVanier: https://community.particle.io/t/eeprom-persistence-issue/16514/39
//other code components used from other sources are cited in-line.
//
// Project Authors:
//	- Sid Kasat, CS Junior @ UC Irvine
//	- Mindy Saylors, EE Junior @ UC Irvine
//  - Jigar Hira, EE Sophomore @ UC Irvine
//  - Michael Klopfer, PhD, CalPlug/Calit2 Technical Director, UC Irvine
//
//Project Managers: Dr. Michael Klopfer, Prof. GP Li.
//California Institute for Telecommunications and Information Technology (Calit2), 2017-2019, (v.1.6)
//University of California, Irvine
//Extended components of project copyright Regents of the Univeristy of California and relesed into the public domain.
//===========================================================================


#include "application.h" //provides brownout protection capabilities - should be first library defined (Built in Photon Library used)

// This #include statement was automatically added by the Particle IDE.  (Built in Photon Library used)

#include <Adafruit_SI1145.h>

// This #include statement was automatically added by the Particle IDE.  (Built in Photon Library used)

#include "SparkFun_Photon_Weather_Shield_Library/SparkFun_Photon_Weather_Shield_Library.h"

// Add math to get sine and cosine for wind vane  (Built in Photon Library used)

#include <math.h>

// This #include statement was automatically added by the Particle IDE.  (Built in Photon Library used)

#include <MQTT.h>

//I2C Multiplexer Control Library  (Built in Photon Library used)

#include <TCA9548A-RK.h>

//***************Required to allow operation on Particle Photon even if connection is down.  Without this the geiger counter will come active and drain batteries when connection drops, for implementation notes, see: https://community.particle.io/t/solved-make-photon-run-code-without-being-necessarily-connected-to-the-particle-cloud/25953/2
SYSTEM_MODE (SEMI_AUTOMATIC)//statement show in examples with and without semicolons in use, note, the sensor diag messages can be missed in this mode!!  See details:  https://docs.particle.io/support/troubleshooting/mode-switching/photon/
SYSTEM_THREAD (ENABLED) //statement show in examples with and without semicolons in use, run in two threads by RTOS, details:  https://docs.particle.io/reference/device-os/firmware/photon/#system-thread

//Wifi Toggle for Photon Cloud reconnect holders 
boolean connectToCloud = false; //In semi-auto mode the connection needs to be started and managed by the user code.  This is the indicator for cloud status connection request
const uint32_t msRetryDelay = 5*60000; // retry every 5min 
const uint32_t msRetryTime  =   30000; // stop trying after 30sec
unsigned long cloudlastcommtime = 0;  //holder for last time value when particle connection was observed
unsigned long cloudreboottimeout = (60*1000)*10; //timout in 10 minutes and reset if no connection is made, try to recconect on next reboot
unsigned long cloudconnectiontestperiod = (60*1000)*5;  //check for conenction every 2 minutes
unsigned int cloudconnectionretryperiodcounter = 0;  //connection time period counter
unsigned long lastcloudconnectretry = 0; //counter to hold millis value to retry particle cloud connection
int cloudconnectretryattemptsallowed = 20; //connection attempts before a reset is performed
long msRetryDelayreconnect = (60*1000)*2; //retry every 2 min
bool retryRunning = false;
Timer retryTimer(msRetryDelay, retryConnect);  // timer to retry connecting
Timer stopTimer(msRetryTime, stopConnect);     // timer to stop a long running try
//*******************

//Initialize I2C Hardware
TCA9548A mux(Wire, 0); //Initiaalize the I2C Multiplexer with default settings
Weather sensor; //Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Adafruit_SI1145 uv = Adafruit_SI1145(); //create object for UV, there is a test at this point but the I2C is not set yet

//Function prototypes
void callback(char* topic, byte* payload, unsigned int length); //Function Prototype for MQTT callback for subscribe functionality

//Interface Ports & Pins
const int GeigerPowerPin = D7; //this is the source for the enable pin (required as inverted) used to turn on the power regulator to supply the geiger counter with power
const int RainPin = D2;
const int AnemometerPin = D3;
const int WindVanePin = A0;

//Loop and timing control
const unsigned int sensorCapturePeriod = 100; //(ms) 0.1 second  // Each time we loop through the main loop, we check to see if it's time to capture the I2C sensor readings
const unsigned int publishPeriod = 10000; //(ms) 10 seconds, 
unsigned int timeNextPublish; // Each time we loop through the main loop, we check to see if it's time to publish the data we've collected, update this value
unsigned int timeNextSensorReading = 100; //(ms) 0.1 second
int ConnectTrysBeforeReset = 20; //number of times to try a connect to MQTT before resetting Photon.  Warning:  This is a kludge to force a reset if no MQTT connection works, set to -1 to disable timeout (not reccommended), an alternate approach to toggle wifi is shown but not activated

//Geiger Counter reading management
unsigned int lastmillis = millis (); //used to manage reporting of loop
unsigned int lastgeigerreporttime = millis (); //used to manage reporting of Geiger Counter values
const unsigned int geigerstart = 1; // run period with reporting for the geiger counter (min)
const unsigned int geigerdelay = 15; // delay for geiger counter (min) after first run
const unsigned int GeigerCounterRun = 1; //time for geiger counter to pre-run before collecting data
const unsigned int GeigerInitialDelay = 120000;  //time in ms to initial geiger preclusion period startup after a restart
unsigned long timeNextGeigerReading = 0;  //inititalize variable - period used for timing the next reading of the geiger counter
unsigned long geigerdelta = 60000;  //period added to geiger counter start in setup to make the new "timeNextGeigerReading" value (ms), added delay (and somewhat redundant to GeigerInitialDelay), needs to be beyond the start point after exclusion period so it doesnt get left behind and not run the sequential if statements required to update value
bool GEIGER_READING = false; //start with the geiger counter set to off
long int geigerreportruncounter = 0; //record for number of times to read from serial port to try to publish
long int geigerreportserialreadruncounter = 0; //record for number of times to read from serial port
int geigerreportdelay = 3100; //checks if the geiger counter reported sucessfully


//Global values for functions

//Temp and Humidity sensor + Pressure Sensor
float humidityRHTotal = 0.0;
unsigned int humidityRHReadingCount = 0;
float tempFTotal = 0.0;
unsigned int tempFReadingCount = 0;
float tempCTotal = 0.0;
unsigned int tempCReadingCount = 0;
float pressurePascalsTotal = 0.0;
float partialpressureH2O = 0.0;
float partialpressureH2Osat = 0.0;
float dewpointC = 0.0;
float dewpointF = 0.0;
unsigned int pressurePascalsReadingCount = 0;
int conntectattempt=0; //attempts for connection retry
int mqttconntectionretries = 35; //try count to test connection to MQTT broker before incrementing a failed test.

//Rain Sensor
volatile unsigned int rainEventCount;
unsigned int lastRainEvent;
const float RainScaleInches = 0.011; // Each pulse is .011 inches of rain

//Windspeed Sensor
float AnemometerScaleMPH = 1.492; // Windspeed if we got a pulse every second (i.e. 1Hz)
volatile unsigned int AnemoneterPeriodTotal = 0;
volatile unsigned int AnemoneterPeriodReadingCount = 0;
volatile unsigned int GustPeriod = UINT_MAX;
unsigned int lastAnemoneterEvent = 0;

//Wind Direction Indicator
float windVaneCosTotal = 0.0;  // For the wind vane, we need to average the unit vector components (the sine and cosine of the angle)
float windVaneSinTotal = 0.0;  // For the wind vane, we need to average the unit vector components (the sine and cosine of the angle)
unsigned int windVaneReadingCount = 0;

//Values for Light/UV/IR Sensor
float lightVisTotal = 0.0;
unsigned int lightVisReadingCount = 0;

float lightUVTotal = 0.0;
unsigned int lightUVReadingCount = 0;

float lightIRTotal = 0.0;
unsigned int lightIRReadingCount = 0;

//Holder for cached readings from sensors
float tempFrec = 0;
float tempCrec = 0;; // *** need to make
//Serial.println("After Temp");
float humidityRHrec = 0;
//Serial.println("After Humidity");
float pressureKParec = 0;
//Serial.println("After Press");
float rainInchesrec = 0;
//Serial.println("After Rain");
float windMPHrec = 0;
float windDegreesrec = 0;
 //Serial.println("After Wind");
float UVIndexrec = 0; //return averaged values for UV (remember to div by 100 for index!)
float visrec = 0; //return averaged values for Vis
float IRrec = 0; //return averaged values for IR



//*************************Functions*******************************
void setup() 
{
  WiFi.selectAntenna(ANT_EXTERNAL); //explicit call for external antenna use on Particle Photon, alternatively:   //WiFi.selectAntenna(ANT_AUTO); //select auto antenna when external one is in use, it will try this)
  Serial.begin(9600); //required for reporting over USB
  Serial1.begin(9600, SERIAL_8N1); //start communication for Geiger Counter input with 9600 bps, 8/N/1 Serial settings (RX and TX pins)
  initializeGeigerCounter();
  Particle.connect(); //run explicit connection to Particle Cloud (required in semi-auto mode), this is the first connection try
  delay (3000); //let sensors boot up and WiFi connection start the process
  connectToCloud = false; //cancel request to try to connect at loop start - keep it as false at this point, this skips check until report
 //Brownout protection for solar input
  printBrownOutResetLevel();
  setBrowoutResetLevel();
  printBrownOutResetLevel();
  delay(100);
  initializeCloudMQTT();
  delay(50);
	mux.begin();
	delay(50);
	mux.setChannel(0); //switch to I2c Bus 0 before initializing the UV sensor
    delay (100);
    initializeUV(); //contains a replacement of uv.begin() without the check for 0x45 on device 0x00 which causes issues with the other sensors.
    delay (50);
	mux.setChannel(1);  //Go to Main I2C bus selection
    delay(50);
    initializeTempHumidityAndPressure();
    delay (50);
    initializeRainGauge();
    initializeAnemometer();
    initializeWindVane();
    delay (50);
    //Stay on Main I2C bus selection
    // Schedule the next sensor reading and publish events
    timeNextSensorReading = millis() + sensorCapturePeriod;
    timeNextPublish = millis() + publishPeriod;
    timeNextGeigerReading = millis() + geigerdelta;
    
}

void printBrownOutResetLevel() //Used to protect code operation if voltage drops too low because of limited solar
{
  Serial.println("Reading BOR");

  uint8_t bor = FLASH_OB_GetBOR();

  switch(bor) {
    case OB_BOR_OFF:
      Serial.println("OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.10 V");  //Debug report to serial
      break;
    case OB_BOR_LEVEL1:
      Serial.println("OB_BOR_LEVEL1: Supply voltage ranges from 2.10 to 2.40 V");
      break;
    case OB_BOR_LEVEL2:
      Serial.println("OB_BOR_LEVEL2: Supply voltage ranges from 2.40 to 2.70 V");
      break;
    case OB_BOR_LEVEL3:
      Serial.println("OB_BOR_LEVEL3: Supply voltage ranges from 2.70 to 3.60 V");
      break;
  }
  Serial.println("");
}

void setBrowoutResetLevel()  //Used to protect code operation if voltage drops too low because of limited solar
{
  const uint8_t desiredBOR = OB_BOR_LEVEL3;
  if(FLASH_OB_GetBOR() != desiredBOR) {
    Serial.println("Writing BOR");

    /* Steps from http://www.st.com/web/en/resource/technical/document/programming_manual/CD00233952.pdf */
    /* See also https://github.com/spark/firmware/blob/aefb3342ed50314e502fc792f673af7a74f536f9/platform/MCU/STM32F2xx/STM32_StdPeriph_Driver/src/stm32f2xx_flash.c#L615 */

    /* To run any operation on this sector, the option lock bit (OPTLOCK) in the Flash option control register (FLASH_OPTCR) must be cleared. */
    FLASH_OB_Unlock();

    /* Modifying user option bytes */
    /* 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register */
    FLASH_WaitForLastOperation();
    /* 2. Write the desired option value in the FLASH_OPTCR register */
    FLASH_OB_BORConfig(desiredBOR);

    /* 3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register */
    FLASH_OB_Launch();

    /* disable the FLASH option control register access (recommended to protect the Option Bytes against possible unwanted operations) */
    FLASH_OB_Lock();

    Serial.println("Done writing BOR");
  } else {
    Serial.println("BOR value already set");
  }
  Serial.println("");
}

//===========================================================
// MQTT Client
//===========================================================
 void callback(char* topic, byte* payload, unsigned int length) // dont need current function for basic operation-- used for subscribe, this is from the example, but can be adapted later
 {
     //loopback test for MQTT subscribe function, currently not used.
     char p[length + 1];
     memcpy(p, payload, length);
     p[length] = NULL;
     if (!strcmp(p, "TEST1"))
     {
         Serial.println("VAL 1 RCVD from MQTT Subscribe Read");
         Particle.publish(String::format("Test Post 1 Read! (%f Min. Runtime)", (millis())/60000));
     }
     else if (!strcmp(p, "TEST2"))
     {
         Particle.publish(String::format("Test Post 2 Read! (%f Min. Runtime)", (millis())/60000));
     }
     else if (!strcmp(p, "TEST3"))
     {
         Particle.publish(String::format("Test Post 3 Read! (%f Min. Runtime)", (millis())/60000));
     }
     else
     {
         //Serial.println("MQTT Subscribe Read: Nothing RCVD");  //Default Case, commented to prevent constant reporting as it is not in use
     }
 }
 MQTT client("XXXX.cloudmqtt.com", 14668, callback);  //NOTE:  Object created after the callback is setup: server, port, type
 
 
 void initializeCloudMQTT() 
 {
     client.connect("XXXXX.cloudmqtt.com", "XXXUSERXXX", "XXXXXPASSXXXXX");  //server, username, password
    // publish/subscribe
        delay (500); //get connection established, don't take too long for timeout
         if (client.isConnected()) 
         {
           client.publish("Weather_Station/CONNECTION_STATUS","Connected"); //Send on connection point to MQTT
           Particle.publish(String::format("MQTT Connection Established! (Published @ %f Min. Runtime)", (float(millis())/60000.0)));
         }
 }


 void publishToMQTT(float tempF, float tempC, float humidityRH, float pressureKPa, float rainInches, float windMPH, float windDegrees, float UV, float vis, float IR) //make sure to do anclient connected check before calling!
 {
    char payload[255];  // To write multiple fields for MQTT Posting
    
    humidityRH = (humidityRH+(25-tempC)*(-0.15));  //corrects errors from calibration point at 25C, from p4 of datasheet
 	
    //Calibration Corrections for Temp, Pressure, Humidity
   

    //Simple Linear RH Calibration, propagates to deriv. calculations
    //Careful!  RH is calculated by temp in the sensor, this relationship may be non linar with respect to calibration performed here!  Alternate approach is shown above, use one or the other!
    float RHGain = 1.0; //1.1493, previously in testing
    float RHOffset = 0;  //9.7717, previously in testing, could be around 50 assuming no change in gain--???, often starts at -2 RH, so guessing its an offset issue
    
    humidityRH = humidityRH*RHGain + RHOffset;
   
    //Derivative Calculations for partial pressure and dewpoint from final values to be reported
    //Datasheet provided relative humidity compensation:
    
     
    float dewpointC = dewpoint(tempC,humidityRH);
    float dewpointF = (dewpointC * 1.8) + 32;
    float partialpressureH2O = vaporpressureH2O(dewpointC);
   
          /*
    //Advanced RH Calibration - Stripping RH down to components then apply calibration to PP H2O values - use raw sensor values to separate, perform this task before applying gain and offset for temp vals, alternate way of correcting RH value
    float PPGain = 1;
    float PPOffset = 0;
    float partialpressureH2Osat = vaporpressureH2Osat(tempC);  //calculation of vapor pressure of water at the current temperature
    partialpressureH2O = partialpressureH2O*PPGain + PPOffset; //apply linear calibration to the PP value (denoted e), this will correct for a linear error in sensor measurement prior to the calculation within the sensor for RH
    humidityRH = (partialpressureH2O/partialpressureH2Osat)*100;  //back calculate RH from e and es, replace RH value with this calibrated update value
    */
   
    //Now that the RH calculations are done, update temperature values with calibration 
    float TCGain = 1;
    float TCOffset = 0;
    tempC = tempC*TCGain + TCOffset;

    float TFGain = 1;
    float TFOffset = 0;
    tempF = tempF*TFGain + TFOffset;
    
    //Assume linear pressure calibration
    float PresGain = 1;
    float PresOffset = 0;
    pressureKPa = pressureKPa*PresGain + PresOffset;
    
    //Report updated values to MQTT
 	snprintf(payload, sizeof(payload), "%0.2f", tempF);
 	client.publish("Weather_Station/Temperature_F", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", tempC);
 	client.publish("Weather_Station/Temperature_C", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", humidityRH);
 	client.publish("Weather_Station/Humidity_RH", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", pressureKPa);
 	client.publish("Weather_Station/Pressure_ KPa", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", dewpointC);
 	client.publish("Weather_Station/DewPoint_C", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", dewpointF);
 	client.publish("Weather_Station/DewPoint_F", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", partialpressureH2O);
 	client.publish("Weather_Station/H2OPartialPressure_ KPa", payload);

 	snprintf(payload, sizeof(payload), "%0.2f", windMPH);
 	client.publish("Weather_Station/WindSpeed_MPH", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", windDegrees);
 	client.publish("Weather_Station/WindDirection_DEG", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.3f", rainInches);
 	client.publish("Weather_Station/RainFall_INCHES_PerRepPeriod", payload); //show with an extra decimal as this is cumulative, this is to avoid roundoff error when totaling

	snprintf(payload, sizeof(payload), "%0.2f", UV);
 	client.publish("Weather_Station/LightUV_INDEX", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", vis);
 	client.publish("Weather_Station/LightVIS_ADC", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", IR);
 	client.publish("Weather_Station/LightIR_ADC", payload);
 	
 	if (GEIGER_READING == false); //When no data is supposed to be present, put in a placeholder.  Do not read in geiger value until the ~2 min mark (100 seconds) when readings are valid, mute by forcing placement of * placeholders during this period
	{
	    client.publish("Weather_Station/GEIGER_OUTPUT", "*");
    }
 	
 	snprintf(payload, sizeof(payload), "%0.2f", float(millis())/60000.0);
 	client.publish("Weather_Station/RUNTIME_MIN", payload);
 	client.loop();  //keepalive for MQTT
 	
 	if (Particle.connected() == true)
       {
            cloudconnectionretryperiodcounter = 0; //reset retry counter for cloud connection
     	    Particle.publish(String::format("MQTTPublish for Senor Reporting: %f MIN Runtime", (float(millis())/60000.0)));
 	        Particle.publish(String::format("Geiger Counter Active in Last Publish?: %d", (GEIGER_READING)));
       }
 }


//===========================================================
// Temp, Humidity and Pressure
//===========================================================
// The temperature, humidity, and pressure sensors are on board
// the weather station board, and use I2C to communicate.  The sensors are read
// frequently by the main loop, and the results are averaged over the publish cycle
void initializeTempHumidityAndPressure() 
{
        sensor.begin();  //Initialize Sensors on Weather Shield for Pressure and Temp/Humidity, Initialize the I2C sensors and ping them
    //Explicit Check of HTU21D (part of sensor.begin()), this may be helpful fpr this sensor only - still looking into it
        #define HTU21D_ADDRESS 0x40
	    uint8_t ID_1;
    	//Check device ID
	    Wire.beginTransmission(HTU21D_ADDRESS);
	    Wire.write(0xFC);
	    Wire.write(0xC9);
	    Wire.endTransmission();
        Wire.requestFrom(HTU21D_ADDRESS,1);
        ID_1 = Wire.read();
        Particle.publish("HTU21D Temp/RH Sensor Type: ", String::format("%u",ID_1)); //Report sensor type value to Particle Console
    //
    
    //Set to Barometer Mode
    sensor.setModeBarometer();
    // Set Oversample rate
    sensor.setOversampleRate(7);
    //Necessary register calls to enble temp, baro and alt
    sensor.enableEventFlags(); 
}


void captureTempHumidityPressure() 
{
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations
    // Measure Temperature in F from the HTU21D or Si7021
    
     // Measure Relative Humidity and following temp values form 3 reads, then compute median from the HTU21D or Si7021, for some reason this value is much more unstable than temp, apply median filter to it
 float humidityRH = sensor.getRH();
 
    // Linear Correction for humidity sensor (to fix calibration issue) is available if needed, this is a non linear response but there is a factor in the library used: RH = -6+125*Srh/2^16 (for any resolution)
  //If the result is reasonable, add it to the running mean
  if (humidityRH > -10 && humidityRH < 150)   //the lower bound should be 0, validity checked after linear correction, negative is nonphysicial, but sometimes read error, once fixed, set min bound to 0
  {
      // Add the observation to the running sum, and increment the number of observations
      humidityRHTotal += humidityRH;
      humidityRHReadingCount++;
  }
  delay (1);
  
  //Need to implement median for temp sensors, right now based off the last read
  float tempF = sensor.readTempF(); //note there is a diff between readTemp and getTemp, the latter is a unique read!
  //If the result is reasonable, add it to the running mean
  if(tempF > -50 && tempF < 150)
  {
      // Add the observation to the running sum, and increment the number of observations
      tempFTotal += tempF;
      tempFReadingCount++;
  }
  delay (1);
  
   // Measure Temperature in C from the HTU21D, in this not the Si7021
  float tempC = sensor.readTemp(); //yes, you can convert, yes, reading the sensor again is totally redundant
  //If the result is reasonable, add it to the running mean
  if(tempC > -65 && tempC < 65) 
  {
      // Add the observation to the running sum, and increment the number of observations
      tempCTotal += tempC;
      tempCReadingCount++;
  }
 delay (1);

  //Measure Pressure from the MPL3115A2
  float pressurePascals = sensor.readPressure();

  //If the result is reasonable, add it to the running mean
  // What's reasonable? http://findanswers.noaa.gov/noaa.answers/consumer/kbdetail.asp?kbid=544
  if(pressurePascals > 80000 && pressurePascals < 110000)
  {
      // Add the observation to the running sum, and increment the number of observations
      pressurePascalsTotal += pressurePascals;
      pressurePascalsReadingCount++;
  }
  return;
}


float getAndResetTempF()
{
    if(tempFReadingCount == 0) 
    {
        return 0;
    }
    float result = tempFTotal/float(tempFReadingCount);
    tempFTotal = 0.0;
    tempFReadingCount = 0;
    return result;
}


float getAndResetTempC()
{
    if(tempCReadingCount == 0) 
    {
        return 0;
    }
    float result = tempCTotal/float(tempCReadingCount);
    tempCTotal = 0.0;
    tempCReadingCount = 0;
    return result;
}


float getAndResetHumidityRH()
{
    if(humidityRHReadingCount == 0) 
    {
        return 0;
    }
    float result = humidityRHTotal/float(humidityRHReadingCount);
    humidityRHTotal = 0.0;
    humidityRHReadingCount = 0;
    return result;
}


float getAndResetPressurePascals()
{
    if(pressurePascalsReadingCount == 0) 
    {
        return 0;
    }
    float result = pressurePascalsTotal/float(pressurePascalsReadingCount);
    pressurePascalsTotal = 0.0;
    pressurePascalsReadingCount = 0;
    return result;
}


float dewpoint(float TempC, float RH)
{
    float dp= 243.04*(log(RH/100)+((17.625*TempC)/(243.04+TempC)))/(17.625-log(RH/100)-((17.625*TempC)/(243.04+TempC)));
    return dp;
}


float vaporpressureH2O(float dpC)
{
    float e= 6.11*pow(10,((7.5*dpC)/(237.3 + dpC)));  //actual vapor pressure  (10^x)
    return e*10;  //convert hPa to KPa to report
}


float vaporpressureH2Osat(float TempC)
{
    float es= 6.11*pow(10,((7.5*TempC)/(237.3 + TempC)));  //saturated vapor pressure  (10^x)
    return es*10;  //convert hPa to KPa to report
}


//===========================================================================
// Rain Guage
//===========================================================================
void initializeRainGauge() 
{
  pinMode(RainPin, INPUT_PULLUP); //pullup is default state, connection made on rain event.
  rainEventCount = 0;
  lastRainEvent = 0;
  attachInterrupt(RainPin, handleRainEvent, FALLING); //trigger on pulse falling edge as bucket dumps
  return;
}
 
  
void handleRainEvent() 
{
    // Count rain gauge bucket tips as they occur
    // Activated by the magnet and reed switch in the rain gauge, attached to input D2
    unsigned int timeRainEvent = millis(); // grab current time
    // ignore switch-bounce glitches less than 10mS after initial edge, this is a max top end value
    if(timeRainEvent - lastRainEvent < 10) {
      return;
    }
    
    rainEventCount++; //Increase this minute's amount of rain
    lastRainEvent = timeRainEvent; // set up for next event
}


float getAndResetRainInches()
{
    float temprainevent = ((float) rainEventCount); // this seems OK now, there may be a need if there is a remaining issue to perform a pointercast from unsigned to float to deal with lost bits with normal typecasting, see this: https://stackoverflow.com/questions/49458917/casting-from-unsigned-int-to-float, // see getAndResetAnemometerMPH() function and usage for a similar approach passing pointers, understand on some devices there can be an endian issue without memcop.  //float temprainevent = *((float*)&rainEventCount); //perform a pointercast from unsigned to float to deal with lost bits with normal typecasting, see this: https://stackoverflow.com/questions/49458917/casting-from-unsigned-int-to-float, // see getAndResetAnemometerMPH() function and usage for a similar approach passing pointers, understand on some devices there can be an endian issue without memcopy
    float result = -1; //set with fail value as default
    result = RainScaleInches * temprainevent;
    rainEventCount = 0; //reset global value as counter for bucket tips
    return result;
}


//===========================================================================
// Wind Speed (Anemometer)
//===========================================================================

// The Anemometer generates a frequency relative to the windspeed.  1Hz: 1.492MPH, 2Hz: 2.984MPH, etc.
// We measure the average period (elaspsed time between pulses), and calculate the average windspeed since the last recording.
void initializeAnemometer() 
{
  pinMode(AnemometerPin, INPUT_PULLUP);  
  AnemoneterPeriodTotal = 0;
  AnemoneterPeriodReadingCount = 0;
  GustPeriod = UINT_MAX;  //  The shortest period (and therefore fastest gust) observed
  lastAnemoneterEvent = 0;
  attachInterrupt(AnemometerPin, handleAnemometerEvent, FALLING);  // on falling edge, the rain bucket indicates volume has been collected and dumped, record at this point
  return;
  }
  
  
void handleAnemometerEvent() 
{
    // Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
     unsigned int timeAnemometerEvent = millis(); // grab current time
     
    //If there's never been an event before (first time through), then just capture it
    if(lastAnemoneterEvent != 0) 
    {
        // Calculate time since last event
        unsigned int period = timeAnemometerEvent - lastAnemoneterEvent;
        // ignore switch-bounce glitches less than 10mS after initial edge (which implies a max windspeed of 149mph)
        if(period < 10) {
          return;
        }
        if(period < GustPeriod) {
            // If the period is the shortest (and therefore fastest windspeed) seen, capture it
            GustPeriod = period;
        }
        AnemoneterPeriodTotal += period;
        AnemoneterPeriodReadingCount++;
    }
    
    lastAnemoneterEvent = timeAnemometerEvent; // set up for next event
}


float getAndResetAnemometerMPH(float * gustMPH)
{
    if(AnemoneterPeriodReadingCount == 0)
    {
        *gustMPH = 0.0;
        return 0;
    }
    // Nonintuitive math:  We've collected the sum of the observed periods between pulses, and the number of observations.
    // Now, we calculate the average period (sum / number of readings), take the inverse and muliple by 1000 to give frequency, and then mulitply by our scale to get MPH.
    // The math below is transformed to maximize accuracy by doing all muliplications BEFORE dividing.
    float result = AnemometerScaleMPH * 1000.0 * float(AnemoneterPeriodReadingCount) / float(AnemoneterPeriodTotal);
    AnemoneterPeriodTotal = 0;
    AnemoneterPeriodReadingCount = 0;
    *gustMPH = AnemometerScaleMPH  * 1000.0 / float(GustPeriod);
    GustPeriod = UINT_MAX;
    return result;
}


//===========================================================
// Wind Vane
//===========================================================
void initializeWindVane() 
{
    return;
}


void captureWindVane()
{
    // Read the wind vane, and update the running average of the two components of the vector
    unsigned int windVaneRaw = analogRead(WindVanePin);
    
    float windVaneRadians = lookupRadiansFromRaw(windVaneRaw);
    if(windVaneRadians > 0 && windVaneRadians < 6.14159)
    {
        windVaneCosTotal += cos(windVaneRadians);
        windVaneSinTotal += sin(windVaneRadians);
        windVaneReadingCount++;
    }
    return;
}


float getAndResetWindVaneDegrees()
{
    if(windVaneReadingCount == 0) 
    {
        return 0;
    }
    float avgCos = windVaneCosTotal/float(windVaneReadingCount);
    float avgSin = windVaneSinTotal/float(windVaneReadingCount);
    float result = atan(avgSin/avgCos) * 180.0 / 3.14159; //should use atan2 to get quadrant
    windVaneCosTotal = 0.0;
    windVaneSinTotal = 0.0;
    windVaneReadingCount = 0;
    // atan can only tell where the angle is within 180 degrees.  Need to look at cos to tell which half of circle we're in
    if(avgCos < 0) result += 180.0;
    // atan will return negative angles in the NW quadrant -- push those into positive space.
    if(result < 0) result += 360.0;
    
   return result;
}


float lookupRadiansFromRaw(unsigned int analogRaw)
{
    // The mechanism for reading the weathervane isn't arbitrary, but effectively, we just need to look up which of the 16 positions we're in.
    if(analogRaw >= 2200 && analogRaw < 2400) return (3.14);//South
    if(analogRaw >= 2100 && analogRaw < 2200) return (3.53);//SSW
    if(analogRaw >= 3200 && analogRaw < 3299) return (3.93);//SW
    if(analogRaw >= 3100 && analogRaw < 3200) return (4.32);//WSW
    if(analogRaw >= 3890 && analogRaw < 3999) return (4.71);//West
    if(analogRaw >= 3700 && analogRaw < 3780) return (5.11);//WNW
    if(analogRaw >= 3780 && analogRaw < 3890) return (5.50);//NW
    if(analogRaw >= 3400 && analogRaw < 3500) return (5.89);//NNW
    if(analogRaw >= 3570 && analogRaw < 3700) return (0.00);//North
    if(analogRaw >= 2600 && analogRaw < 2700) return (0.39);//NNE
    if(analogRaw >= 2750 && analogRaw < 2850) return (0.79);//NE
    if(analogRaw >= 1510 && analogRaw < 1580) return (1.18);//ENE
    if(analogRaw >= 1580 && analogRaw < 1650) return (1.57);//East
    if(analogRaw >= 1470 && analogRaw < 1510) return (1.96);//ESE
    if(analogRaw >= 1900 && analogRaw < 2000) return (2.36);//SE
    if(analogRaw >= 1700 && analogRaw < 1750) return (2.74);//SSE
    if(analogRaw > 4000) return(-1); // Open circuit?  Probably means the sensor is not connected
    Particle.publish("error", String::format("Got %d from Windvane.",analogRaw), 60 , PRIVATE);
    return -1;
}


//===========================================================================
// Geiger Counter 
//===========================================================================

// Notes:
// 
//	Geiger Values are submitted to through TX RX via the Serial1 USART connection
//	FORMAT of CSV: CPS, ####, CPM, ####, uSv/hr, ##.##, SLOW/FAST/INST
//
//	Be sure to read only if Serial1.available() is true
//
// Fix:
//	Need to have mode_speed to remain constant for reading to be sure averaging
//	cps, cpm, uSv/hr values make sense
//
//	Need to get captureGeigerValues() to keep taking in values until <I say it needs to>
//When no value is vailable, only * is returned
void initializeGeigerCounter()
{
	//Serial1.begin(9600);    	//Baud rate: 9600 - initalize above in setup
	//Serial1.begin(9600, SERIAL_9N1); // via TX/RX pins, 9600 9N1 mode (shown as config example, see )https://docs.particle.io/reference/device-os/firmware/photon/#serial
	pinMode(GeigerPowerPin, OUTPUT);
	digitalWrite(GeigerPowerPin, HIGH); //turn off geiger counter read with HIGH (ivnverse enable on the controller)
	return;
}


void captureGeigerValues()
   { //Captures values (caution: uses a while loop which may disrupt timing)
   
   //Serial.printlnf("STARTED");
   
    int arrlen=50; //read buffer array length
    int q=0; //characters read in from accumulated serial buffer, start count at 0
    
    //Initialize read array:
	char test[arrlen];
	test[0]='*'; //check character at array start
	for (int i=1;i<arrlen;i++)
    	{
	     test[i]='\0';
	    }
	  
	long int mil = millis(); 
	bool escape = false;
	
	while(Serial1.read() >= 0); //Flush buffer fully by reading it out and doing nothing with it, this is quick, try to flush to get a clean start for next run
	   
	do
	{
        //Read in the accumulated serial buffer
        while (q<arrlen && Serial1.available()>0) //read in and replace the * and null characters
            {
            char c = Serial1.read(); //read in characters one by one
    	    test[q] = c;
    	    
    	    if (c == 'W' || c == 'T')
    	    {
    	        escape = true;
    	    }
    	    
            q++; //increment array index
            geigerreportserialreadruncounter = geigerreportserialreadruncounter+1;
    	    }
    	    
    	    
	} while (((test[0] == 'C') && (q<45) && (escape == false)) || (((mil + 150) > millis()) && (millis() > 20000))); //timeout protected from millis overrun
	
    //check to see if the buffer has valid data in it, see if characters read in are valid to report
	if (q>30 && (test[0]=='C'|| test[0]=='*') ) // PREV: i>10 (or 1>30) && (test[0]=='C' || test[0]=='*')check for min length for valid response, and if the C (for CPM, the first characters of the response) or * characters are present indicating a valid return
	    {
	        
	        //Serial.printlnf("%s", test); /* SERIAL TEST */
	   
	   //                          vvv There is a race condition here. test is published successfully w/out the time check
	   //                          vvv                                 test gets overwritten w/ the check
	    if (client.isConnected()) /*&& (lastgeigerreporttime + geigerloopdelay < millis())*/ //Only report if new data is collected and the client is available, the geiger reports every second, so this should catch it, once per reading and restrict to reporting every 5 seconds
            {
   		        client.publish("Weather_Station/GEIGER_OUTPUT",test);
	            char payload[255];
	            snprintf(payload, sizeof(payload), "%0.2f", float(millis())/60000);
                client.publish("Weather_Station/RUNTIME_MIN", payload);
                Particle.publish(String::format("MQTTPublish for Geiger Counter@ %f MIN Runtime", float((millis())/60000.0)));
                lastgeigerreporttime = millis();
            }
            //the checking will be done by the other periodic function, if this fails, the other function will catch and retry a reconnect
        Particle.publish(String::format("Valid Geiger Counter Reading @ %f MIN Runtime", float((millis())/60000.0)));
	    }
    geigerreportruncounter = geigerreportruncounter+1;
    
    }

/*  //Legacy geiger counter function version, tested and known functional
void captureGeigerValues()
{ //Captures values (caution: uses a while loop which may disrupt timing)
 bool dataIsNotCollected = true;		// to collect data for one line of data

	do
	{
	    //bool newdata=false;
		int arrlen=50;
		char test[arrlen];
		test[0]='*';
		for (int i=1;i<arrlen;i++)
		{
		    test[i]='\0';
		}

		int i=0;
	    while (i<arrlen && Serial1.available())
	    {
	        char c= Serial1.read();
		    test[i]=c;
	        i++;
		}
        	if (i>26 && (test[0]=='C' || test[0]=='*')) //check for min length for valid response, and if the C (for CPM, the first characters of the response) or * characters are present indicating a valid return
        	{
        	    if (client.isConnected() && (lastmillis + geigerloopdelay < millis())) //Only report if new data is collected and the client is available, the geiger reports every second, so this should catch it, once per reading and restrict to reporting every 5 seconds
                    {
           		        client.publish("Calit2_Weather_Station/GEIGER_OUTPUT",test);
        	            char payload[255];
        	            snprintf(payload, sizeof(payload), "%0.2f", float(millis())/60000);
                         client.publish("Calit2_Weather_Station/RUNTIME_MIN", payload);
                         Particle.publish(String::format("MQTTPublish@ %f MIN Runtime", (millis())/60000));
                         lastmillis = millis (); //set new value for lastmillis, this is used to meter the rate of the loop
                    }
        	}
		dataIsNotCollected = false;
	} while (dataIsNotCollected);
}
*/


//*****************Equivelant of uv.begin() in the Adafruit SI1145 library, but without the restart of wire.begin() and the initial test read check for 0x45 from 0x00*****************
bool initializeUV()
{
    //I2C SI1145 UV Sensor Parameters - from library, this has to be in here as the initialization of the library causes an Issue witha  device conflict before the I2C switch is initialized.  Rather than forking the library, we introduced some of the library functions into the main code here:
    /* COMMANDS */
    #define SI1145_PARAM_QUERY 0x80
    #define SI1145_PARAM_SET 0xA0
    #define SI1145_NOP 0x0
    #define SI1145_RESET    0x01
    #define SI1145_BUSADDR    0x02
    #define SI1145_PS_FORCE    0x05
    #define SI1145_ALS_FORCE    0x06
    #define SI1145_PSALS_FORCE    0x07
    #define SI1145_PS_PAUSE    0x09
    #define SI1145_ALS_PAUSE    0x0A
    #define SI1145_PSALS_PAUSE    0xB
    #define SI1145_PS_AUTO    0x0D
    #define SI1145_ALS_AUTO   0x0E
    #define SI1145_PSALS_AUTO 0x0F
    #define SI1145_GET_CAL    0x12
    
    /* Parameters */
    #define SI1145_PARAM_I2CADDR 0x00
    #define SI1145_PARAM_CHLIST   0x01
    #define SI1145_PARAM_CHLIST_ENUV 0x80
    #define SI1145_PARAM_CHLIST_ENAUX 0x40
    #define SI1145_PARAM_CHLIST_ENALSIR 0x20
    #define SI1145_PARAM_CHLIST_ENALSVIS 0x10
    #define SI1145_PARAM_CHLIST_ENPS1 0x01
    #define SI1145_PARAM_CHLIST_ENPS2 0x02
    #define SI1145_PARAM_CHLIST_ENPS3 0x04
    
    #define SI1145_PARAM_PSLED12SEL   0x02
    #define SI1145_PARAM_PSLED12SEL_PS2NONE 0x00
    #define SI1145_PARAM_PSLED12SEL_PS2LED1 0x10
    #define SI1145_PARAM_PSLED12SEL_PS2LED2 0x20
    #define SI1145_PARAM_PSLED12SEL_PS2LED3 0x40
    #define SI1145_PARAM_PSLED12SEL_PS1NONE 0x00
    #define SI1145_PARAM_PSLED12SEL_PS1LED1 0x01
    #define SI1145_PARAM_PSLED12SEL_PS1LED2 0x02
    #define SI1145_PARAM_PSLED12SEL_PS1LED3 0x04
    
    #define SI1145_PARAM_PSLED3SEL   0x03
    #define SI1145_PARAM_PSENCODE   0x05
    #define SI1145_PARAM_ALSENCODE  0x06
    
    #define SI1145_PARAM_PS1ADCMUX   0x07
    #define SI1145_PARAM_PS2ADCMUX   0x08
    #define SI1145_PARAM_PS3ADCMUX   0x09
    #define SI1145_PARAM_PSADCOUNTER   0x0A
    #define SI1145_PARAM_PSADCGAIN 0x0B
    #define SI1145_PARAM_PSADCMISC 0x0C
    #define SI1145_PARAM_PSADCMISC_RANGE 0x20
    #define SI1145_PARAM_PSADCMISC_PSMODE 0x04
    
    #define SI1145_PARAM_ALSIRADCMUX   0x0E
    #define SI1145_PARAM_AUXADCMUX   0x0F
    
    #define SI1145_PARAM_ALSVISADCOUNTER   0x10
    #define SI1145_PARAM_ALSVISADCGAIN 0x11
    #define SI1145_PARAM_ALSVISADCMISC 0x12
    #define SI1145_PARAM_ALSVISADCMISC_VISRANGE 0x20
    
    #define SI1145_PARAM_ALSIRADCOUNTER   0x1D
    #define SI1145_PARAM_ALSIRADCGAIN 0x1E
    #define SI1145_PARAM_ALSIRADCMISC 0x1F
    #define SI1145_PARAM_ALSIRADCMISC_RANGE 0x20
    
    #define SI1145_PARAM_ADCCOUNTER_511CLK 0x70
    
    #define SI1145_PARAM_ADCMUX_SMALLIR  0x00
    #define SI1145_PARAM_ADCMUX_LARGEIR  0x03
    
        /* REGISTERS */
    #define SI1145_REG_PARTID  0x00
    #define SI1145_REG_REVID  0x01
    #define SI1145_REG_SEQID  0x02
    
    #define SI1145_REG_INTCFG  0x03
    #define SI1145_REG_INTCFG_INTOE 0x01
    #define SI1145_REG_INTCFG_INTMODE 0x02
    
    #define SI1145_REG_IRQEN  0x04
    #define SI1145_REG_IRQEN_ALSEVERYSAMPLE 0x01
    #define SI1145_REG_IRQEN_PS1EVERYSAMPLE 0x04
    #define SI1145_REG_IRQEN_PS2EVERYSAMPLE 0x08
    #define SI1145_REG_IRQEN_PS3EVERYSAMPLE 0x10
    
    
    #define SI1145_REG_IRQMODE1 0x05
    #define SI1145_REG_IRQMODE2 0x06
    
    #define SI1145_REG_HWKEY  0x07
    #define SI1145_REG_MEASRATE0 0x08
    #define SI1145_REG_MEASRATE1  0x09
    #define SI1145_REG_PSRATE  0x0A
    #define SI1145_REG_PSLED21  0x0F
    #define SI1145_REG_PSLED3  0x10
    #define SI1145_REG_UCOEFF0  0x13
    #define SI1145_REG_UCOEFF1  0x14
    #define SI1145_REG_UCOEFF2  0x15
    #define SI1145_REG_UCOEFF3  0x16
    #define SI1145_REG_PARAMWR  0x17
    #define SI1145_REG_COMMAND  0x18
    #define SI1145_REG_RESPONSE  0x20
    #define SI1145_REG_IRQSTAT  0x21
    #define SI1145_REG_IRQSTAT_ALS  0x01
    
    #define SI1145_REG_ALSVISDATA0 0x22
    #define SI1145_REG_ALSVISDATA1 0x23
    #define SI1145_REG_ALSIRDATA0 0x24
    #define SI1145_REG_ALSIRDATA1 0x25
    #define SI1145_REG_PS1DATA0 0x26
    #define SI1145_REG_PS1DATA1 0x27
    #define SI1145_REG_PS2DATA0 0x28
    #define SI1145_REG_PS2DATA1 0x29
    #define SI1145_REG_PS3DATA0 0x2A
    #define SI1145_REG_PS3DATA1 0x2B
    #define SI1145_REG_UVINDEX0 0x2C
    #define SI1145_REG_UVINDEX1 0x2D
    #define SI1145_REG_PARAMRD 0x2E
    #define SI1145_REG_CHIPSTAT 0x30
    
    #define SI1145_ADDR 0x60

  uint8_t id = SI1145i2cread8(0x00); //Search for this on the 0x00 address, this is required as part of the startup
  Particle.publish(String::format("UV Sensor ID: %u - Should be equivelant of 0x45 ",id)); //reint returned sensor value, if returned  Typically 4 (in a uint8 container displayed as %u in printgf()) appears to be OK as a return
  id = 0x45; // Force this check to be true!  (see note below)
  
  if (id != 0x45) 
     {
           return false; // (the note) look for SI1145, if the value 0x45 is not found, exit the function - this check causes problems with multiple devices sharing I2C addresses in this design, even when connected properly!  Force this to pass if it gets stuck and you know the sensor is present
     }
     
  //Apply the reset sequence
  SI1145i2cwrite8(SI1145_REG_MEASRATE0, 0);
  SI1145i2cwrite8(SI1145_REG_MEASRATE1, 0);
  SI1145i2cwrite8(SI1145_REG_IRQEN, 0);
  SI1145i2cwrite8(SI1145_REG_IRQMODE1, 0);
  SI1145i2cwrite8(SI1145_REG_IRQMODE2, 0);
  SI1145i2cwrite8(SI1145_REG_INTCFG, 0);
  SI1145i2cwrite8(SI1145_REG_IRQSTAT, 0xFF);

  SI1145i2cwrite8(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  SI1145i2cwrite8(SI1145_REG_HWKEY, 0x17);
  delay(10);
  
    /***********************************/
  // enable UVindex measurement coefficients!
  SI1145i2cwrite8(SI1145_REG_UCOEFF0, 0x29);
  SI1145i2cwrite8(SI1145_REG_UCOEFF1, 0x89);
  SI1145i2cwrite8(SI1145_REG_UCOEFF2, 0x02);
  SI1145i2cwrite8(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  SI1145i2cwriteParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);
  
  // enable interrupt on every sample
  SI1145i2cwrite8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  SI1145i2cwrite8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);  

/****************************** Prox Sense 1 */

  // program LED current
  SI1145i2cwrite8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  SI1145i2cwriteParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  SI1145i2cwriteParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  SI1145i2cwriteParam(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  SI1145i2cwriteParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  SI1145i2cwriteParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  SI1145i2cwriteParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  SI1145i2cwriteParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  SI1145i2cwriteParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  SI1145i2cwriteParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);



  // fastest clocks, clock div 1
  SI1145i2cwriteParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  SI1145i2cwriteParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  SI1145i2cwriteParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


/************************/

  // measurement rate for auto
  SI1145i2cwrite8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
  
  // auto run
  SI1145i2cwrite8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
}
//****************************************************************************************

//use median values for this sensor to avoid value jumping for light sensor - median of three sucessive reads.
float getUV()
{

float a = uv.readUV();
 delay (3);
  float b = uv.readUV();
 delay (3);
  float c = uv.readUV();
 delay (3);
 
float middle;
float readval;

//calculate the median value for 3 reads, use standard algorithm for calculating median

  if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 readval = middle;
 
         //If the result is reasonable, add it to the running mean
          if(readval > 0 && readval < 100000) //this may be only 10 for the max value for this sensor, verify!
              {
                // Add the observation to the running sum, and increment the number of observations
                  lightUVTotal += readval;
                  lightUVReadingCount++;
                }
  return readval;
}


float getVis()
{

 float a = uv.readVisible();
 delay (3);
  float b = uv.readVisible();
 delay (3);
  float c = uv.readVisible();
 delay (3);
 
float middle;
float readval;

//calculate the median value for 3 reads

  if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 readval = middle;
         //If the result is reasonable, add it to the running mean
        if(readval > 0 && readval < 100000) 
            {
             // Add the observation to the running sum, and increment the number of observations
            lightVisTotal += readval;
            lightVisReadingCount++;
             }
   return readval;
}


float getIR()
{
 float a = uv.readIR();
 delay (3);
  float b = uv.readIR();
 delay (3);
  float c = uv.readIR();
 delay (3);
float middle;
float readval;

//calculate the median value for 3 reads, median function shown, use middle value

  if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 readval = middle;
 
         //If the result is reasonable, add it to the running mean
        if(readval > 0 && readval < 100000) 
         {
           // Add the observation to the running sum, and increment the number of observations
           lightIRTotal += readval;
           lightIRReadingCount++;
         }

    return readval;
}


float getUVReadings()
{
    //average readings from UV/ VIS/IR sensor, average and return
    //UV Index
   if (lightUVReadingCount == 0)
   {
       lightUVReadingCount=1; //avoid div by zero if no measurements recorded
   }
    float UVindex = lightUVTotal/float(lightUVReadingCount);
    lightUVTotal = 0.0;
    lightUVReadingCount = 0;
    
	return UVindex; //return UVindex (do the /100 for the returned val)
}


float getVisReadings()
{
    //average readings from UV/ VIS/IR sensor, average and return
   if (lightUVReadingCount == 0)
   {
       lightVisReadingCount = 1; //avoid div by zero if no measurements recorded
   }
      //Visible Light Level
    float vis = lightVisTotal/float(lightVisReadingCount);
    lightVisTotal = 0.0;
    lightVisReadingCount = 0;

	return vis;
	//return vis in ADC units;
}


float getIRReadings()
{
    //average readings from UV/ VIS/IR sensor, average and return
       if (lightUVReadingCount ==0)
   {
       lightVisReadingCount=1; //avoid div by zero if no measurements recorded
   }
    //IR Light Level
    float IR = lightIRTotal/float(lightIRReadingCount);
    lightIRTotal = 0.0;
    lightIRReadingCount = 0;

	return IR;
	//return IR in ADC units;
}


//*******************From SI1145 Library - I2C functions*****************************
void SI1145i2cwrite8(uint8_t reg, uint8_t val) 
{
  Wire.beginTransmission(SI1145_ADDR); // start transmission to SI1145 device, default address is 0x60
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}

//From SI1145 Library - I2C functions
uint8_t SI1145i2cread8(uint8_t reg) 
{

  Wire.beginTransmission(SI1145_ADDR); // start transmission to SI1145 device, default address is 0x60
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)SI1145_ADDR, (uint8_t)1);  //default address is 0x60
  return Wire.read();
}

//From SI1145 Library - I2C functions
uint8_t SI1145i2cwriteParam(uint8_t p, uint8_t v) 
{

  SI1145i2cwrite8(SI1145_REG_PARAMWR, v);
  SI1145i2cwrite8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return SI1145i2cread8(SI1145_REG_PARAMRD);
}


//functions to manage reconnection to Particle cloud and the Particle Photon's WiFi for semi-auto mode operation - toggle WiFi functions
void retryConnect()  //see here for details: https://community.particle.io/t/solved-make-photon-run-code-without-being-necessarily-connected-to-the-particle-cloud/25953/2
{
  if (!Particle.connected())   // if not connected to cloud
  {
    Serial.println("reconnect");
    stopTimer.start();         // set of the timout time
    WiFi.on();
    Particle.connect();        // start a reconnectino attempt
  }
  else                         // if already connected
  {
    Serial.println("connected");
    retryTimer.stop();         // no further attempts required
    retryRunning = false;
  }
}

void stopConnect()
{
    Serial.println("stopped");

    if (!Particle.connected()) // if after retryTime no connection
    WiFi.off();              // stop trying and swith off WiFi
    stopTimer.stop();
}


//*******Main Loop**********
void loop() 
{
    if (millis() > lastmillis)
        {
            lastmillis = millis(); //rollover checker, make sure this variable monotonically increases
        }
        
    // Capture any sensors that need to be polled (temp, humidity, pressure, wind vane), this is where the reads happen that will be averaged
    if(timeNextSensorReading <= millis()) 
    {
        if (GEIGER_READING==false) //only read new values when the geiger counter is off to avoid noise problems, in this case don't pull in likely errored values
        {
            delay (5);
            mux.setChannel(0);  //Switch to alt I2C Bus for reading the light sensor
            delay (10);
            getUV(); //Read UV Index value from SI1145 UV/VIS/IR Sensor
            delay (1);
            getIR(); //Read IR value from SI1145 UV/VIS/IR Sensor
            delay (1);
            getVis(); //Read visible light value from SI1145 UV/VIS/IR Sensor
            delay (5);
            mux.setChannel(1);  //Switch back to Main I2C Bus
            delay(100);
            captureTempHumidityPressure();
            delay (2);
            captureWindVane();
        // Schedule the next sensor reading
        }
        timeNextSensorReading = millis() + sensorCapturePeriod;
    }
    
        //crappy failsafe
	   //GEIGER_READING=false;
	   //digitalWrite(GeigerPowerPin, HIGH); //enable geiger counter, this happens when GEIGER_READING=true;
	if(timeNextGeigerReading <= millis() && (millis() > GeigerInitialDelay))  //Activate geiger counter after 30 seconds from startup, let first reading come in for other sensors so this can be cached
	{ // turn on geiger counter
	    GEIGER_READING=true;
		digitalWrite(GeigerPowerPin, LOW); //enable geiger counter, this happens when GEIGER_READING=true;
		if((timeNextGeigerReading+(60000*GeigerCounterRun) <= millis()) && ((lastgeigerreporttime + geigerreportdelay) < millis()) ) //pre-run length for geiger counter before reading
		{ // start taking data after a minute
			GEIGER_READING=true; //redundant
		    digitalWrite(GeigerPowerPin, LOW); //enable geiger counter, this happens when GEIGER_READING=true;  //redundant
			captureGeigerValues(); //read from the serial out on the geiger counter
			if(timeNextGeigerReading + (60000*GeigerCounterRun) + (geigerstart*60000) <= millis()) //run reporting period for the geiger counter
			{
				//close and prime for next run
				timeNextGeigerReading = (geigerdelay*60000) + millis(); //delay until restart
				digitalWrite(GeigerPowerPin, HIGH);  //disable geiger counter, end reading session
				GEIGER_READING=false;  //disable geiger counter, end reading session
				Particle.publish(String::format("%ld Function Read attempts to the Geiger Counter in last cycle", (geigerreportruncounter))); //notify of cycle to read GC
				Particle.publish(String::format("%ld Serial character read attempts for the Geiger Counter in last cycle", (geigerreportserialreadruncounter))); //notify of serial total trys to read GC
				geigerreportruncounter=0; //reset the counter for Geiger Counter read attempts
				geigerreportserialreadruncounter = 0 ;
				delay (1500); //wait a bit before polling other sensors to let all stabilize after reading session
			}
		}

	}		

   // Publish the data collected to Particle and MQTT
    if(timeNextPublish <= millis()) 
    {
       if (GEIGER_READING==false) //the geiger counter can cause noise issues, only read new values when the unit is off!  Don't reset past values, use the cached ones during reading of the geiger counter
        {
        // Get the data to be published, update the global cached values
            tempFrec = getAndResetTempF();
    		tempCrec = getAndResetTempC(); // *** need to make
    		//Serial.println("After Temp");
            humidityRHrec = getAndResetHumidityRH();
            //Serial.println("After Humidity");
            pressureKParec = getAndResetPressurePascals() / 1000.0;
            //Serial.println("After Press");
            rainInchesrec = getAndResetRainInches();
            //Serial.println("After Rain");
            float gustMPH;
            windMPHrec = getAndResetAnemometerMPH(&gustMPH);
            windDegreesrec = getAndResetWindVaneDegrees();
             //Serial.println("After Wind");
		    UVIndexrec = getUVReadings(); //return averaged values for UV (remember to div by 100 for index!)
		    visrec = getVisReadings(); //return averaged values for Vis
		    IRrec = getIRReadings(); //return averaged values for IR
		    
        }
        //at this point see if the particle connection is even OK, try to reconnect here also by toggling WiFi, the check is now in the main loop in semi-auto mode, this can be reenabled if required in use
        //if (!retryRunning && !Particle.connected())  // if we have not already scheduled a retry and are not connected to Particle Cloud, toggle WIFI and retry connection (alternative to forced restart)
           // { 
              //  stopTimer.start();         // set timeout for auto-retry by system
              //  retryRunning = true;
              //  retryTimer.start();        // schedula a retry to c
          //  }
        
        if (client.isConnected()) //check to see if MQTT connection is active, if so, push, if not, try to reestablish connection.
            {
    		publishToMQTT(tempFrec, tempCrec, humidityRHrec, pressureKParec, rainInchesrec, windMPHrec, windDegreesrec,UVIndexrec/100,visrec,IRrec);
    		client.loop(); //Check with loop active, refresh connection after post,keep MQTT connection alive
    		//Serial.println("Just Published to MQTT");  //Serial DEBUG message
            }
         else  //try a disconnect and reconnect operation if connection is not active
            {  

              client.disconnect();
              delay (2000); //wait 2 seconds for disconnect to happen connection before trying a new connection
              Particle.publish(String::format("Main Publish Fail: Now %f Min. Runtime", (float(millis())/60000.0))); //notify of connection failure to MQTT Broker (provided general connectivity)
              initializeCloudMQTT();  //Run connection function again
              delay (4000); //wait 2.0 seconds for connection
              int check_connection = 0;  //declare connection checker
              
              //Test if connection goes down for a connection restore
              int retrycounts = 0;  //reset connection retry count counter before loop is entered
              while (retrycounts < mqttconntectionretries && check_connection == 0) //try multiple timed retries to connect to MQTT broker after requesting a connection - break the loop when connection is seen or a timeout happens
              {
                if(client.isConnected() == 1) //check for MQTT connection, first try
                     {
                         check_connection = 1; //connection re-established, now break the loop
                     }
                     else
                        {
                            delay (1000); //add a minor delay between checks to allow connection to be established to test
                        }
                  
                retrycounts++; //loop timeout counter
            }
                 
            if (check_connection == 1) //if connection reestablished, try to publish again, assume to this point the c
                {
                    publishToMQTT(tempFrec, tempCrec, humidityRHrec, pressureKParec, rainInchesrec, windMPHrec, windDegreesrec,UVIndexrec/100,visrec,IRrec);
    	        	client.loop(); //Check with loop active, refresh connection after post,keep MQTT connection alive
    	        	Particle.publish(String::format("Client Publish Second Try now worked: Now %f Min. Runtime", (float(millis())/60000.0))); //notify of connection failure to MQTT Broker (provided general connectivity)
    	        	//Serial.println("Just Published to MQTT");  //Serial DEBUG message
    	        	conntectattempt=0; //connection reestablished, reset the counter
                }
               else
                {
                  conntectattempt++;
                }
             
              if (ConnectTrysBeforeReset < conntectattempt)  //resetsystem after specified failed sequential attempts to connect
                  {
                      //counts resets when system resets, they will accrew (conntectattempt will be set to zero, no need to reset in here)
                    Particle.publish(String::format("Client Connection Could not be established, Resetting Photon: %f Min. Runtime", (float(millis())/60000.0))); //notify of connection failure to MQTT Broker (provided general connectivity)
    		        client.disconnect(); //force a disconnect (should already be disconnected)
    		        delay(3000); //give time for any remaining signal to be sent out on another thread befpre restarting
                    System.reset(); //reset the Photon if there is no response, try again after it resets.  Eternally reset and retry connection
                  }
             }
         
        
            timeNextPublish = millis() + publishPeriod;  // Schedule the next publish event
    connectToCloud = true; //after each post attempt, reset toggle to test the connection to the Particle Photon Cloud, this slows down the number of attempts to check connection to Photon Cloud, it will do a quick check to see if still connected each time a post is tried.
    }
    
    if (millis() < lastmillis)
        {
          lastmillis  =  millis();  //millis() roll over protection
          timeNextPublish =  millis() + publishPeriod;  //millis() roll over protection
          lastgeigerreporttime = millis(); 
          lastcloudconnectretry  = millis();
          cloudlastcommtime = millis();
          timeNextGeigerReading = millis() + geigerdelta;
          cloudconnectionretryperiodcounter = 0; //reset counter if there is a rollover
        }

// The rain and wind speed sensors use interrupts, and so data is collected "in the background", these sensors ate taken care of.

//If the photon is in Semi-Auto mode, an explicit connection needs to be made to the particle cloud and checked to make sure it is live.  This is done here when allowed during each loop operation:
  if(connectToCloud && (millis()>lastcloudconnectretry) && Particle.connected() == false) //check and retry Particle Cloud connection if it is dropped, required in semi-auto mode, check first if the toggle is set to check before polling
    {
        Particle.connect();
        connectToCloud = false; //cancel request to try to connect until reset again during next publish, this slows down checking rate to limit time spent doing this
        lastcloudconnectretry = msRetryDelayreconnect+millis(); //millis overflow caught by other function, no protection on this one - slows down use of this function
        //keep retrying eternally, the failure for MQTT connection after an exteended period will trigger a reset if needed.
        cloudconnectionretryperiodcounter = (cloudconnectionretryperiodcounter+1); //index counter if connection is lost
    }
    else
    {
       if ((millis()>cloudconnectiontestperiod+cloudlastcommtime) && Particle.connected() == true)
       {
           cloudlastcommtime = millis(); 
           cloudconnectionretryperiodcounter = 0; //reset retry counter for cloud connection
       }
    }
    
  if (cloudconnectretryattemptsallowed<cloudconnectionretryperiodcounter) //This is a fail-safe that should only be hit if there is a chronic connection issue, this forces a reset of the board if there is no connection established - check to see if the number of timeouts is greater than the period, indicating that there is no consistant connection
    {
 	    client.disconnect(); //force a disconnect (should already be disconnected)
        delay(5000); //give time for any remaining signal to be sent out on another thread befpre restarting, there shouldn't be anything if no connection
        System.reset(); //reset the Photon if there is no response, try again after it resets.  Eternally reset and retry connection   
    }

}
