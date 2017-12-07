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
//
// Authors:
//	- Sid Kasat, CS Junior @ UC Irvine
//	- Mindy Saylors, EE Junior @ UC Irvine
//
//Project Managers: Dr. Michael Klopfer, Prof. GP Li.
//California Institute for Telecommunications and Information Technology (Calit2), 2017
//University of California, Irvine
//Extended components of project copyright Regents of the Univeristy of California and relesed into the public domain.
//===========================================================================
#include "application.h" //provides brownout protection capabilities - should be first library defined

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_SI1145.h>

// This #include statement was automatically added by the Particle IDE.
#include "SparkFun_Photon_Weather_Shield_Library/SparkFun_Photon_Weather_Shield_Library.h"

// Add math to get sine and cosine for wind vane
#include <math.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

//I2C Multiplexer Control Library
#include <TCA9548A-RK.h>


TCA9548A mux(Wire, 0); //Initiaalize the I2C Multiplexer with default settings

	
// Each time we loop through the main loop, we check to see if it's time to capture the I2C sensor readings
unsigned int sensorCapturePeriod = 100; //0.1 second
unsigned int timeNextSensorReading;


// Each time we loop through the main loop, we check to see if it's time to capture the geiger counter readings
unsigned int timeNextGeigerReading;
const unsigned int MINUTE = 60000; // 1 minute(s)  (a really longe delay is added here to improve battery life with the current solar charger)

// Each time we loop through the main loop, we check to see if it's time to publish the data we've collected
unsigned int publishPeriod = 10000; //10 second
unsigned int timeNextPublish; 

bool GEIGER_READING= false;

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Weather sensor;

Adafruit_SI1145 uv = Adafruit_SI1145(); //create object for UV, there is a test at this point but the I2C is not set yet


void setup() {
    Serial.begin(9600); //required for reporting over USB
    //Brownout protection for solar input
  delay(2000);
  printBrownOutResetLevel();
  setBrowoutResetLevel();
  printBrownOutResetLevel();
    initializeCloudMQTT();
	mux.begin();
	mux.setChannel(1);  //Go to Main I2C bus selection
        delay(50);
        initializeTempHumidityAndPressure();
        initializeRainGauge();
        initializeAnemometer();
        initializeWindVane();
    	initializeGeigerCounter();
    mux.setChannel(0); //switch to I2c Bus 0 before initializing the UV sensor
    	 delay (50);
    	 initializeUV(); //contains a replacement of uv.begin() without the check for 0x45 on device 0x00 which causes issues with the other sensors.
    mux.setChannel(1);  //Return to Main I2C bus selection
    // Schedule the next sensor reading and publish events
    timeNextSensorReading = millis() + sensorCapturePeriod;
    timeNextPublish = millis() + publishPeriod; 
	timeNextGeigerReading = millis() + sensorCapturePeriod; //start reading shortly after initlization
}

void printBrownOutResetLevel() {
  Serial.println("Reading BOR");

  uint8_t bor = FLASH_OB_GetBOR();

  switch(bor) {
    case OB_BOR_OFF:
      Serial.println("OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.10 V");
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

void setBrowoutResetLevel() {
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
 void callback(char* topic, byte* payload, unsigned int length); // dont need prototype currently ***
 void callback(char* topic, byte* payload, unsigned int length) {
     char p[length + 1];
     memcpy(p, payload, length);
     p[length] = NULL;
     if (!strcmp(p, "RED"))
         RGB.color(255, 0, 0);
     else if (!strcmp(p, "GREEN"))
         RGB.color(0, 255, 0);
     else if (!strcmp(p, "BLUE"))
         RGB.color(0, 0, 255);
     else
         RGB.color(255, 255, 255);
 }

 MQTT client("m12.cloudmqtt.com", 14668, callback);
 void initializeCloudMQTT() {
     client.connect("***SERVER******.cloudmqtt.com", "************USER*******", "******************PWD******");
    // publish/subscribe
     if (client.isConnected()) {
       client.publish("CONNECTION_STATUS","Connected.");
     }
 }

 void publishToMQTT(float tempF,float tempC,float humidityRH,float pressureKPa,float rainInches,float windMPH,float windDegrees, float UV, float vis, float IR) {
    // To write multiple fields, you set the various fields you want to send
 	char payload[255];
 	
 	//Derivative Calculations for partial pressure and dewpoint from final values to be reported
        float dewpointC = dewpoint(tempC,humidityRH);
        float dewpointF = (dewpointC * 1.8) + 32;
        
        float partialpressureH2O = vaporpressureH2O(dewpointC);
        //float partialpressureH2Osat = vaporpressureH2Osat(tempC);
        //float rh_calc= (partialpressureH2O/partialpressureH2Osat)*100;  //calculate RH from e and es

 	snprintf(payload, sizeof(payload), "%0.2f", tempF);
 	client.publish("Calit2_Weather_Station/Temperature_F", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", tempC);
 	client.publish("Calit2_Weather_Station/Temperature_C", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", humidityRH);
 	client.publish("Calit2_Weather_Station/Humidity_RH", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", pressureKPa);
 	client.publish("Calit2_Weather_Station/Pressure_ KPa", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", dewpointC);
 	client.publish("Calit2_Weather_Station/DewPoint_C", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", dewpointF);
 	client.publish("Calit2_Weather_Station/DewPoint_F", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", partialpressureH2O);
 	client.publish("Calit2_Weather_Station/H2OPartialPressure_ KPa", payload);

 	snprintf(payload, sizeof(payload), "%0.2f", windMPH);
 	client.publish("Calit2_Weather_Station/WindSpeed_MPH", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", windDegrees);
 	client.publish("Calit2_Weather_Station/WindDirection_DEG", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", rainInches);
 	client.publish("Calit2_Weather_Station/RainFall_INCHESperRepPeriod", payload);

	snprintf(payload, sizeof(payload), "%0.2f", UV);
 	client.publish("Calit2_Weather_Station/LightUV_INDEX", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", vis);
 	client.publish("Calit2_Weather_Station/LightVIS_ADC", payload);
 	
 	snprintf(payload, sizeof(payload), "%0.2f", IR);
 	client.publish("Calit2_Weather_Station/LightIR_ADC", payload);
 	
 	if (GEIGER_READING == false || millis()<=60000) //Solves blank issue for first minute
 	{
 	client.publish("Calit2_Weather_Station/GEIGER_OUTPUT", "*");
 	}
 	
 	snprintf(payload, sizeof(payload), "%0.2f", float(millis())/1000);
 	client.publish("Calit2_Weather_Station/RUNTIME_SEC", payload);
 }


//===========================================================
// Temp, Humidity and Pressure
//===========================================================
// The temperature, humidity, and pressure sensors are on board
// the weather station board, and use I2C to communicate.  The sensors are read
// frequently by the main loop, and the results are averaged over the publish cycle


void initializeTempHumidityAndPressure() {
    //Initialize the I2C sensors and ping them
    
    //Initialize Sensors on Weather Shield for Pressure and Temp/Humidity
    sensor.begin();
    
    ///*
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
        Particle.publish("HTU21D Temp/RH Sensor: ", String::format("%d",ID_1)); //Report value to Particle Console
    //*/
    
    //Set to Barometer Mode
    sensor.setModeBarometer();
    
    // Set Oversample rate
    sensor.setOversampleRate(7);

    //Necessary register calls to enble temp, baro and alt
    sensor.enableEventFlags(); 
}

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

void captureTempHumidityPressure() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations
  

  // Measure Temperature in F from the HTU21D or Si7021
  float tempF = sensor.getTempF();
  
  //If the result is reasonable, add it to the running mean
  if(tempF > -50 && tempF < 150)
  {
      // Add the observation to the running sum, and increment the number of observations
      tempFTotal += tempF;
      tempFReadingCount++;
  }
  
    // Measure Relative Humidity from the HTU21D or Si7021
 float humidityRH = sensor.getRH();
    
    float gain=1.4;    //HTU21D Sensor failure was experienced, this set of calibration factors seemed to make the data roughly "normal" - calibration done with hygrometer, this is atypical use!  No need for linear calibration if sensor is working properly
    float offset = 22;  //HTU21D Sensor failure was experienced, this set of calibration factors seemed to make the data roughly "normal" - calibration done with hygrometer, this is atypical use!  No need for linear calibration if sensor is working properly
    humidityRH = humidityRH*gain + offset;
  
  //If the result is reasonable, add it to the running mean
  if(humidityRH > 0 && humidityRH < 105)   //the lower bound should be 0, but something funny is going on....
  {
      // Add the observation to the running sum, and increment the number of observations
      humidityRHTotal += humidityRH;
      humidityRHReadingCount++;
  }
  
   // Measure Temperature in C from the HTU21D or Si7021
  float tempC = sensor.getTemp();
  
  //If the result is reasonable, add it to the running mean
  if(tempC > -65 && tempC < 65) 
  {
      // Add the observation to the running sum, and increment the number of observations
      tempCTotal += tempC;
      tempCReadingCount++;
  }

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
    if(tempFReadingCount == 0) {
        return 0;
    }
    float result = tempFTotal/float(tempFReadingCount);
    tempFTotal = 0.0;
    tempFReadingCount = 0;
    return result;
}

float getAndResetTempC()
{
    if(tempCReadingCount == 0) {
        return 0;
    }
    float result = tempCTotal/float(tempCReadingCount);
    tempCTotal = 0.0;
    tempCReadingCount = 0;
    return result;
}

float getAndResetHumidityRH()
{
    if(humidityRHReadingCount == 0) {
        return 0;
    }
    float result = humidityRHTotal/float(humidityRHReadingCount);
    humidityRHTotal = 0.0;
    humidityRHReadingCount = 0;
    return result;
}

float getAndResetPressurePascals()
{
    if(pressurePascalsReadingCount == 0) {
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

float vaporpressureH2O( float dpC)
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
int RainPin = D2;
volatile unsigned int rainEventCount;
unsigned int lastRainEvent;
float RainScaleInches = 0.011; // Each pulse is .011 inches of rain

void initializeRainGauge() {
  //pinMode(RainPin, INPUT_PULLUP);
  rainEventCount = 0;
  lastRainEvent = 0;
  //attachInterrupt(RainPin, handleRainEvent, FALLING);
  return;
  }
  
void handleRainEvent() {
    // Count rain gauge bucket tips as they occur
    // Activated by the magnet and reed switch in the rain gauge, attached to input D2
    unsigned int timeRainEvent = millis(); // grab current time
    // ignore switch-bounce glitches less than 10mS after initial edge
    if(timeRainEvent - lastRainEvent < 10) {
      return;
    }
    
    rainEventCount++; //Increase this minute's amount of rain
    lastRainEvent = timeRainEvent; // set up for next event
}

float getAndResetRainInches()
{
    float result = RainScaleInches * float(rainEventCount);
    rainEventCount = 0;
    return result;
}


//===========================================================================
// Wind Speed (Anemometer)
//===========================================================================

// The Anemometer generates a frequency relative to the windspeed.  1Hz: 1.492MPH, 2Hz: 2.984MPH, etc.
// We measure the average period (elaspsed time between pulses), and calculate the average windspeed since the last recording.

int AnemometerPin = D3;
float AnemometerScaleMPH = 1.492; // Windspeed if we got a pulse every second (i.e. 1Hz)
volatile unsigned int AnemoneterPeriodTotal = 0;
volatile unsigned int AnemoneterPeriodReadingCount = 0;
volatile unsigned int GustPeriod = UINT_MAX;
unsigned int lastAnemoneterEvent = 0;

void initializeAnemometer() {
  pinMode(AnemometerPin, INPUT_PULLUP);
  AnemoneterPeriodTotal = 0;
  AnemoneterPeriodReadingCount = 0;
  GustPeriod = UINT_MAX;  //  The shortest period (and therefore fastest gust) observed
  lastAnemoneterEvent = 0;
  attachInterrupt(AnemometerPin, handleAnemometerEvent, FALLING);
  return;
  }
  
void handleAnemometerEvent() {
    // Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
     unsigned int timeAnemometerEvent = millis(); // grab current time
     
    //If there's never been an event before (first time through), then just capture it
    if(lastAnemoneterEvent != 0) {
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
void initializeWindVane() {
    return;
}

// For the wind vane, we need to average the unit vector components (the sine and cosine of the angle)
int WindVanePin = A0;
float windVaneCosTotal = 0.0;
float windVaneSinTotal = 0.0;
unsigned int windVaneReadingCount = 0;

void captureWindVane() {
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
    if(windVaneReadingCount == 0) {
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
//	Be sure to use only if Serial1.available() is true
//
// Fix:
//	Need to have mode_speed to remain constant for reading to be sure averaging
//	cps, cpm, uSv/hr values make sense
//
//	Need to get captureGeigerValues() to keep taking in values until <I say it needs to>
//When no value is vailable, only * is returned

int GeigerPowerPin = D7; //this is the source for the enable pin (required as inverted) used to turn on the power regulator to supply the geiger counter with power

void initializeGeigerCounter(){
	Serial1.begin(9600);    	//Baud rate: 9600
	pinMode(GeigerPowerPin, OUTPUT);
	digitalWrite(GeigerPowerPin, HIGH); //turn off geiger counter read with HIGH (ivnverse enable on the controller)
	return;
}


// Values for get and reset function
float cpsval=0;			
float cpmval=0;
float usvHRval =0; 
String mode_speedval="";

void captureGeigerValues(){ //Captures values (uses a while loop which may disrupt timing)
	bool dataIsNotCollected = true;		// to collect data for one line of data

	do{
		int arrlen=50;
		char test[arrlen];
		test[0]='*';
		for (int i=1;i<arrlen;i++){
		    test[i]='\0';
		}

		int i=0;
	    while (i<arrlen and Serial1.available()){
	        char c= Serial1.read();
		    test[i]=c;
	        i++;
	        
		}
		
		if (test[0]=='C' or test[0]=='*'){
		client.publish("Calit2_Weather_Station/GEIGER_OUTPUT",test);
		char payload[255];
		snprintf(payload, sizeof(payload), "%0.2f", float(millis())/1000);
 	    client.publish("Calit2_Weather_Station/RUNTIME_SEC", payload);
		}
		
		dataIsNotCollected = false;
	}while (dataIsNotCollected);
}


bool initializeUV()
{
   //Equivelant of uv.begin() in the Adafruit SI1145 library, but without the restart of wire.begin() and the initial test read check for 0x45 from 0x00
   
//I2C SI1145 UV Sensor Parameters - from library
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
  Particle.publish(String::format("UV Sensor ID: %d - Should be equivelant of 0x45 ",id));
  id = 0x45; // Force this check to be true!  (see note below)
  
  if (id != 0x45) 
     {
           return false; // (the note) look for SI1145, if the value 0x45 is not found, exit the function - this causes problems with multiple devices!  Force this to pass if it gets stuck and you know the sensor is present
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

//Values for Light/UV/IR Sensor
float lightVisTotal = 0.0;
unsigned int lightVisReadingCount = 0;

float lightUVTotal = 0.0;
unsigned int lightUVReadingCount = 0;

float lightIRTotal = 0.0;
unsigned int lightIRReadingCount = 0;

float getUV()
{
    mux.setChannel(0); //switch  to 0 before initializing the UV sensor
        delay (50);
        float readval = uv.readUV();
         //If the result is reasonable, add it to the running mean
          if(readval > 0 && readval < 10) 
              {
                // Add the observation to the running sum, and increment the number of observations
                  lightUVTotal += readval;
                  lightUVReadingCount++;
                }
  mux.setChannel(1); //switch  to 1 before leaving function
  delay (50);
  return readval;
}

float getVis()
{
    mux.setChannel(0); //switch  to 0 before initializing the UV sensor
        delay (50);
        float readval = uv.readVisible();
         //If the result is reasonable, add it to the running mean
        if(readval > 0 && readval < 1000) 
            {
             // Add the observation to the running sum, and increment the number of observations
            lightVisTotal += readval;
            lightVisReadingCount++;
             }
   mux.setChannel(1); //switch  to 1 before leaving function
    delay (50);
   return readval;
}

float getIR()
{
    mux.setChannel(0); //switch  to 0 before initializing the UV sensor
        delay (50);
        float readval = uv.readIR();
         //If the result is reasonable, add it to the running mean
        if(readval > 0 && readval < 1000) 
         {
           // Add the observation to the running sum, and increment the number of observations
           lightIRTotal += readval;
           lightIRReadingCount++;
         }
    mux.setChannel(1); //switch  to 1 before leaving function
     delay (50);
    return readval;
}


float getUVReadings()
{
    //average readings from UV/ VIS/IR sensor, average and return
    //UV Index
   if (lightUVReadingCount ==0)
   {
       lightUVReadingCount=1; //avoid div by zero if no measurements recorded
   }
    float UVindex = lightUVTotal/float(lightUVReadingCount);
    lightUVTotal = 0.0;
    lightUVReadingCount = 0;
    
	return UVindex/100; //return UVindex/100
}

float getVisReadings()
{
    //average readings from UV/ VIS/IR sensor, average and return
   if (lightUVReadingCount ==0)
   {
       lightVisReadingCount=1; //avoid div by zero if no measurements recorded
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

//From SI1145 Library - I2C functions
void SI1145i2cwrite8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(SI1145_ADDR); // start transmission to SI1145 device, default address is 0x60
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}

//From SI1145 Library - I2C functions
uint8_t SI1145i2cread8(uint8_t reg) {

  Wire.beginTransmission(SI1145_ADDR); // start transmission to SI1145 device, default address is 0x60
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)SI1145_ADDR, (uint8_t)1);  //default address is 0x60
  return Wire.read();
}

//From SI1145 Library - I2C functions
uint8_t SI1145i2cwriteParam(uint8_t p, uint8_t v) {

  SI1145i2cwrite8(SI1145_REG_PARAMWR, v);
  SI1145i2cwrite8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return SI1145i2cread8(SI1145_REG_PARAMRD);
}

//Main Loop
void loop() {

    // Capture any sensors that need to be polled (temp, humidity, pressure, wind vane)
    // The rain and wind speed sensors use interrupts, and so data is collected "in the background"
    
    //if(timeNextSensorReading <= millis()) {
        mux.setChannel(0);  //Switch to Main I2C Bus
        delay (20);
            getIR(); //Read IR value from SI1145 UV/VIS/IR Sensor
            getVis(); //Read visible light value from SI1145 UV/VIS/IR Sensor
            getUV(); //Read UV Index value from SI1145 UV/VIS/IR Sensor
        mux.setChannel(1);  //Switch to Main I2C Bus
        delay(20);
            captureTempHumidityPressure();
            captureWindVane();
        // Schedule the next sensor reading
        timeNextSensorReading = millis() + sensorCapturePeriod;
    //}

	if(timeNextGeigerReading <= millis())
	{ // turn on geiger counter
		digitalWrite(GeigerPowerPin, LOW); //enable geige counter
		GEIGER_READING=true;
		if(timeNextGeigerReading+MINUTE <= millis()){ // start taking data after a minute
			captureGeigerValues();
			GEIGER_READING=true;
			if(timeNextGeigerReading + 2*MINUTE <= millis()){
				timeNextGeigerReading = 10*MINUTE+millis();
				digitalWrite(GeigerPowerPin, HIGH);  //disable geige counter, end reading session
				GEIGER_READING=false;  //disable geige counter, end reading session
			}
		}	
	}		

	
    // Publish the data collected to Particle and to ThingSpeak
    if(timeNextPublish <= millis()) 
    {
        // Get the data to be published
            float tempF = getAndResetTempF();
    		float tempC = getAndResetTempC(); // *** need to make
    		//Serial.println("After Temp");
            float humidityRH = getAndResetHumidityRH();
            //Serial.println("After Humidity");
            float pressureKPa = getAndResetPressurePascals() / 1000.0;
            //Serial.println("After Press");
            float rainInches = getAndResetRainInches();
            //Serial.println("After Rain");
            float gustMPH;
            float windMPH = getAndResetAnemometerMPH(&gustMPH);
            float windDegrees = getAndResetWindVaneDegrees();
        //Serial.println("After Wind");
		    float UVIndex = getUVReadings(); //return averaged values for UV
		    float vis = getVisReadings(); //return averaged values for Vis
		    float IR= getIRReadings(); //return averaged values for IR
		    

		publishToMQTT(tempF, tempC, humidityRH, pressureKPa, rainInches, windMPH, windDegrees,UVIndex,vis,IR);
		//Serial.println("After Publish");
        // Schedule the next publish event
        timeNextPublish = millis() + publishPeriod;
    }
    
    delay(10);
}
