//===========================================================================
// Description:
//	Photon weather station includes temperature, humidity, pressure, rain in inches,
// wind speed, direction of wind, and ionizing radiation data from UV and geiger counter sensors.
// The Geiger Counter that was interfaced via UART is by Mighty Ohm:  http://mightyohm.com/blog/products/geiger-counter/
// Based on the example code from Haodong Liang who used ThingSpeak, but modified for it also to be 
// used for CloudMQTT. Link to his project: https://www.hackster.io/hliang/thingspeak-weather-station-data-analysis-2877b0
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


// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_SI1145.h>

// This #include statement was automatically added by the Particle IDE.
#include "SparkFun_Photon_Weather_Shield_Library/SparkFun_Photon_Weather_Shield_Library.h"

// Add math to get sine and cosine for wind vane
#include <math.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>
#define TCAADDR 0x70

// Each time we loop through the main loop, we check to see if it's time to capture the I2C sensor readings
unsigned int sensorCapturePeriod = 100; //0.1 second
unsigned int timeNextSensorReading;

Adafruit_SI1145 uv = Adafruit_SI1145();

// Each time we loop through the main loop, we check to see if it's time to capture the geiger counter readings
unsigned int timeNextGeigerReading;
const unsigned int MINUTE = 60000; // 1 minute

// Each time we loop through the main loop, we check to see if it's time to publish the data we've collected
unsigned int publishPeriod = 10000; //1 second
unsigned int timeNextPublish; 

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    initializeCloudMQTT();
    initializeTempHumidityAndPressure();
    initializeRainGauge();
    initializeAnemometer();
    initializeWindVane();
    initializeGeigerCounter();
    tcaselect(4);
	initializeUV();
    Serial.begin(9600);
    // Schedule the next sensor reading and publish events
    timeNextSensorReading = millis() + sensorCapturePeriod;
    timeNextPublish = millis() + publishPeriod; 
    timeNextGeigerReading = millis() + sensorCapturePeriod; //start reading shortly after initlization
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

 MQTT client("CLIENT", PORT, callback);
 void initializeCloudMQTT() {
     client.connect("CLIENT", "USERNAME", "PASSWORD");
    // publish/subscribe
     if (client.isConnected()) {
       client.publish("CONNECTION_STATUS","Connected.");
     }
 }

 void publishToMQTT(float tempF,float tempC,float humidityRH,float pressureKPa,float rainInches,float windMPH,float windDegrees, float UV) {
    // To write multiple fields, you set the various fields you want to send
 	char payload[255];

 	snprintf(payload, sizeof(payload), "%0.1f", tempF);
 	client.publish("TemperatureF", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", tempC);
 	client.publish("TemperatureC", payload);
	
 	snprintf(payload, sizeof(payload), "%0.1f", humidityRH);
 	client.publish("Humidity", payload);
	
 	snprintf(payload, sizeof(payload), "%0.1f", pressureKPa);
 	client.publish("Pressure_ KPa", payload);
	
 	snprintf(payload, sizeof(payload), "%0.1f", windMPH);
 	client.publish("Wind_Speed_MPH", payload);
	
 	snprintf(payload, sizeof(payload), "%0.0f", windDegrees);
 	client.publish("Wind_Degrees", payload);

	snprintf(payload, sizeof(payload), "%0.2f", UV);
 	client.publish("UV_Index", payload);
 }

 void publishToMQTT(float cpsPub, float cpmPub, float uSv_hrPub) { 
    // To write multiple fields, you set the various fields you want to send
 	char payload[255];

 	snprintf(payload, sizeof(payload), "%0.0f", cpsPub);
 	client.publish("Geiger_counter_counts_per_second", payload);
	
 	snprintf(payload, sizeof(payload), "%0.0f", cpmPub);
 	client.publish("Geiger_counter_counts_per_minute", payload);
	
 	snprintf(payload, sizeof(payload), "%0.2f", uSv_hrPub);
 	client.publish("MicroSieverts_per_hour_uSv/hr", payload);
	
 }

  
  
//===========================================================
// Temp, Humidity and Pressure
//===========================================================
// The temperature, humidity, and pressure sensors are on board
// the weather station board, and use I2C to communicate.  The sensors are read
// frequently by the main loop, and the results are averaged over the publish cycle

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Weather sensor;

void initializeTempHumidityAndPressure() {
    //Initialize the I2C sensors and ping them
    sensor.begin();
    //Set to Barometer Mode
    sensor.setModeBarometer();
    // Set Oversample rate
    sensor.setOversampleRate(7);
    //Necessary register calls to enble temp, baro and alt
    sensor.enableEventFlags(); 
    
    return;
}

float humidityRHTotal = 0.0;
unsigned int humidityRHReadingCount = 0;
float tempFTotal = 0.0;
unsigned int tempFReadingCount = 0;
float tempCTotal = 0.0;
unsigned int tempCReadingCount = 0;
float pressurePascalsTotal = 0.0;
unsigned int pressurePascalsReadingCount = 0;

void captureTempHumidityPressure() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations
  
  // Measure Relative Humidity from the HTU21D or Si7021
  float humidityRH = sensor.getRH();
  
  //If the result is reasonable, add it to the running mean
  if(humidityRH > 0 && humidityRH < 105) 
  {
      // Add the observation to the running sum, and increment the number of observations
      humidityRHTotal += humidityRH;
      humidityRHReadingCount++;
  }

  // Measure Temperature from the HTU21D or Si7021
  float tempF = sensor.getTempF();
  
  //If the result is reasonable, add it to the running mean
  if(tempF > -50 && tempF < 150)
  {
      // Add the observation to the running sum, and increment the number of observations
      tempFTotal += tempF;
      tempFReadingCount++;
  }
  
  float tempC = sensor.getTemp();
  
  //If the result is reasonable, add it to the running mean
  if(tempC > -45 && tempC < 65)
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

int GeigerPowerPin = D7;

void initializeGeigerCounter(){
	Serial1.begin(9600);    	//Baud rate: 9600
	pinMode(GeigerPowerPin, OUTPUT);
	digitalWrite(GeigerPowerPin, HIGH);
	return;
}


// Values for get and reset function
float cpsval=0;			
float cpmval=0;
float usvHRval =0; 
String mode_speedval="";

void captureGeigerValues(){ //Captures values (uses a while loop which may disrupt timing)

	int countcomma = 0; 	// for Geiger Counter Data Logging 
	bool secondNewLine = false; 		// to gather data on the second newline
	bool dataIsNotCollected = true;		// to collect data for one line of data

	// Geiger Counter Data variables
	char cps[5]="";			
	char cpm[5]="";
	char usvHR[10]="";
	char mode_speed[10]="";  	

	do{
		char c= (uint8_t)Serial1.read(); //Read in the data (one char each loop)
		if(c=='\n' && secondNewLine == false){
			secondNewLine = true;
		}else{
			if(c==','){
				countcomma++; // increment the comma count for each comma
			} else if(c=='\n'){ // if the entire line of data has been recived then type cast
				cpsval = atof(cps); //Type cast to float
				cpmval = atof(cpm); //Type cast to float
				usvHRval = atof(usvHR); //Type cast to float
				mode_speedval = mode_speed;
				dataIsNotCollected = false;
			} else{
				switch(countcomma){
					case 1: //If one comma (CPS) and so forth for the rest of the cases
						//snprintf() will reset the char array to the third parameter
						snprintf(cps, sizeof(cps), "%s%c", cps, c);
						break;
					case 3: 
						snprintf(cpm, sizeof(cpm), "%s%c", cpm, c);
						break;
					case 5: 
						snprintf(usvHR, sizeof(usvHR), "%s%c", usvHR, c);
						break;
					case 6:
						snprintf(mode_speed, sizeof(mode_speed), "%s%c", mode_speed, c);
						break;
					default: 
						break;
				} 
			}
		}
	}while (dataIsNotCollected);


}
float getAndResetCPS()
{
	if(mode_speedval == " INST") { // Recorded cps value is cps/60  when in INST mode (from geiger manual)
		cpsval*=60;
	}
	float result = cpsval;
	cpsval = 0;
	mode_speedval = "";
    return result;
}

float getAndResetCPM()
{
    float result = cpmval;
	cpmval = 0;
    return result;
}

float getAndResetuSv_hr()
{
    float result = usvHRval;
	usvHRval = 0;
    return result;
}

void initializeUV()
{
	Serial.begin(9600);  
	if (! uv.begin()) {
	Serial.println("Didn't find Si1145");
	}
}

float getUVReadings()
{
	float UVindex = uv.readUV();
	// the index is multiplied by 100 so to get the
	// integer index, divide by 100!
	return UVindex/100.0;  
}

void loop() {

    // Capture any sensors that need to be polled (temp, humidity, pressure, wind vane)
    // The rain and wind speed sensors use interrupts, and so data is collected "in the background"
    
    //if(timeNextSensorReading <= millis()) {
        captureTempHumidityPressure();
        captureWindVane();
        // Schedule the next sensor reading
        timeNextSensorReading = millis() + sensorCapturePeriod;
    //}
	
	if(timeNextGeigerReading <= millis()){ // turn on geiger counter
		digitalWrite(GeigerPowerPin, LOW);
		if(timeNextGeigerReading+MINUTE <= millis()){ // start taking data after a minute
			captureGeigerValues();
			float cpsPub = getAndResetCPS();
			float cpmPub = getAndResetCPM();
			float uSv_hrPub = getAndResetuSv_hr();
			//char  modeSpeedPub = getAndResetModeSpeed();
			Serial.println(cpsPub);
			Serial.println(cpmPub);
			Serial.println(uSv_hrPub);
			
			publishToMQTT(cpsPub, cpmPub, uSv_hrPub);
			if(timeNextGeigerReading + 2*MINUTE <= millis()){
				timeNextGeigerReading = 8*MINUTE+millis();
				digitalWrite(GeigerPowerPin, HIGH);
			}
		}	
	}		

	
    // Publish the data collected to Particle and to ThingSpeak
    if(timeNextPublish <= millis()) {
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
		tcaselect(4);
		    float UVIndex = getUVReadings();

		publishToMQTT(tempF, tempC, humidityRH, pressureKPa, rainInches, windMPH, windDegrees,UVIndex);
		//Serial.println("After Publish");
        // Schedule the next publish event
        timeNextPublish = millis() + publishPeriod;
    }
    
    delay(10);
}
