/*
 * Project AquaMaster - FSM Approach
 * Description: Watering program for the back deck
 * Author: Chip McClelland
 * Date: 5/8/2018

 Wiring for Chirp (Board/Assign/Cable) - Red/Vcc/Orange, Black/GND/Green, Blue/SCL/Green&White, Yellow/SDA/Orange&White

 To see Finite State Machine Diagram, past this uml code below in http://www.plantuml.com/plantuml/uml
 @startuml
 skinparam backgroundColor LightYellow
 skinparam state {
   BackgroundColor LightBlue
   BorderColor Gray
   FontName Impact
 }
 [*] --> Initializing
 Initializing: 10 seconds
 Initializing -Down-> Idle: Success
 Initializing -Down-> Error: Failure
 Idle --> Sensing: New Hour
 Sensing --> AwaitingForecast: Valid Data
 Sensing -Left-> Error: Bad Data
 AwaitingForecast -Left-> Watering: Done Waiting
 Watering -Up-> Reporting: Watering Complete
 Reporting -Up-> AwaitingReceipt: Report Sent
 AwaitingReceipt -Right-> Idle: Reprting Complete
 AwaitingReceipt -Right-> Error: Bad Report
 @enduml

*/

//STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));      // continually switches at high speed between antennas
STARTUP(WiFi.selectAntenna(ANT_INTERNAL));      // continually switches at high speed between antennas
STARTUP(System.enableFeature(FEATURE_RESET_INFO));  // Track why we reset
SYSTEM_THREAD(ENABLED);

// Software Release lets me know what version the Particle is running
#define SOFTWARERELEASENUMBER "0.55"

// Included Libraries
#include <I2CSoilMoistureSensor.h>          // Apollon77's Chirp Library: https://github.com/Apollon77/I2CSoilMoistureSensor

// Function Prototypes
I2CSoilMoistureSensor sensor;               // For the Chirp sensor

// Constants for Pins
const int solenoidPin = D2;                 // Pin that controls the MOSFET that turn on the water
const int solenoidPin2 = D7;                     // Used for debugging, can see when water is ON
const int donePin = D6;                     // Pin the Electron uses to "pet" the watchdog
const int wakeUpPin = A7;                   // This is the Particle Electron WKP pin
const int flowPin = A2;                     // Where the flow meter pulse comes in

// Timing Variables
unsigned long publishTimeStamp = 0;         // Keep track of when we publish a webhook
unsigned long resetWaitTimeStamp = 0;       // Starts the reset wait clock
unsigned long webhookWaitTime = 45000;      // How long will we let a webhook go before we give up
unsigned long resetWaitTime = 30000;        // Will wait this lonk before resetting.
unsigned long oneMinuteMillis = 60000;      // For Testing the system and smaller adjustments
unsigned long publishFrequency = 1000;          // How often can we publish to Particle
unsigned long lastPublish;

// EEPROM Memory Map
int controlRegisterAddr = 0;                // First Byte - Control Register
                                            // Control Register (7-2 Open, 1 - Enabled, 0 - Verbose Mode)
int lastWateredPeriodAddr = 1;              // Sesond Byte - Last Watered Period (hour)
int lastWateredDayAddr = 2;                 // Third Byte - Last Watered Day
int resetCountAddr = 3;                     // Fourth Byte - Reset count
int timeZoneAddr = 4;                       // Fifth Byte (signed) - Time zone

// Control Variables
bool waiting = false;
byte controlRegister;                               // Stores the control register values
bool verboseMode;
byte resetCount;
byte currentPeriod = 0;                      // Change length of period for testing 2 places in main loop
byte lastWateredPeriod = 0;                  // So we only wanter once an hour
byte lastWateredDay = 0;                     // Need to reset the last watered period each day
byte currentDay = 0;                         // Updated so we can tell which day we last watered


// Watering Variables
int shortWaterMinutes = 1;                  // Short watering cycle
int longWaterMinutes = 5;                   // Long watering cycle - must be shorter than watchdog interval!
int wateringMinutes = 0;                    // How long will we water based on time or Moisture
int startWaterHour = 5;                     // When can we start watering
int stopWaterHour = 12;                     // When do we stop for the day
bool watering = false;                      // Status - watering?
bool waterEnabled = 1;                       // Allows you to disable watering from the app or Ubidots
float expectedRainfallToday = 0;            // From Weather Underground Simple Forecast qpf_allday
int forecastDay = 0;                        // So we can know when we get a response from Weather Underground
unsigned long wateringStarted = 0;

// Measurement Variables
char Signal[17];                            // Used to communicate Wireless RSSI and Description
char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};
char* capDescription[6] = { "Very Dry", "Dry", "Normal", "Wet", "Very Wet", "Waterlogged"};
char Rainfall[5];                           // Report Rainfall preduction
int capValue = 0;                           // This is where we store the soil moisture sensor raw data
int soilTemp = 0;                           // Soil Temp is measured 3" deep
char Moisture[15];                          // Combines description and capValue

// Control Variables
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
bool doneEnabled = true;           // This enables petting the watchdog
char wateringContext[25];                   // Why did we water or not sent to Ubidots for context
float rainThreshold = 0.45;                  // Expected rainfall in inches which would cause us not to water

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SENSING_STATE, SCHEDULING_STATE, FORECASTING_STATE, WATERING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
State state = INITIALIZATION_STATE;


void setup() {
  pinMode(donePin,OUTPUT);                  // Allows us to pet the watchdog
  digitalWrite(donePin, HIGH);              // Pet now while we are getting set up
  digitalWrite(donePin, LOW);
  pinMode(solenoidPin,OUTPUT);              // Pin to turn on the water
  pinMode(solenoidPin2,OUTPUT);              // Pin to turn on the water
  digitalWrite(solenoidPin, LOW);           // Make sure it is off
  digitalWrite(solenoidPin2, LOW);           // Make sure it is off
  pinMode(wakeUpPin,INPUT_PULLDOWN);        // The signal from the watchdog is active HIGH
  attachInterrupt(wakeUpPin, watchdogISR, RISING);   // The watchdog timer will signal us and we have to respond


  Particle.variable("WiFiStrength", Signal);      // These variables are used to monitor the device will reduce them over time
  Particle.variable("Moisture", Moisture);
  Particle.variable("Enabled", waterEnabled);
  Particle.variable("Release",releaseNumber);
  Particle.variable("LastWater",lastWateredPeriod);
  Particle.variable("RainFcst", Rainfall);
  Particle.function("start-stop", startStop);       // Here are the functions for easy control
  Particle.function("Enabled", wateringEnabled);    // I can disable watering simply here
  Particle.function("Measure", takeMeasurements);   // If we want to see Temp / Moisture values updated
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-Verbose",setVerboseMode);


  Wire.begin();
  sensor.begin(true);                       // reset the Chirp sensor

  char responseTopic[125];
  String deviceID = System.deviceID();
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, AquaMasterHandler, MY_DEVICES);       // Subscribe to the integration response event
  Particle.subscribe("hook-response/weatherU_hook", weatherHandler, MY_DEVICES);       // Subscribe to weather response

  int8_t tempTimeZoneOffset = EEPROM.read(timeZoneAddr);                  // Load Time zone data from EEPROM
  if (tempTimeZoneOffset > -12 && tempTimeZoneOffset < 12) Time.zone((float)tempTimeZoneOffset);
  else Time.zone(-4);                             // Default if no valid number stored - Raleigh DST (watering is for the summer)

  resetCount = EEPROM.read(resetCountAddr);       // Retrive system recount data from EEPROM
  if (System.resetReason() == RESET_REASON_PIN_RESET)  // Check to see if we are starting from a pin reset
  {
    resetCount++;
    EEPROM.write(resetCountAddr,resetCount);    // If so, store incremented number - watchdog must have done This
  }

  controlRegister = EEPROM.read(controlRegisterAddr);
  verboseMode = controlRegister  & 0b00000001;
  waterEnabled = controlRegister & 0b00000010;
  lastWateredPeriod = EEPROM.read(lastWateredPeriodAddr);    // Load the last watered period from EEPROM
  lastWateredDay = EEPROM.read(lastWateredDayAddr);          // Load the last watered day from EEPROM

  if (sensor.getAddress() == 32) state = SENSING_STATE;    // Finished Initialization - time to enter main loop and wait for the top of the hour
  //else state = ERROR_STATE;
  else state = SENSING_STATE;
  if (verboseMode) {
    waitUntil(meterParticlePublish);
    Particle.publish("State","Idle");
    lastPublish = millis();
  }
}


void loop() {
  switch(state) {
    case IDLE_STATE:
      if (Time.day() != currentDay)
      {
        resetCount = 0;
        EEPROM.write(resetCountAddr,resetCount);    // If so, store incremented number - watchdog must have done This
      }
      if (Time.hour() != currentPeriod)                       // Spring into action each hour on the hour
      {
        currentPeriod = Time.hour();                          // Set the new current period
        currentDay = Time.day();                              // Sets the current Day
        // This next line protects against a reboot causing rewatering in same period
        if (currentPeriod != lastWateredPeriod || currentDay != lastWateredDay) {
          state = SENSING_STATE;
          if (verboseMode) {
            waitUntil(meterParticlePublish);
            Particle.publish("State","Sensing");
            lastPublish = millis();
          }
        }
      }
      break;

    case SENSING_STATE:
      getWiFiStrength();                                    // Get the WiFi Signal strength
      soilTemp = int(sensor.getTemperature()/(float)10);    // Get the Soil temperature
      while(sensor.isBusy());             // Wait to make sure sensor is ready.
      if (!getMoisture())                 // Test soil Moisture - if valid then proceed
      {
        state = ERROR_STATE;
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Error - getMoisture Failed");
          lastPublish = millis();
        }
        break;
      }
      else if((strncmp(Moisture,"Wet",3) == 0) || (strncmp(Moisture,"Very Wet",8) == 0) || (strncmp(Moisture,"Waterlogged",11) == 0))
      {
        strcpy(wateringContext,"Not Needed");
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Reporting - Too Wet");
          lastPublish = millis();
        }
        state = REPORTING_STATE;
      }
      else
      {
        Particle.publish("weatherU_hook");                    // Get the weather forcast
        publishTimeStamp = millis();                          // So we can know how long to wait
        forecastDay = expectedRainfallToday = 0;              // So we know when we get an updated forecast
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Scheduling");
          lastPublish = millis();
        }
        state = SCHEDULING_STATE;
      }
      break;

    case SCHEDULING_STATE:
      waitUntil(meterParticlePublish);
      state = FORECASTING_STATE;
      if (currentPeriod < startWaterHour || currentPeriod > stopWaterHour)
      {
        strcpy(wateringContext,"Not Time");
        if(verboseMode) Particle.publish("State","Reporting - Not Time");
        wateringMinutes = 0;
        state = REPORTING_STATE;
      }
      else if (!waterEnabled)
      {
        strcpy(wateringContext,"Not Enabled");
        if(verboseMode) Particle.publish("State","Reporting - Not Enabled");
        wateringMinutes = 0;
        state = REPORTING_STATE;
      }
      else if (currentPeriod == startWaterHour) wateringMinutes = longWaterMinutes;  // So, the first watering is long
      else wateringMinutes = shortWaterMinutes;                                 // Subsequent are short - fine tuning
      if(verboseMode && state == FORECASTING_STATE) Particle.publish("State","Forecasting");
      lastPublish = millis();
      break;


    case FORECASTING_STATE:
      if ((millis() >= (publishTimeStamp + webhookWaitTime)) || forecastDay)
      {
        state = WATERING_STATE;

        if (expectedRainfallToday > rainThreshold)
        {
          strcpy(wateringContext,"Heavy Rain Expected");
          if (verboseMode) {
            waitUntil(meterParticlePublish);
            Particle.publish("State","Reporting - Rain Forecast");
            lastPublish = millis();
          }
          state = REPORTING_STATE;
        }
      }
      if (verboseMode && state == WATERING_STATE) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Watering");
        lastPublish = millis();
      }
      break;

    case WATERING_STATE:
      if (!watering)
      {
        // If you get to here - Watering is enabled, it is time, no heavy rain expected and the soil is dry - so let's water
        strcpy(wateringContext,"Watering");
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Started Watering");
          lastPublish = millis();
        }
        digitalWrite(donePin, HIGH);                            // We will pet the dog now so we have the full interval to water
        digitalWrite(donePin, LOW);                             // We set the delay resistor to 50k or 7 mins so that is the longest watering duration
        doneEnabled = false;                                    // Will suspend watchdog petting until water is turned off
        digitalWrite(solenoidPin, HIGH);                        // Turn on the water
        digitalWrite(solenoidPin2,HIGH);
        watering = true;
        wateringStarted = millis();
      }
      if (watering && millis() >= (wateringStarted + wateringMinutes * oneMinuteMillis))
      {
        digitalWrite(solenoidPin, LOW);
        digitalWrite(solenoidPin2,LOW);
        watering = false;
        doneEnabled = true;                                     // Successful response - can pet the dog again
        digitalWrite(donePin, HIGH);                            // If an interrupt came in while petting disabled, we missed it so...
        digitalWrite(donePin, LOW);                             // will pet the fdog just to be safe
        lastWateredDay = currentDay;
        lastWateredPeriod = currentPeriod;
        EEPROM.write(lastWateredPeriodAddr,currentPeriod);        // Sets the last watered period to the current one
        EEPROM.write(lastWateredDayAddr,currentDay);              // Stored in EEPROM since this issue only comes in case of a reset
        state = REPORTING_STATE;                                  // If this fails, the watchdog will reset
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Reporting - Done Watering");
          lastPublish = millis();
        }
      }
      break;

    case REPORTING_STATE:
      if (!waiting)
      {
        publishTimeStamp = millis();
        waiting = true;                                         // Make sure we set the flag for flow through this case
        doneEnabled = false;                                    // If we get a valid response from Ubidots, this will be set back true
        sendToUbidots();
      }
      else if (waiting && doneEnabled)
      {
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Idle");
          lastPublish = millis();
        }
        state = IDLE_STATE;       // This is how we know if Ubidots got the data
        waiting = false;
      }
      else if (waiting && millis() >= (publishTimeStamp + webhookWaitTime))
      {
        state = ERROR_STATE;
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Error - Reporting Timed Out");
          lastPublish = millis();
        }
        waiting = false;
      }
      break;

    case ERROR_STATE:                               // Set up so I could have other error recovery options than just reset in the future
      if (!waiting)
      {
        if (resetCount > 3)
        {
          resetCount = 0;
          EEPROM.write(resetCountAddr,resetCount);    // If so, store incremented number - watchdog must have done This
          currentPeriod = Time.hour();  // Let's wait an hour to report again.
          if (verboseMode) {
            waitUntil(meterParticlePublish);
            Particle.publish("State","Excess Resets - 1 hour break");
            lastPublish = millis();
          }
          state =IDLE_STATE;
        }
        else {
          waiting = true;
          resetWaitTimeStamp = millis();
          if (verboseMode) {
            waitUntil(meterParticlePublish);
            Particle.publish("State","Resetting in 30 sec");
            lastPublish = millis();
          }
        }
      }
      if (millis() >= (resetWaitTimeStamp + resetWaitTime)) System.reset();
      break;
  }
}


void sendToUbidots()                                      // Houly update to Ubidots for serial data logging and analysis
{
  digitalWrite(donePin, HIGH);                            // We will pet the dog now so we have the full interval to water
  digitalWrite(donePin, LOW);                             // We set the delay resistor to 50k or 7 mins so that is the longest watering duration
  // Uncomment this next line only after you are sure your watchdog timer interval is greater than the Ubidots response period (about 5 secs)
  doneEnabled = false;                                    // Turns off watchdog petting - only a successful response will re-enable
  char data[256];                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Moisture\":%i, \"Watering\":%i, \"key1\":\"%s\", \"SoilTemp\":%i}",capValue, wateringMinutes, wateringContext, soilTemp);
  Particle.publish("AquaMaster_hook", data , PRIVATE);

}

int startStop(String command)                             // So we can manually turn on the water for testing and setup
{
  if (command == "1")
  {
    wateringMinutes = shortWaterMinutes;                  // Manual waterings are short
    strcpy(wateringContext,"User Initiated");             // Add the right context for publishing
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Watering - User Initiated");
      lastPublish = millis();
    }
    state = WATERING_STATE;
    return 1;
  }
  else if (command == "0")                                // This allows us to turn off the water at any time
  {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Stopped Watering - User Initiated");
      lastPublish = millis();
    }
    wateringStarted = 0;    // This will stop the watering
    return 1;
  }
  return 0;
}

int wateringEnabled(String command)                       // If I sense something is amiss, I can easily disable watering
{
  if (command == "1")                                   // Default - enabled
  {
    waterEnabled = 1;
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Watering Enabled");
      lastPublish = millis();
    }
    controlRegister = EEPROM.read(controlRegisterAddr);
    controlRegister = (0b00000010 | controlRegister);                    // Enable Watering
    EEPROM.write(controlRegisterAddr,controlRegister);                   // Write it to the register
    return 1;
  }
  else                            // Ensures no watering will occur
  {
    waterEnabled = 0;
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("State","Watering Disabled");
      lastPublish = millis();
    }
    controlRegister = EEPROM.read(controlRegisterAddr);
    controlRegister = (0b11111101 & controlRegister);                    // Disable Watering
    EEPROM.write(controlRegisterAddr,controlRegister);                   // Write it to the register
    return 1;
  }
}

int takeMeasurements(String command)
{
  if (command == "1")                                   // Default - enabled
  {
    state = SENSING_STATE;
    return 1;
  }
  else return 0;                                              // Never get here but if we do, let's be safe and disable
}

int getWiFiStrength()
{
    int wifiRSSI = WiFi.RSSI();
    if (wifiRSSI > 0) {
        sprintf(Signal, "Error");
    }else {
        int strength = map(wifiRSSI, -127, -1, 0, 5);
        sprintf(Signal, "%s: %d", levels[strength], wifiRSSI);
    }
    return 1;
}


int getMoisture()                                        // Here we get the soil moisture and characterize it to see if watering is needed
{
  capValue = sensor.getCapacitance();                     // capValue is typically between 300 and 700
  if ((capValue <= 400) || (capValue >= 520))
  {
    sprintf(Moisture, "Out of Range: %d", capValue);
    //return 0;   // Quick check for a valid value
  }
  int strength = map(capValue, 400, 520, 0, 5);           // Map - these values to cases that will use words that are easier to understand
  sprintf(Moisture, "%s: %d", capDescription[strength], capValue);
  return 1;
}

void NonBlockingDelay(int millisDelay)                    // Used for a non-blocking delay - will allow for interrrupts and Particle calls
{
  unsigned long commandTime = millis();
  while (millis() <= millisDelay + commandTime)
  {
    Particle.process();                                   // This ensures that we can still service Particle processes
  }
  return;
}

void weatherHandler(const char *event, const char *data)  // Extracts the expected rainfall for today from webhook response
{
  // Uses forecast JSON for Raleigh-Durham Airport
  // Response template gets current date and qpf_allday
  // Only look at the current day
  // JSON payload - http://api.wunderground.com/api/(my key)/forecast/q/nc/raleigh-durham.json
  // Response Template: "{{#forecast}}{{#simpleforecast}}{{#forecastday}}{{date.day}}~{{qpf_allday.in}}~{{/forecastday}}{{/simpleforecast}}{{/forecast}}"
  if (!data && verboseMode) {                                            // First check to see if there is any data
    waitUntil(meterParticlePublish);
    Particle.publish("Rainfall", "No Data");
    lastPublish = millis();
    return;
  }
  char strBuffer[30] = "";                                // Create character array to hold response
  strcpy(strBuffer,data);                                 // Copy into the array
  forecastDay = atoi(strtok(strBuffer, "\"~"));       // Use the delimiter to find today's date and expected Rainfall
  expectedRainfallToday = atof(strtok(NULL, "~"));
  snprintf(Rainfall,sizeof(Rainfall),"%4.2f",expectedRainfallToday);
}

void AquaMasterHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{watering.0.status_code}}"
  if (!data && verboseMode) {                                            // First check to see if there is any data
    waitUntil(meterParticlePublish);
    Particle.publish("AquaMaster", "No Data");
    lastPublish = millis();
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (verboseMode) {
      waitUntil(meterParticlePublish);
      Particle.publish("AquaMaster","Success");
      lastPublish = millis();
    }
    doneEnabled = true;                                   // Successful response - can pet the dog again
    digitalWrite(donePin, HIGH);                          // If an interrupt came in while petting disabled, we missed it so...
    digitalWrite(donePin, LOW);                           // will pet the dog just to be safe
  }
  else if (verboseMode)
  {
    waitUntil(meterParticlePublish);
    Particle.publish("AquaMaster", data);             // Publish the response code
    lastPublish = millis();
  }
}

void watchdogISR()                                        // Will pet the dog ... if petting is allowed
{
  if (doneEnabled)                                        // the doneEnabled is used for Ubidots and Watering to prevent lockups
  {
    digitalWrite(donePin, HIGH);                          // This is all you need to do to pet the dog low to high transition
    digitalWrite(donePin, LOW);
  }
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  EEPROM.write(timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in EEPROM
  time_t t = Time.now();
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  if (verboseMode) {
    waitUntil(meterParticlePublish);
    Particle.publish("Time",data);
    lastPublish = millis();
    waitUntil(meterParticlePublish);
    Particle.publish("Time",Time.timeStr(t));
    lastPublish = millis();
  }
  return 1;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    controlRegister = EEPROM.read(controlRegisterAddr);
    controlRegister = (0b00000001 | controlRegister);                    // Turn on verboseMode
    EEPROM.write(controlRegisterAddr,controlRegister);                        // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Set Verbose Mode");
    lastPublish = millis();
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    controlRegister = EEPROM.read(controlRegisterAddr);
    controlRegister = (0b11111110 & controlRegister);                    // Turn off verboseMode
    EEPROM.write(controlRegisterAddr,controlRegister);                        // Write it to the register
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Cleared Verbose Mode");
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}
