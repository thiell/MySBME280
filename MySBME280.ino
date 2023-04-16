//////////////////////////
//     Configuration    //
//////////////////////////

// Customize your device name. Ex: "Greenhouse", "Grow rack", "Top shelf", etc...
String Location = "Yankee New Baro";     // Location of the device, will be sent to the gateway with each device

#define BME280_SENSOR             // Enable to monitor temperature, humidity and pressure using a BME280 module
#define FORECAST                  // Enable forecast algorithm

// Use custom static node ID
// If disabled node ID will be automatically assigned
#define MY_NODE_ID 32

// Use custom parent node ID. Use 0 to force direct connection to the gateway.
// If disabled node ID will be automatically assigned
//#define MY_PARENT_NODE_ID 0

// Enable repeater feature for this node
//#define MY_REPEATER_FEATURE

// Advanced options
// Use low power transmission for RF24, better signal/noise ratio but shorter range
//#define MY_RF24_PA_LEVEL RF24_PA_LOW
// Enable OTA feature
#define MY_OTA_FIRMWARE_FEATURE         // Require eeprom (U3) on the PCB
// Enable message signing
//#define MY_SIGNING_ATSHA204             // Require ATSHA205 (U5) on the PCB and configured with your system key
// Enable debug prints to serial monitor
#define MY_DEBUG

// BME280 Barometric pressure sensor
// Adapt this constant: set it to the altitude above sealevel at your home location.
const float ALTITUDE = 2; // meters above sea level. Use your smartphone GPS to get an accurate value!

//////////////////////////
// End of configuration //
//////////////////////////

// MySensors
#define MY_RADIO_NRF24          // Enable and select radio type attached
#include <SPI.h>
#include <MySensors.h>
#define SN "MySBME280"         // Name of the sketch
#define SV "2.2.0"              // Version (2.0 : use MySensors 2.0)
boolean metric = true;          // Use SI by default
#define MESSAGEWAIT 500         // Wait a few ms between radio Tx

#if defined BME280_SENSOR
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#undef BME280_ADDRESS         // Undef BME280_ADDRESS from the BME280 library to easily override I2C address
#define BME280_ADDRESS (0x76) // Low = 0x76 , High = 0x77 (default on adafruit and sparkfun BME280 modules, default for library)
Adafruit_BME280 bme; // Use I2C
#define CHILD_ID_TEMP 2
#define CHILD_ID_HUM 3
#define CHILD_ID_BARO 6
const unsigned long SLEEP_TEMPHUMBARO = 60000;  // Sleep time between TempHum reads (in milliseconds), default 2min30 (150000ms)
unsigned long TempHumBaroMillis = 0;
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgBaro(CHILD_ID_BARO, V_PRESSURE);
MyMessage msgForecast(CHILD_ID_BARO, V_FORECAST);
#endif

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in the forecast algorithm
// get kPa/h by dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;
float dP_dt;

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}


// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}


void setup() {
  Serial.println(F("Starting setup()"));
  metric = getControllerConfig().isMetric;  // was getConfig().isMetric; before MySensors v2.1.1

#if defined BME280_SENSOR
  // Start up BME280 sensor
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or I2C address!"));
    while (1) {
      yield();
    }
  }
#endif

  Serial.println(F("Node ready to receive messages"));

}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  //sendSketchInfo("BME280 Sensor", "1.6");

  // Send the Sketch Version Information to the Gateway
  wait(MESSAGEWAIT);
  sendSketchInfo(SN, SV);

  // Register sensors to gw (they will be created as child devices)
#if defined BME280_SENSOR
  String bmeTempReg = "Temp " + Location;
  String bmeHumReg = "Hum " + Location;
  String bmeBaroReg = "Baro " + Location;
  wait(MESSAGEWAIT);
  present(CHILD_ID_TEMP, S_TEMP, bmeTempReg.c_str());
  wait(MESSAGEWAIT);
  present(CHILD_ID_HUM, S_HUM, bmeHumReg.c_str());
  wait(MESSAGEWAIT);
  present(CHILD_ID_BARO, S_BARO, bmeBaroReg.c_str());
#endif
}

// Loop
void loop() {

  unsigned long currentMillis = millis();       // Start timer

#if defined BME280_SENSOR
  if (currentMillis - TempHumBaroMillis >= SLEEP_TEMPHUMBARO) {    // Every SLEEP_TIME fetch data from sensors and send to controller
    TempHumBaroMillis = currentMillis;

    // Fetch temperature from sensor
    float temperature;
    temperature = bme.readTemperature();
    temperature = round(temperature * 10) / 10.0; // 0.1°C accuracy is sensistive enough
    if (!metric) {
      temperature = temperature * 9.0 / 5.0 + 32.0;
    }
    wait(MESSAGEWAIT);
    send(msgTemp.set(temperature, 1));
#if defined MY_DEBUG
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.println(metric ? F(" °C") : F(" °F"));
#endif

    // Fetch humidity from sensor
    float humidity;
    humidity = bme.readHumidity();
    humidity = round(humidity);     // 1% RH accuracy is sensistive enough
    // Clip humidity values to a valid range
    // If sent RH > 100% domoticz will ignore both temperature and humidity readings
    humidity = humidity > 100 ? 100 : humidity;
    humidity = humidity < 0   ? 100   : humidity;
    wait(MESSAGEWAIT);
    send(msgHum.set(humidity, 0));
#if defined MY_DEBUG
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F(" %"));
#endif

    // Fetch pressure from sensor
    float pressure;
    pressure = bme.readPressure() / 100.0F;
    pressure = ( pressure / pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255)); // Local pressure adjusted to sea level pressure using user altitude
    pressure = round(pressure * 10) / 10.0; // 0.1 hPa accuracy is sensistive enough
    wait(MESSAGEWAIT);
    send(msgBaro.set(pressure, 1));
#if defined MY_DEBUG
    Serial.print(F("Pressure: "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
#endif

    // Forecast
    int forecast = sample(pressure);
    send(msgForecast.set(weather[forecast]));
#if defined MY_DEBUG
    Serial.print(F("Forecast = "));
    Serial.println(weather[forecast]);
    Serial.println();
#endif
  }
#endif
}
