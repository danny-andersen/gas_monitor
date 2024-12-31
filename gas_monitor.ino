#include <Arduino.h>
#include <EncBreakout.h>
#include <GasBreakout.h>
#include <IOExpander.h>
#include <PotBreakout.h>
#include <WiFi.h>
#include <RunningMedian.h>

#include "LocalSettings.h"

#define BATTERY_VOLTAGE_PIN 0
#define SOUNDER_PIN 2
#define SENSOR_POWER_PIN 3
#define TEST_PIN GPIO_NUM_4
#define ADD_I2C 0x77
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) // 2 ^ GPIO_NUMBER in hex

#define BATTERY_ON_CHARGE 4.1
#define LOW_BATTERY_VOLTAGE 3.40
#define VERY_LOW_BATTERY_VOLTAGE 3.30
#define CRITICALLY_LOW_BATTERY_VOLTAGE 3.20
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define NORMAL_TIME_TO_SLEEP 30   /* Time ESP32 will go to sleep (in seconds) */
#define WARNING_TIME_TO_SLEEP 10  /* Time ESP32 will go to sleep if in warning condition (in seconds) */
#define HIGH_TIME_TO_SLEEP 5      /* Time ESP32 will go to sleep if in high alarm condition (in seconds) */
#define CRITICAL_TIME_TO_SLEEP 2  /* Time ESP32 will go to sleep in critical alarm (in seconds) */

#define FLASH_LED true
#define CACHE_SIZE 32   // Number of results that are cached
#define RESULTS_LEN 100 // Number of chars in a result string that are cached
#define FILTER_SIZE 9
#define BATTERY_HISTORY_SIZE 5 // Number of previous battery voltages to get the median over

// Reducer response is x10 for 50ppm, x100 for 1000ppm
//  CO ~x2 for 10ppm (9ppm is safe limit for CO)
#define CO_WARNING_THRESHOLD 0.5
// CO 10x == 50ppm is safe 8 hour limit
#define CO_HIGH_THRESHOLD 0.1
// CO 12.5x ~= 70ppm is critical limit - sound alarm
#define CO_CRITICAL_THRESHOLD 0.08
// NH3 25ppm recommended max, 35ppm is 15 min max, 50ppm is critical
#define NH3_REDUCER_WARNING_THRESHOLD 0.6
#define NH3_REDUCER_HIGH_THRESHOLD 0.5
#define NH3_REDUCER_CRITICAL_THRESHOLD 0.4
#define NH3_NH3_WARNING_THRESHOLD 0.6
#define NH3_NH3_HIGH_THRESHOLD 0.5
#define NH3_NH3_CRITICAL_THRESHOLD 0.4
// NO2 max limit is 1ppm, 3ppm is high, 5 ppm is critical
// No2 ~x7 for 1
#define NO2_WARNING_THRESHOLD 7
// NO2 x20 for 3
#define NO2_HIGH_THRESHOLD 15
// NO2 x30 for 5
#define NO2_CRITICAL_THRESHOLD 30

GasBreakout gas(Wire, 0x19);
RunningMedian medianFilter(FILTER_SIZE);
RunningMedian medianBatteryFilter(BATTERY_HISTORY_SIZE);

const float alpha = 0.1; // Smoothing factor (adjust as needed)
float filteredGasR = 0;

RTC_NOINIT_ATTR struct
{
  // Temp circular buffer for results strings, to send to control station when half full
  char resultsCache[CACHE_SIZE][RESULTS_LEN];
  // When the sample was gathered, based on total active time
  // We are only interested in sending the delta between now and then - the controlstation will turn this into wall clock time
  unsigned long resultsTimestamp[CACHE_SIZE];
  // Number of results in buffer not sent
  uint8_t resultsCacheCnt;
  // crude way of determining if powered on -
  // lastCacheCnt should be (resultsCacheCnt - 1) or 7 or 0 if resultsCacheCnt = 0,
  // otherwise we have been powered on and so need to reset everything
  uint8_t lastCacheCnt;
  // Total time since last power on event, including sleep time
  unsigned long activeTimeMs;
  // Used to initialise filters after power on
  uint8_t powerOnCnt;
  // Last alarm status - to allow for some time deadbanding
  uint8_t lastNH3AlarmStatus;
  // Whether the current critical alarm has been acknowledged and so whether sounder should be on or off
  bool criticalAlarmAcked;
  // Used to create a median filter to filter out sporadic large values
  uint8_t filterCnt;
  float reducerRawValues[FILTER_SIZE];
  float nh3RawValues[FILTER_SIZE];
  float oxRawValues[FILTER_SIZE];
  // Totals used to create the current average to determine if a reading is above normal.
  // After a count of a 1000, the total and sample count is divided by 10 to give an historic average that is more sensitive to gradual trends in recent data
  float totalReducer;
  float totalNh3;
  float totalOx;
  float totalSampleCnt;
  // Average out last samples of battery voltage to ensure we dont shutdown too early if get a rogue result
  float batteryVoltageHistory[BATTERY_HISTORY_SIZE];
} cache;

int readResponse(NetworkClient *client)
{
  unsigned long timeout = millis();
  while (client->available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(">>> Client Timeout !");
      client->stop();
      return -1;
    }
  }
  int charsReturned = client->available();
  // Read all the lines of the reply from server and print them to Serial
  int ok = -1;
  while (client->available())
  {
    String line = client->readStringUntil('\r');
    // Look for OK
    ok = line.indexOf("200 OK");
    // Serial.print(line);
    if (ok > 0)
    {
      client->flush();
      break;
    }
  }
  return ok;
}

float readBattery(bool powerOn)
{
  uint32_t value = 0;
  float currentValue = 0;
  float retValue = 0;
  float rounds = 11.0;

  // set the ADC resolution to 12 bits (0-4095) for battery voltage
  analogReadResolution(12);
  // to avoid noise, sample the pin several times and average the result
  for (int i = 1; i <= rounds; i++)
  {
    value += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
    delay(1);
  }
  currentValue = value / rounds;
  // if (Serial) Serial.println("Current BV direct: " + String(currentValue*2/1000));

  if (powerOn)
  {
    // Initialise history array
    for (int i = 0; i < BATTERY_HISTORY_SIZE; i++)
    {
      cache.batteryVoltageHistory[i] = currentValue;
    }
    retValue = currentValue;
  }
  else
  {
    // Add the latest value to the array and calculate the median, to remove any sporadic values
    // Remove oldest value
    for (int i = 1; i < BATTERY_HISTORY_SIZE; i++)
    {
      cache.batteryVoltageHistory[i - 1] = cache.batteryVoltageHistory[i];
    }
    // Add latest at the end
    cache.batteryVoltageHistory[BATTERY_HISTORY_SIZE - 1] = currentValue;
    // Calculate the median
    medianBatteryFilter.clear();
    for (int i = 0; i < BATTERY_HISTORY_SIZE; i++)
    {
      medianBatteryFilter.add(cache.batteryVoltageHistory[i]);
      // if (Serial) Serial.println("Cached BV: " + String(cache.batteryVoltageHistory[i]*2/1000));
    }
    retValue = medianBatteryFilter.getMedian();
  }
  // due to the voltage divider (1M+1M) values must be multiplied by 2
  // and convert mV to V
  //  if (Serial) Serial.println("Median BV: " + String(retValue*2/1000));
  return retValue * 2.0 / 1000;
}

uint8_t checkAlarmCondition(GasBreakout::Reading *gas)
{
  // Return status:
  // alarm status = 0 -> no alarm, Bit 5+4 -> NO2 issue, 3+2 -> NH3 issue, 1+0 -> CO issue
  // 0x10, 0x20, 0x30 -> 16, 32, 48 NO2 warning / high / critical
  // 0x08, 0x04, 0xC0 -> 4, 8, 12 -> NH3 issues
  // 1, 2, 3 -> CO issue

  uint8_t alarmStatus = 0;
  // Work out difference to current average for CO and NH3 sensors; if both trigger then it is Ammonia, ethanol, hydrogen or methane/propane/butane
  // If only Reducer triggers it is CO
  // If only NH3 triggers it is likely to be NH3/Ammonia
  // Important note: Reducing + NH3 sensors decrease in resistance with increasing gas. The Oxidiser increases in resistance
  // Have a valid reading
  if (cache.totalSampleCnt >= 100)
  {
    // Have a good average sample value to test
    // Now check for NH3 - if this has triggered then likely that its NH3, not CO that has triggered the sensor
    float currentNh3Avg = cache.totalNh3 / cache.totalSampleCnt;
    float currentReducerAvg = cache.totalReducer / cache.totalSampleCnt;
    if (gas->nh3 <= currentNh3Avg * NH3_NH3_CRITICAL_THRESHOLD && gas->reducing <= currentReducerAvg * NH3_REDUCER_CRITICAL_THRESHOLD)
    {
      // Could be ammonia, ethanol or propane/methane/butane detected
      //  Serial.println("Alarm 6: Reading = " + String(gas->nh3) + " Avg: " + String(currentNh3Avg));
      alarmStatus |= 0x0C;
      cache.lastNH3AlarmStatus = alarmStatus;
    }
    else if (gas->nh3 <= currentNh3Avg * NH3_NH3_HIGH_THRESHOLD && gas->reducing <= currentReducerAvg * NH3_REDUCER_HIGH_THRESHOLD)
    {
      // Could be ammonia, ethanol or propane/methane/butane detected
      //  Serial.println("Alarm 5: Reading = " + String(gas->nh3) + " Avg: " + String(currentNh3Avg));
      alarmStatus |= 0x08;
      cache.lastNH3AlarmStatus = alarmStatus;
    }
    else if (gas->nh3 <= currentNh3Avg * NH3_NH3_WARNING_THRESHOLD && gas->reducing <= currentReducerAvg * NH3_REDUCER_WARNING_THRESHOLD)
    {
      // Could be ammonia, ethanol or propane/methane/butane detected
      //  Serial.println("Alarm 4: Reading = " + String(gas->nh3) + " Avg: " + String(currentNh3Avg));
      alarmStatus |= 0x04;
      cache.lastNH3AlarmStatus = alarmStatus;
    }
    if (gas->reducing <= currentReducerAvg * CO_CRITICAL_THRESHOLD && (cache.lastNH3AlarmStatus & 0x0C) == 0)
    {
      // CO Alarm only if no NH3 alarm
      //  Serial.println("Alarm 3: Reading = " + String(gas->reducing) + " Avg: " + String(currentReducerAvg));
      alarmStatus |= 0x03;
    }
    else if (gas->reducing <= currentReducerAvg * CO_HIGH_THRESHOLD && (cache.lastNH3AlarmStatus & 0x0C) == 0)
    {
      // CO Alarm only if no NH3 alarm
      //  Serial.println("Alarm 2: Reading = " + String(gas->reducing) + " Avg: " + String(currentReducerAvg));
      alarmStatus |= 0x02;
    }
    else if (gas->reducing <= currentReducerAvg * CO_WARNING_THRESHOLD && (cache.lastNH3AlarmStatus & 0x0C) == 0)
    {
      // CO Alarm only if no NH3 alarm
      //  Serial.println("Alarm 1: Reading = " + String(gas->reducing) + " Avg: " + String(currentReducerAvg));
      alarmStatus |= 0x01;
    }
    if (gas->reducing > currentReducerAvg * CO_WARNING_THRESHOLD)
    {
      // Reducer has recovered so can reset any historic NH3 alarm
      cache.lastNH3AlarmStatus = 0;
    }
    // Check for Nitrogen Dioxide
    float currentOxAvg = cache.totalOx / cache.totalSampleCnt;
    if (gas->oxidising >= currentOxAvg * NO2_WARNING_THRESHOLD)
    {
      // NO2 warning
      //  Serial.println("Alarm 7: Reading = " + String(gas->oxidising) + " Avg: " + String(currentOxAvg));
      alarmStatus |= 0x10;
    }
    if (gas->oxidising >= currentOxAvg * NO2_HIGH_THRESHOLD)
    {
      // Serial.println("Alarm 8: Reading = " + String(gas->oxidising) + " Avg: " + String(currentOxAvg));
      alarmStatus |= 0x20;
    }
    if (gas->oxidising >= currentOxAvg * NO2_CRITICAL_THRESHOLD)
    {
      // Serial.println("Alarm 9: Reading = " + String(gas->oxidising) + " Avg: " + String(currentOxAvg));
      alarmStatus |= 0x30;
    }
    float reducerChgPerc = 100 * (gas->reducing - currentReducerAvg) / currentReducerAvg;
    // float nh3ChgPerc = 100 * (gas->nh3 - currentNh3Avg) / currentNh3Avg;
    // float oxChgPerc = 100 * (gas->oxidising - currentOxAvg) / currentOxAvg;
    // Set bit 7 if there has been a significant change in a particular result from the average reducer
    // This will increase the sample rate and also keep the sensor awake and powered on - the sensitivity graphs are based on power being continuously ON
    if (reducerChgPerc <= -20)
    {
      alarmStatus |= 0x80;
    }
  }

  // if (Serial) Serial.println("Alarm status: " + String(alarmStatus));
  if ((alarmStatus & 0x7F) == 0)
  {
    // Only add current values to average if not in an alarm status
    // As the average is the normal baseline
    cache.totalReducer += gas->reducing;
    cache.totalNh3 += gas->nh3;
    cache.totalSampleCnt++;
    cache.totalOx += gas->oxidising;
    if (cache.totalSampleCnt >= 1000)
    {
      // Divide sample count and total by 10.
      // This allows more recent samples to have a greater impact on the average, allowing for some drift in the sensor
      //  Serial.println("Scaling down total samplecnt" );
      cache.totalSampleCnt /= 10;
      cache.totalReducer /= 10;
      cache.totalNh3 /= 10;
      cache.totalOx /= 10;
    }
  }
  return alarmStatus;
}

void driveSounder(uint8_t alarmStatus)
{
  // if (alarmStatus & 0x03 == 0x03 || alarmStatus & 0x0C == 0x0C || alarmStatus & 0x30 == 0x30)
  if (alarmStatus & 0x7F > 0)
  {
    // Sound on any gas detection
    digitalWrite(SOUNDER_PIN, HIGH); // turn the Piezo on to make it buzz.
  }
}

GasBreakout::Reading readSensor()
{
  int tries = 0;
  bool gasReady = false;
  GasBreakout::Reading reading = {0};
  while (!gasReady && tries++ < 20)
  {
    gasReady = gas.initialise();
    if (gasReady)
    {
      if (Serial)
        Serial.println("MICS6814 - Initialised");
      reading = gas.readAll();
      if (Serial)
      {
        Serial.print("Reducing:");
        Serial.print(reading.reducing / 1000);
        Serial.print(",");
        // Serial.print("NH3 (+ H2, C3H8): ");
        Serial.print("NH3:");
        Serial.print(reading.nh3 / 1000);
        Serial.print(",");
        // Serial.print("Ox (NO2, NO, C2H50H)): ");
        Serial.print("Ox:");
        Serial.println(reading.oxidising / 1000);
      }
      // Filter values
      cache.reducerRawValues[cache.filterCnt] = reading.reducing;
      cache.nh3RawValues[cache.filterCnt] = reading.nh3;
      cache.oxRawValues[cache.filterCnt] = reading.oxidising;
      cache.filterCnt++;
      if (cache.filterCnt >= FILTER_SIZE)
      {
        cache.filterCnt = 0;
      }
      // Load filter with last n raw values. Last value is the median value
      medianFilter.clear();
      for (int i = 0; i < FILTER_SIZE; i++)
      {
        medianFilter.add(cache.reducerRawValues[i]);
      }
      reading.reducing = medianFilter.getMedian();
      if (Serial)
      {
        Serial.print("Median Reducing: ");
        Serial.println(reading.reducing / 1000);
      }
      medianFilter.clear();
      for (int i = 0; i < FILTER_SIZE; i++)
      {
        medianFilter.add(cache.nh3RawValues[i]);
      }
      reading.nh3 = medianFilter.getMedian();
      medianFilter.clear();
      for (int i = 0; i < FILTER_SIZE; i++)
      {
        medianFilter.add(cache.oxRawValues[i]);
      }
      reading.oxidising = medianFilter.getMedian();
    }
    else
    {
      if (Serial)
        Serial.println("MICS6814 - Not Initialised");
      delay(100);
    }
  }
  return reading;
}

void cacheResults(uint8_t alarmStatus, float batteryVoltage, GasBreakout::Reading *reading)
{
  String resultStr = "&bv=" + String(batteryVoltage) + "&a=" + String(alarmStatus);
  resultStr += "&red=" + String(reading->reducing) + "&nh3=" + String(reading->nh3) + "&ox=" + String(reading->oxidising);
  float currentOxAvg = cache.totalOx / cache.totalSampleCnt;
  float currentNh3Avg = cache.totalNh3 / cache.totalSampleCnt;
  float currentReducerAvg = cache.totalReducer / cache.totalSampleCnt;
  float reducerChgPerc = 100 * (reading->reducing - currentReducerAvg) / currentReducerAvg;
  float nh3ChgPerc = 100 * (reading->nh3 - currentNh3Avg) / currentNh3Avg;
  float oxChgPerc = 100 * (reading->oxidising - currentOxAvg) / currentOxAvg;
  resultStr += "&redp=" + String(reducerChgPerc) + "&nh3p=" + String(nh3ChgPerc) + "&oxp=" + String(oxChgPerc);
  // Add to cache in RTC
  resultStr.toCharArray(cache.resultsCache[cache.resultsCacheCnt], RESULTS_LEN);
  cache.resultsTimestamp[cache.resultsCacheCnt] = cache.activeTimeMs + millis();
  if (Serial)
    Serial.println("Time: " + String(cache.resultsTimestamp[cache.resultsCacheCnt]) + " Result: " + String(cache.resultsCacheCnt) + " Caching len: " + String(resultStr.length()));
  cache.lastCacheCnt = cache.resultsCacheCnt;
  cache.resultsCacheCnt++;
  if (cache.resultsCacheCnt >= CACHE_SIZE)
  {
    // Buffer full - rotate buffer so lose the first (oldest) one
    for (int i = 0; i < (CACHE_SIZE - 1); i++)
    {
      for (int j = 0; j < RESULTS_LEN; j++)
      {
        cache.resultsCache[i][j] = cache.resultsCache[i + 1][j];
      }
    }
    cache.resultsCacheCnt--;
  }
}

void sendResults()
{
  if (Serial)
    Serial.println("Sending results: " + String(cache.resultsCacheCnt));
  WiFi.setTxPower(WIFI_POWER_7dBm);
  WiFi.begin(ssid, password);
  int maxWait = 5000; // max wait for connection of 5seconds
  while (WiFi.status() != WL_CONNECTED && maxWait > 0)
  {
    delay(200);
    maxWait -= 200;
    Serial.print(WiFi.status());
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    if (Serial)
    {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }

    NetworkClient client;
    // String footer = String(" HTTP/1.1\r\n") + "Host: " + String(host) + "\r\n" + "Connection: close\r\n\r\n";
    String footer = String(" HTTP/1.1\r\n");
    int msgsSent = 0;
    int retVal = 0;
    for (int i = 0; i < cache.resultsCacheCnt && retVal >= 0; i++)
    {
      if (client.connect(host, httpPort))
      {
        unsigned long now = cache.activeTimeMs + millis();
        long delta = now - cache.resultsTimestamp[i];
        if (delta < 0)
        {
          // time has rolled over - just use the sampleTime
          delta = now;
        }
        if (Serial)
          Serial.println("Sample: " + String(i) + " Time: " + String(cache.resultsTimestamp[i]) + " Delta: " + String(delta));
        client.println("GET /airqual?&delta=" + String(delta) + String(cache.resultsCache[i]) + footer);
        client.println();
        retVal = readResponse(&client);
        if (retVal < 0)
        {
          // Message failed to be send - stop sending
          break;
        }
        else
        {
          msgsSent++;
        }
        client.stop();
      }
    }
    if (msgsSent == cache.resultsCacheCnt)
    {
      cache.resultsCacheCnt = 0;
      cache.lastCacheCnt = 0;
    }
    else if (msgsSent > 0)
    {
      // Some havent been sent - shuffle down to send later
      int numToShuffle = cache.resultsCacheCnt - msgsSent;
      for (int i = 0; i < numToShuffle; i++)
      {
        for (int j = 0; j < RESULTS_LEN; j++)
        {
          cache.resultsCache[i][j] = cache.resultsCache[msgsSent + i][j];
        }
      }
      cache.resultsCacheCnt = numToShuffle;
      cache.lastCacheCnt = numToShuffle - 1;
    }
    WiFi.disconnect(true);
  }
  else
  {
    if (Serial)
    {
      Serial.println("");
      Serial.println("WiFi failed to connect in time");
    }
  }
}

void powerOn()
{
  cache.resultsCacheCnt = 0;
  cache.lastCacheCnt = 0;
  cache.activeTimeMs = 0;
  for (int i; i < FILTER_SIZE; i++)
  {
    cache.reducerRawValues[i] = 0;
    cache.nh3RawValues[i] = 0;
    cache.oxRawValues[i] = 0;
  }
  cache.filterCnt = 0;
  cache.totalNh3 = 0;
  cache.totalOx = 0;
  cache.totalReducer = 0;
  cache.totalSampleCnt = 0;

  cache.powerOnCnt = FILTER_SIZE * 4;
  // Read battery and set up average array
  readBattery(true);
}

/*
 * Setup is called each time the controller wakes up from a deep sleep so all of the logic is here
 */

void setup(void)
{
  Serial.begin(115200);
  if (Serial)
  {
    Serial.println("Setup start");
  }
  // See if the test button has been pressed - if so turn on the sounder
  pinMode(TEST_PIN, INPUT_PULLUP);
  bool inTestMode = !digitalRead(TEST_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SOUNDER_PIN, OUTPUT);
  digitalWrite(SOUNDER_PIN, LOW); // Turn sounder off
  // Turn on power to the sensor
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(200);   // Let sensor power up
  Wire.begin(); // I2C mode
  if (esp_reset_reason() == ESP_RST_POWERON)
  {
    Serial.printf("ESP was just switched ON\r\n");
    if (FLASH_LED)
    {
      digitalWrite(LED_BUILTIN, HIGH); // We have a USB power on - turn the LED on
    }
    powerOn();
  }
  else if (Serial && esp_reset_reason() == ESP_RST_DEEPSLEEP)
  {
    int wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
    }
  }
  if (cache.resultsCacheCnt < CACHE_SIZE || (cache.resultsCacheCnt == 0 && cache.lastCacheCnt == 0) || (cache.lastCacheCnt == cache.resultsCacheCnt - 1))
  {
    // This is OK
  }
  else
  {
    if (Serial)
      Serial.println("Last result count: " + String(cache.lastCacheCnt) + " does not match cache count: " + String(cache.resultsCacheCnt) + " - forcing variable setup");
    powerOn();
  }
  bool stayAwake = false;
  uint64_t sleepTimeSecs = NORMAL_TIME_TO_SLEEP;
  float batteryVoltage = 0.0;
  stayAwake = false;
  if (inTestMode)
  {
    // Turn on led and drive sounder while button is pressed
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(SOUNDER_PIN, HIGH); // Turn sounder on
    while (inTestMode)
    {
      inTestMode = !digitalRead(TEST_PIN);
      delay(100);
    }
    digitalWrite(SOUNDER_PIN, LOW); // Turn sounder on
    // Test button has been released - now carry on as though we have been asleep
  }

  // read battery voltage
  batteryVoltage = readBattery(false);
  if ((Serial || batteryVoltage > BATTERY_ON_CHARGE || cache.powerOnCnt > 0) && FLASH_LED)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Serial USB plugged in, we are on charge or have just been turned on - turn the LED on as not running from battery
  }
  // if (Serial)
  //   Serial.printf("Voltage: %4.3f V\r\n", batteryVoltage);

  int alarmStatus = 0;
  GasBreakout::Reading reading = readSensor();
  if (cache.powerOnCnt == 0 && reading.ref != 0)
  {
    // Not in powerOnCnt settle period and we have readings - check whether in alarm status
    alarmStatus = checkAlarmCondition(&reading);
    // Drive Piezo if in critical alarm
    driveSounder(alarmStatus);
    // Stash reading in the cache
    cacheResults(alarmStatus, batteryVoltage, &reading);
  }
  if (cache.powerOnCnt == 0 && !alarmStatus)
  {
    // Only power off the sensor until we have a settled set of results after initial power up
    // Also leave it on if there has been a significant change in sensor readings
    // Otherwise turn off the power
    digitalWrite(SENSOR_POWER_PIN, LOW); // Turn off  power to the sensor
  }
  // Send cached data when half full or if there is an alarm or if battery critical and something to send
  if (cache.resultsCacheCnt >= CACHE_SIZE / 2 || (cache.resultsCacheCnt > 0 && ((alarmStatus & 0x7F) > 0 || batteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE)))
  {
    sendResults();
  }
  if (batteryVoltage > BATTERY_ON_CHARGE && FLASH_LED)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (batteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE)
  {
    // if battery is below LOW_BATTERY_VOLTAGE but still above CRITICALLY_LOW_BATTERY_VOLTAGE,
    // stop doing the regular work
    // when put on charge the device will wakeup after a while and recognize voltage is OK
    // this way the battery can run low, put still wakeup without physical interaction
    if (Serial)
      Serial.println("Battery critically low, hibernating...");

    // switch off everything that might consume power
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);

    // esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_OFF);

    // disable all wakeup sources
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    cache.activeTimeMs += millis();
    esp_deep_sleep_start();

    return;
  }

  if (cache.powerOnCnt <= 0 && sleepTimeSecs != CRITICAL_TIME_TO_SLEEP && batteryVoltage < LOW_BATTERY_VOLTAGE)
  {
    // Not in alarm but battery is running low
    if (Serial)
      Serial.println("Battery low, sleeping for longer...");
    // sleep ~3 minutes if battery is CRITICALLY_LOW_BATTERY_VOLTAGE to VERY_LOW_BATTERY_VOLTAGE
    // sleep ~1 minutes if battery is VERY_LOW_BATTERY_VOLTAGE to LOW_BATTERY_VOLTAGE
    sleepTimeSecs = (batteryVoltage >= VERY_LOW_BATTERY_VOLTAGE) ? 1 * 60ULL : 3 * 60ULL;
  }
  if (cache.powerOnCnt > 0)
  {
    // If only just powered on or hard reset - ignore the first set of readings and read every second
    cache.powerOnCnt--;
    // Only sleep briefly as this increases the sensitivity of the sensor
    // Do this at initial start up and when we detect a significant change in gas resistance.
    sleepTimeSecs = 1;
  }
  if (alarmStatus > 0)
  {
    // Reduce sleep time if there is something in the air...
    // Only sleep briefly as this increases the sensitivity of the sensor
    // Do this when we detect a significant change in gas resistance.
    sleepTimeSecs = CRITICAL_TIME_TO_SLEEP;
    // if (alarmStatus & 0x02 || alarmStatus & 0x04 || alarmStatus == 0x10) {
    //   sleepTimeSecs = HIGH_TIME_TO_SLEEP;
    // } else if (alarmStatus & 0x03 || alarmStatus & 0x0C || alarmStatus == 0x30) {
    //   sleepTimeSecs = CRITICAL_TIME_TO_SLEEP;
    // }
    // Delay dont sleep, to keep sensor power on
    delay(sleepTimeSecs * 1000);
    sleepTimeSecs = 0;
  }
  if (Serial)
  {
    Serial.println("Setup ESP32 to sleep for " + String(sleepTimeSecs) + " Seconds");
    // Let Serial flush before sleeping
    Serial.flush();
    delay(2000);
  }
  esp_sleep_enable_timer_wakeup(sleepTimeSecs * uS_TO_S_FACTOR);
  esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(TEST_PIN), ESP_EXT1_WAKEUP_ANY_LOW);
  cache.activeTimeMs += millis() + (sleepTimeSecs * 1000);
  esp_deep_sleep_start();
}

void loop(void)
{
  Serial.println("Should never be printed");
}
