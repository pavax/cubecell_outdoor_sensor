#include "LoRaWan_APP.h"
#include "RunningMedian.h"
#include <Wire.h>
#include <simple_logger.h>
#include "credentials.h"
#include "softSerial.h"
#include "HT_SH1107Wire.h"
#include "Seeed_BME280.h"

#include "SHT40AD1BSensor.h"

extern SH1107Wire display;

#define DEFAULT_DUTY_CYCLE 1000 * 60 * 10

#define TIMEOUT_MS 2000

#define WAKE_UP_PIN USER_KEY

#define DEFAULT_LOG_LEVEL logger::Debug  // DEBUG: set to Debug for more logging statements or to None for no logs

#define SHT_READINGS 10

#define DISTANCE_READINGS 8
#define DISTANCE_MAX_WAIT_TIME 5000
#define DISTANCE_MAX_SERIAL_DATA_LENGTH 4

#define ADC_WINDSPEED_READINGS 20
#define ADC_WINDSPEED_PIN ADC3
#define ADC_WINDSPEED_VOLTAGE_SCALING_FACTOR 0.5922476
#define WIND_SPEED_MAX_SAMPLES 20

#define ADC_WIND_DIRECTION_READINGS 20
#define ADC_WIND_DIRECTION_PIN ADC2
#define ADC_WIND_DIRECTION_VOLTAGE_SCALING_FACTOR 0.7372464

#define RAIN_SENSOR_MAX_RETRY 3
#define RAIN_SENSOR_INTERUPT_PIN GPIO6
#define RAIN_SENSOR_DATA_LINE_PATTERN "%*s %s %[^,] , %*s %s %*s %*s %s %*s %*s %s"
#define RAIN_SENSOR_MAX_SERIAL_DATA_LENGTH 80
#define RAIN_SENSOR_EVENT_MAX_FINISHED_WAITING_TIME 4 * 60 * 60 * 1000  // 4h
#define RAIN_SENSOR_TX_PIN GPIO4
#define RAIN_SENSOR_RX_PIN GPIO7

#define BME_READINGS 5

#define DEFAULT_BACKGROUND_MEASURE_INTERVAL 1000 * 60;  // every 60sec

static TimerEvent_t backgroundMeasureTimer;

/**
   SHT-40 Sensor
*/
SHT40AD1BSensor shtSensor(&Wire1);


/**
   Ultrasonic Distance Sensor
*/
RunningMedian distanceReadingSamples = RunningMedian(DISTANCE_READINGS);

/**
  Wind Sensor
*/
RunningMedian windSpeedReadingSamples = RunningMedian(WIND_SPEED_MAX_SAMPLES);

/**
   Rain Sensor
*/

softSerial rainSensorSerial(RAIN_SENSOR_TX_PIN, RAIN_SENSOR_RX_PIN);


/**
  BME280
*/
BME280 bme280(&Wire1);

static int32_t temperature, temperature2;

static uint32_t batteryVoltage, uptimeCount, distance, windSpeedVoltageMedian, rainEventCounter, pressure, humidity, humidity2, windSpeedVoltageMax;

static uint8_t serialCode, windDirection;

static unsigned long lastRainDetectionTime = 0;

static bool accelWoke = false;

static bool keepLedStatusEnabled = false;

static double acc, eventAcc, totalAcc, rInt;

static char buffer[RAIN_SENSOR_MAX_SERIAL_DATA_LENGTH];

static char scheduledRainCommand = 0;

uint32_t backgroundMeasureInterval = DEFAULT_BACKGROUND_MEASURE_INTERVAL;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = DEFAULT_DUTY_CYCLE;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;


void blinkRGB(uint32_t color, int times = 3, int blinkTime = 500) {
  if (!LoRaWAN.isRgbEnabled()) {
    return;
  }
  for (int i = 0; i < times; i++) {
    turnOnRGB(color, blinkTime);
    turnOnRGB(0, blinkTime);
  }
}

void onRainDetected() {
  if (deviceState == DEVICE_STATE_SLEEP) {
    rainEventCounter++;
    lastRainDetectionTime = millis();
    logger::debug(F("Rain Sensor: Detected rain"));
    blinkRGB(0x00FF21, 3, 200);
    turnOffRGB();
  }
}

void sendRainSensorCommand(char command) {
  logger::debug(F("send command: %c"), command);
  rainSensorSerial.printf(";TEST\r\n%c\r\n", command);
  rainSensorSerial.flush();
}

void processRainSensorDataLine(char dataLine[]) {
  char accB[7], eventAccB[7], totalAccB[7], rIntB[7], unit[4];
  sscanf(dataLine, RAIN_SENSOR_DATA_LINE_PATTERN, &accB, &unit, &eventAccB, &totalAccB, &rIntB);
  if (strncmp(unit, "in", 2) == 0) {
    logger::warn(F("wrong unit - expected metric"));
    return;
  }

  acc = atof(accB);

  double currrentEventAcc = atof(eventAccB);
  if (eventAcc > 0 && currrentEventAcc == 0) {
    logger::debug(F("EventAcc was reset"));
    rainEventCounter = 0;
  }

  eventAcc = currrentEventAcc;
  totalAcc = atof(totalAccB);
  rInt = atof(rIntB);
}


void initManualRun() {
  logger::set_level(logger::Debug);

  digitalWrite(Vext, LOW);
  delay(100);

  LoRaWAN.enableRgb();
  turnOnRGB(0x005050, 250);
  turnOnRGB(0x002450, 250);
  turnOnRGB(0x000050, 250);
  turnOnRGB(0, 0);

  initDisplay();
}

void initDisplay() {
  LoRaWAN.enableDisplay();
  display.screenRotate(ANGLE_180_DEGREE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
}

void displayUpTimeCount() {
  if (LoRaWAN.isDisplayEnabled()) {
    logger::debug(F("displayUpTimeCount"));
    display.clear();

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);

    display.drawString(58, 5, F("Daten messen..."));
    display.drawHorizontalLine(0, 24, 128);

    sprintf(buffer, "%d", uptimeCount);
    display.drawString(58, 33, buffer);
    display.display();
  }
}

void displaySplash() {
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);

  display.drawString(58, 12, F("Weather Station"));
  display.drawHorizontalLine(0, 33, 128);
  display.drawString(58, 40, F("Version 1.1"));
  display.drawString(58, 52, F("(c) Patrick Dobler"));

  display.display();
}


bool processRainSerialData() {
  int len = 0;
  while (len = rainSensorSerial.readBytesUntil('\n', buffer, RAIN_SENSOR_MAX_SERIAL_DATA_LENGTH)) {
    buffer[len] = 0;
    logger::debug(buffer);

    if (strncmp(buffer, "Acc", 3) == 0 && strstr(buffer, "TotalAcc") != NULL) {
      processRainSensorDataLine(buffer);
      return true;
    }

    if (strncmp(buffer, "Event", 5) == 0) {
      logger::debug(F("Rain Event was detected (Code: 20)"));
      serialCode = 20;
    } else if (strncmp(buffer, "LensBad", 7) == 0) {
      logger::debug(F("Lense Bad Info Found (Code: 21)"));
      serialCode = 21;
    } else if (strncmp(buffer, "Reset", 5) == 0) {
      logger::debug(F("Rain Sensor was reset (Code: 22)"));
      serialCode = 22;
    } else if (strncmp(buffer, "PwrDays", 7) == 0) {
      logger::debug(F("Rain Sensor restarted (Code: 23)"));
      serialCode = 23;
    } else if (strncmp(buffer, "EmSat", 5) == 0) {
      logger::debug(F("EmSat detected (Code: 24)"));
      serialCode = 24;
    }

    return false;
  }
}

void readRainSensor() {
  serialCode = 0;

  if (scheduledRainCommand != 0) {
    turnOnRGB(0x00FFFF, 500);  // cyan
    logger::info(F("Execute scheduled command: %c"), scheduledRainCommand);
    sendRainSensorCommand(scheduledRainCommand);
    scheduledRainCommand = 0;
    delay(5000);
    turnOffRGB();
  }

  if (rainEventCounter == 0) {
    logger::debug(F("No need to fetch rain-sensor data"));
    serialCode = 1;
    return;
  }

  if (rainEventCounter > 0 && (millis() - lastRainDetectionTime >= RAIN_SENSOR_EVENT_MAX_FINISHED_WAITING_TIME)) {
    logger::debug(F("No rain detected since a long time: reset rain event"));
    rainEventCounter = 0;
    serialCode = 8;
    return;
  }

  int retryCount = 0;
  bool successRead = false;
  while (!successRead && retryCount <= RAIN_SENSOR_MAX_RETRY) {
    sendRainSensorCommand('r');
    successRead = processRainSerialData();
    if (successRead) {
      turnOnRGB(0xCD00FF, 500);  // purple
      break;
    }
    retryCount++;
    if (LoRaWAN.isRgbEnabled()) {
      blinkRGB(COLOR_SEND, 3, 250);  // blink red
      turnOnRGB(COLOR_SEND, 0);      // red
    } else {
      delay(500);
    }
  }

  if (!successRead) {
    logger::warn(F("Read Rain Sensor Timed-out"));
    serialCode = 9;
  }
}

int measureWindDirection() {
  int measuredVoltage_mV = 0;
  for (int i = 0; i < ADC_WIND_DIRECTION_READINGS; i++) {
    measuredVoltage_mV = analogReadmV(ADC_WIND_DIRECTION_PIN);
    delay(10);
  }
  // Convert millivolts to volts
  float measuredVoltage_V = measuredVoltage_mV / 1000.0;

  float actualVoltage = measuredVoltage_V / ADC_WIND_DIRECTION_VOLTAGE_SCALING_FACTOR;

  int result = convertToWindDirection(actualVoltage);

  logger::debug(F(" - Measured Voltage: %d mV"), (int)measuredVoltage_V);
  logger::debug(F(" - Actual Voltage: %d mV"), actualVoltage);
  logger::debug(F(" - Wind  Direction: %d"), result);

  return result;
}

int convertToWindDirection(float voltage) {
  // measured reading are based on 3.3V
  if (voltage < 0.1) return 1;
  else if (voltage < 0.5) return 2;
  else if (voltage < 1.0) return 3;
  else if (voltage < 1.5) return 4;
  else if (voltage < 2.0) return 5;
  else if (voltage < 2.4) return 6;
  else if (voltage < 2.9) return 7;
  else return 8;
}


int measureWindSpeedVoltage(bool silentMode = false) {
  long sum = 0;
  for (int i = 0; i < ADC_WINDSPEED_READINGS; i++) {
    int voltage = analogReadmV(ADC_WINDSPEED_PIN);
    if (voltage <= 42) {
      voltage = 0;
    }
    sum += voltage;
    delay(10);
  }

  float measuredVoltage_V = (sum / ADC_WINDSPEED_READINGS);
  int actualVoltage = measuredVoltage_V / ADC_WINDSPEED_VOLTAGE_SCALING_FACTOR;

  if (!silentMode) {
    logger::debug(F(" - Measured Voltage: %d mV"), (int)measuredVoltage_V);
    logger::debug(F(" - Actual Voltage: %d mV"), actualVoltage);
  }

  return actualVoltage;
}

void onMeasureInBackground() {
  windSpeedReadingSamples.add(measureWindSpeedVoltage(true));
  TimerStart(&backgroundMeasureTimer);
}

bool meassureAirData() {
  if (!bme280.init()) {
    logger::warn(F(" - Device Error"));
    return false;
  }

  for (int i = 0; i <= BME_READINGS; i++) {
    temperature2 = bme280.getTemperature() * 100;
    pressure = bme280.getPressure();
    humidity2 = bme280.getHumidity() * 100;
    delay(100);
  }

  logger::debug(F(" - Temperature: %d C"), int(temperature2 / 100));
  logger::debug(F(" - Pressure: %d Pa"), int(pressure / 100));
  logger::debug(F(" - Humidity: %d %%"), int(humidity2 / 100));

  return true;
}

static void prepareTxFrame(uint8_t port) {
  logger::debug(F("Up-Time Count: %d"), uptimeCount);

  // stop background measurement timer
  TimerStop(&backgroundMeasureTimer);

  // disable vext for analog measurements
  bool isDisplayEnabled = LoRaWAN.isDisplayEnabled();
  if (digitalRead(Vext) == LOW) {
    if (isDisplayEnabled) {
      LoRaWAN.disableDisplay();
    }
    digitalWrite(Vext, HIGH);
    delay(50);
  }
  detachInterrupt(WAKE_UP_PIN);


  logger::debug(F("Battery: Start to measure"));
  for (int x = 0; x <= 10; x++) {
    batteryVoltage = getBatteryVoltage();
    delay(10);
  }
  logger::debug(F("Battery: %d [mV]"), batteryVoltage);
  logger::debug(F("Battery: Done"));


  logger::debug(F("Windspeed: Start to measure"));
  windSpeedReadingSamples.add(measureWindSpeedVoltage());
  windSpeedVoltageMedian = windSpeedReadingSamples.getMedian();
  windSpeedVoltageMax = windSpeedReadingSamples.getHighest();
  logger::debug(F("Windspeed samples: %d"), windSpeedReadingSamples.getCount());
  logger::debug(F("Median Windspeed: %d [mV] "), windSpeedVoltageMedian);
  logger::debug(F("Max Windspeed: %d [mV] "), windSpeedVoltageMax);
  windSpeedReadingSamples.clear();
  logger::debug(F("Windspeed: Done"));


  // enable vext
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);
  if (digitalRead(Vext) == HIGH) {
    digitalWrite(Vext, LOW);
    delay(50);
    if (isDisplayEnabled) {
      initDisplay();
      displayUpTimeCount();
    }
  }

  Wire1.begin();
  delay(100);


  logger::debug(F("SHT-40: init"));
  logger::debug(F("SHT-40: Start to measure"));
  float temp, hum;
  for (int x = 0; x < SHT_READINGS; x++) {
    if (shtSensor.GetTemperature(&temp) == SHT40AD1B_STATUS_ERROR) {
      logger::err(F(" - Temperature reading failed"));
      temp = 0;
    }
    if (shtSensor.GetHumidity(&hum) == SHT40AD1B_STATUS_ERROR) {
      logger::err(F(" - Humidity reading failed "));
      hum = 0;
    }
    delay(50);
  }
  temperature = temp * 100;
  humidity = hum * 100;
  logger::debug(F(" - Temperature: %d [°C]"), int(temperature / 100.0));
  logger::debug(F(" - Humidity: %d [%%]"), int(humidity / 100.0));
  blinkRGB(0xffff00, 3, 250);  // yellow
  logger::debug(F("SHT-40: Done"));


  logger::debug(F("Ultrasonic Distance: Start to measure"));
  distance = measureDistance();
  if (distance >= 0) {
    blinkRGB(0x0000ff, 3, 250);  // blue
    logger::debug(F("Ultrasonic Distance: Done"));
  }
  logger::debug(F("Distance: %d [cm]"), int(distance / 10.0));


  logger::debug(F("Wind-Direction: Start to measure"));
  windDirection = measureWindDirection();
  logger::debug(F("Wind-Direction:  %d  "), windDirection);
  blinkRGB(0x00ffff, 3, 250);  // cyan
  logger::debug(F("Wind-Direction: Done"));


  logger::debug(F("Rain Sensor: Start to measure"));
  readRainSensor();
  blinkRGB(0xFF1493, 3, 250);  // pink
  logger::debug(F("Rain Sensor: Done"));


  logger::debug(F("Air Sensor: Start to measure"));
  if (meassureAirData()) {
    blinkRGB(0x0000ff, 3, 250);  // blue
  }
  logger::debug(F("Air Sensor: Done"));


  appDataSize = 38;

  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);

  appData[4] = highByte(temperature);
  appData[5] = lowByte(temperature);

  appData[6] = highByte(humidity);
  appData[7] = lowByte(humidity);

  appData[8] = highByte(distance);
  appData[9] = lowByte(distance);

  appData[10] = highByte(windSpeedVoltageMedian);
  appData[11] = lowByte(windSpeedVoltageMedian);

  appData[12] = highByte(windSpeedVoltageMax);
  appData[13] = lowByte(windSpeedVoltageMax);

  appData[14] = highByte(rainEventCounter);
  appData[15] = lowByte(rainEventCounter);
  logger::debug(F("Rain Counter: %d"), rainEventCounter);

  int tmp = acc * 100;
  appData[16] = highByte(tmp);
  appData[17] = lowByte(tmp);
  logger::debug(F("Rain Acc: %d"), tmp);

  tmp = eventAcc * 100;
  appData[18] = highByte(tmp);
  appData[19] = lowByte(tmp);
  logger::debug(F("Rain Event-Acc: %d"), tmp);

  tmp = totalAcc * 100;
  appData[20] = highByte(tmp);
  appData[21] = lowByte(tmp);
  logger::debug(F("Rain Total-Acc: %d"), tmp);

  tmp = rInt * 100;
  appData[22] = highByte(tmp);
  appData[23] = lowByte(tmp);
  logger::debug(F("Rain RInt: %d"), tmp);

  appData[24] = serialCode;
  logger::debug(F("Rain Serial-Code: %d"), serialCode);

  long lastRainAgo = (millis() - lastRainDetectionTime) / 1000;
  appData[25] = lastRainAgo >> 24;
  appData[26] = lastRainAgo >> 16;
  appData[27] = lastRainAgo >> 8;
  appData[28] = lastRainAgo & 0xFF;
  logger::debug(F("Last Rain detection: %d min ago"), (int)(lastRainAgo / 60.0));

  appData[29] = windDirection;

  appData[30] = highByte(temperature2);
  appData[31] = lowByte(temperature2);

  appData[32] = highByte(humidity2);
  appData[33] = lowByte(humidity2);

  tmp = pressure;
  appData[34] = tmp >> 24;
  appData[35] = tmp >> 16;
  appData[36] = tmp >> 8;
  appData[37] = tmp & 0xFF;

  for (size_t i = 0; i < appDataSize; ++i) {
    sprintf(buffer, "0x%02X ", appData[i]);
    Serial.print(buffer);
  }
  Serial.println();

  if (LoRaWAN.isDisplayEnabled()) {
    display.clear();

    // PAGE 1
    sprintf(buffer, "Temperatur: %d [C]", int(temperature / 100.0));
    display.drawString(64, 0, buffer);

    sprintf(buffer, "Distance: %d [cm]", int(distance / 10.0));
    display.drawString(64, 15, buffer);

    sprintf(buffer, "Max Wind-Speed: %d [mV]", windSpeedVoltageMax);
    display.drawString(64, 30, buffer);

    sprintf(buffer, "Wind-Dir: %d", windDirection);
    display.drawString(64, 45, buffer);


    display.display();
    delay(5000);
    display.clear();

    // PAGE 2
    sprintf(buffer, "Rain Acc: %d [mm]", (int)(acc + 0.5));
    display.drawString(64, 0, buffer);

    sprintf(buffer, "Rain Event-Acc: %d [mm]", (int)(eventAcc + 0.5));
    display.drawString(64, 15, buffer);

    sprintf(buffer, "Last-Rain: %d [min]", (int)(lastRainAgo / 60.0));
    display.drawString(64, 30, buffer);

    // sprintf(buffer, "Serial-Code: %d", serialCode);
    // display.drawString(64, 45, buffer);

    sprintf(buffer, "Batt: %d [mV]", batteryVoltage);
    display.drawString(64, 45, buffer);

    display.display();
    delay(5000);
    display.clear();

    // PAGE 3
    // TODO
  }

  Wire1.end();

  digitalWrite(Vext, HIGH);

  uptimeCount++;
}

void prepareBeforeSleep() {
  if (!isTxConfirmed) {
    if (!keepLedStatusEnabled) {
      LoRaWAN.disableRgb();
    }
    LoRaWAN.disableDisplay();
    digitalWrite(Vext, HIGH);
  }

  if (rainEventCounter > 0) {
    // We need to keep the RAIN_SENSOR_TX_PIN set to HIGH so that the rain sensor will evantually reset it's "accEvent" field
    // ~60min after the last rain-drop was detected.
    // Note: As long as the RAIN_SENSOR_TX_PIN is set to HIGH, the rain sensor will never enter it's sleep mode.
    logger::debug(F("Keep Rain-Sensor Serial enabled in low power mode"));
    digitalWrite(RAIN_SENSOR_TX_PIN, HIGH);
  } else {
    // When the RAIN_SENSOR_TX_PIN is et to LOW, the rain sensor should enter it's sleep mode ~20min afterwards.
    logger::debug(F("Turn off Rain-Sensor Serial in low power mode"));
    digitalWrite(RAIN_SENSOR_TX_PIN, LOW);
  }

  TimerStart(&backgroundMeasureTimer);

  logger::set_level(DEFAULT_LOG_LEVEL);
}

void onWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    Serial.println(F("Woke up by WAKE_UP_PIN during sleep"));
    accelWoke = true;
    delay(10);
  }
}

int measureDistance() {
  unsigned long startTime = millis();
  distanceReadingSamples.clear();
  startTime = millis();
  Serial1.setTimeout(TIMEOUT_MS);
  Serial1.flush();
  while (!distanceReadingSamples.isFull()) {
    if ((startTime + DISTANCE_MAX_WAIT_TIME) < millis()) {
      logger::err("Error: Timed out reading distance sensor");
      break;
    }
    int result = readMaxSonarDistance();
    if (result >= 0) {
      distanceReadingSamples.add(result);
    }
  }
  if (!distanceReadingSamples.isFull()) {
    logger::err(F("Error: Could not measure distance"));
    return -1;
  } else {
    return distanceReadingSamples.getMedian();
  }
}

bool isValidNumber(const char* str) {
  if (strlen(str) > DISTANCE_MAX_SERIAL_DATA_LENGTH) {
    Serial.println("Serial Error: Too many digits");
    return false;
  }
  if (*str == '\0') {
    Serial.println("Serial Error: empty string");
    return false;
  }
  while (*str != '\0') {
    if (!isdigit(*str)) {
      Serial.print("Serial Error: non digit char found: ");
      Serial.println(*str);
      return false;
    }
    str++;
  }
  return true;
}

int readMaxSonarDistance() {
  unsigned long startTime = millis();
  size_t bytesRead = Serial1.readBytesUntil('R', buffer, sizeof(buffer) - 1);
  if (bytesRead == 0) {
    return -1;
  }
  buffer[bytesRead] = '\0';

  // trim
  char* end = buffer + strlen(buffer) - 1;
  while (end > buffer && isspace(*end)) {
    *end-- = '\0';
  }

  if (isValidNumber(buffer)) {
    return atoi(buffer);
  } else {
    return -9;
  }
}

void setupRainsensor() {
  rainSensorSerial.begin(9600);
  //rainSensorSerial.setTimeout(5000);
  delay(5000);  // wait for sensor to start up
  //sendRainSensorCommand('k');
  //processRainSerialData();
  rainSensorSerial.setTimeout(2000);
  sendRainSensorCommand('p');
  sendRainSensorCommand('l');
  sendRainSensorCommand('m');
  processRainSerialData();
}

void setup() {
  Serial.begin(115200);
  logger::set_serial(Serial);
  Serial1.begin(9600);

  pinMode(Vext, OUTPUT);

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  pinMode(RAIN_SENSOR_INTERUPT_PIN, INPUT_PULLUP);
  attachInterrupt(RAIN_SENSOR_INTERUPT_PIN, onRainDetected, RISING);

#if (AT_SUPPORT)
  enableAt();
#endif

  TimerInit(&backgroundMeasureTimer, onMeasureInBackground);
  TimerSetValue(&backgroundMeasureTimer, backgroundMeasureInterval);

  initManualRun();

  displaySplash();

  setupRainsensor();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        //LoRaWAN.displayJoining();
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        displayUpTimeCount();
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        prepareBeforeSleep();
        logger::info("Go to sleep for: %d sec", (int)(txDutyCycleTime / 1000.0));
        delay(100);

        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          initManualRun();
          logger::debug(F("Start sending cycle due to wakeup"));
          LoRaWAN.txNextPacket();
          accelWoke = false;
        } else {
          //LoRaWAN.displayAck();
          if (isTxConfirmed && LoRaWAN.hasReceivedAck()) {
            if (!keepLedStatusEnabled) {
              LoRaWAN.disableRgb();
            }
            LoRaWAN.disableDisplay();
            digitalWrite(Vext, HIGH);
            LoRaWAN.resetReceivedAck();
          }
          LoRaWAN.sleep();
        }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}

void setKeepStatusLightEnabled(bool keepEnabled) {
  keepLedStatusEnabled = keepEnabled;
  logger::info(F("keepLedStatusEnabled: %s "), keepLedStatusEnabled ? "true" : "false");
  if (keepLedStatusEnabled) {
    LoRaWAN.enableRgb();
  } else {
    LoRaWAN.disableRgb();
  }
}

// AT Command in the form of:
// AT<the command>=<value>
// e.g: AT+KeepLedStatusEnabled=1
//AT Command                Value
//+LORAWAN=1                LoRaWAN  1, LoRa 0
//+OTAA=1                   OTAA -1, ABP-0
//+Class=A                  Class A or C
//+ADR=1                    1 on 0 for off
//+IsTxConfirmed=1          LoRaWAN ACK Message 1 on, 0 off.
//+AppPort=2                The Application Port 2 for general APPs and 10 for TTN MAPPER.
//+DutyCycle=60000          The time between transmission in mS. Typically, 15000 to 3600000
//+ConfirmedNbTrials=8      The number of adaptive rate changes allowed.
//+DevEui=???               Unique (OTAA Mode)
//+AppEui=???               Unique (OTAA Mode)
//+AppKey=???               Unique (OTAA Mode)
//+NwkSKey=???              Unique (ABP Mode)
//+Passkey=???              Unique (ABP Mode)
//+DevAddr=???              Unique (ABP Mode)
//+LPM=1                    Low Power Mode
//+ChipID=?                 get ChipID
//+JOIN=1                   start join
//+DelCDKEY=1               to delete the CDKEY
//+DefaultSet=1             to reset parameter to Default setting
//+KeepLedStatusEnabled=1   Keep Status Lights enabled
bool checkUserAt(char* cmd, char* content) {
  if (strcmp(cmd, "KeepLedStatusEnabled") == 0) {
    setKeepStatusLightEnabled(strcmp(content, "1") == 0 || strcmp(content, "true") == 0);
    return true;
  }
  return false;
}

void downLinkDataHandle(McpsIndication_t* mcpsIndication) {
  Serial.printf("+REV DATA: %s, RXSIZE: %d, PORT: %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA: ");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (mcpsIndication->Port == 4 && mcpsIndication->BufferSize > 0) {
    // handle DutyCycle
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    Serial.printf("new newSleepTime received: %d sec\r\n", newSleepTime);
    appTxDutyCycle = newSleepTime * 1000;
    saveDr();
    Serial.print(F("new DutyCycle received: "));
    Serial.print(appTxDutyCycle);
    Serial.println(F("ms"));
  } else if (mcpsIndication->Port == 5 && mcpsIndication->BufferSize > 0) {
    // handle keep status light
    bool keepStatusLightEnabled = mcpsIndication->Buffer[0] != 0;
    setKeepStatusLightEnabled(keepStatusLightEnabled);
  } else if (mcpsIndication->Port == 6 && mcpsIndication->BufferSize > 0) {
    // handle scheduledRainCommand
    scheduledRainCommand = char(mcpsIndication->Buffer[0]);
    Serial.print(F("scheduled rain command: "));
    Serial.println(scheduledRainCommand);
  }
  if (mcpsIndication->Port == 7 && mcpsIndication->BufferSize > 0) {
    // handle backgroundMeasureInterval
    int newIntervalTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    Serial.printf("new interval time received: %d sec\r\n", newIntervalTime);
    backgroundMeasureInterval = newIntervalTime * 1000;
    Serial.print(F("new backgroundMeasureInterval received: "));
    Serial.print(backgroundMeasureInterval);
    Serial.println(F("ms"));
  } else if (mcpsIndication->Port == 9) {
    // handle restart
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
  delay(50);
}
