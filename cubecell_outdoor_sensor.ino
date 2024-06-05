#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "softSerial.h"
#include "RunningMedian.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <simple_logger.h>
#include "credentials.h"

#define TIMEOUT_MS 2000

#define WAKE_UP_PIN USER_KEY

#define DEFAULT_LOG_LEVEL logger::Debug  // DEBUG: set to Debug for more logging statements or to None for no logs$

#define DALLAS_READINGS 3

#define DISTANCE_READINGS 8
#define MAX_READING_WAIT_TIME 5000

#define MAX_SONAR_SERIAL_DATA_LENGTH 4

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 1000 * 60 * 15;

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


/**
   Dallas Temperature Sensor
*/
int oneWirePin = GPIO5;
OneWire oneWire(oneWirePin);
DallasTemperature dallasTemperatures(&oneWire);


/**
   Ultrasonic Distance Sensor
*/
softSerial softwareSerial(GPIO3 /*TX pin*/, GPIO2 /*RX pin*/);
RunningMedian samples = RunningMedian(DISTANCE_READINGS);


/**
   ADS 1115 16-bit
*/
Adafruit_ADS1115 ads;

static int32_t temperature;

static unsigned int batteryVoltage, uptimeCount, distance, windSpeedVoltage;

bool accelWoke = false;

boolean recalibrateDuty = false;

char buffer[16];

void blinkRGB(uint32_t color, int times = 3, int blinkTime = 500) {
  turnOnRGB(COLOR_SEND, 1000);
  for (int i = 0; i < times; i++) {
    turnOnRGB(color, blinkTime);
    turnOnRGB(0, blinkTime);
  }
}

int measureWindSpeedVoltage() {
  ads.begin();
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  int16_t adc3;
  float multiplier = 0.125F;
  int adc3Voltage = 0;
  for (size_t i = 0; i < 4; ++i) {
    adc3 = ads.readADC_SingleEnded(3);
    adc3Voltage = adc3 * 0.000125 * (4.0 / 3.3) * 1000;
    delay(10);
  }

  return adc3Voltage;
}

static void prepareTxFrame(uint8_t port) {
  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  if (digitalRead(Vext) == HIGH) {
    digitalWrite(Vext, LOW);
    delay(500);
  }

  logger::debug(F("Up-Time Count: %d"), uptimeCount);


  logger::debug("Dallas Temperature: init");
  dallasTemperatures.begin();
  delay(50);
  if (dallasTemperatures.getDeviceCount() > 0) {
    logger::debug("Dallas Temperature: Start to measure");
    for (int x = 1; x <= DALLAS_READINGS; x++) {
      dallasTemperatures.requestTemperatures();
      temperature = dallasTemperatures.getTempCByIndex(0) * 100;
    }
    logger::debug("Dallas Temperature: Done");
  }
  logger::debug("Temperature: %d [°C]", int(temperature / 100.0));


  logger::debug("Ultrasonic Distance: Start to measure");
  distance = measureDistance();
  logger::debug("Distance:    %d [cm]", int(distance / 10.0));


  logger::debug("ADS: Start to measure");
  windSpeedVoltage = measureWindSpeedVoltage();
  logger::debug("Wind Speed:  %d [mV] ", windSpeedVoltage);
  logger::debug("ADS: Done");
  Wire.end();

  digitalWrite(Vext, HIGH);
  delay(100);

  logger::debug(F("Battery:   %d [mV]"), batteryVoltage);

  appDataSize = 10;

  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);

  appData[4] = highByte(temperature);
  appData[5] = lowByte(temperature);

  appData[6] = highByte(distance);
  appData[7] = lowByte(distance);

  appData[8] = highByte(windSpeedVoltage);
  appData[9] = lowByte(windSpeedVoltage);

  for (size_t i = 0; i < appDataSize; ++i) {
    sprintf(buffer, "0x%02X ", appData[i]);
    Serial.print(buffer);
  }
  Serial.println();

  uptimeCount++;
}

void prepareBeforeSleep() {
  if (!isTxConfirmed) {
    LoRaWAN.disableRgb();
    digitalWrite(Vext, HIGH);
  }
  logger::set_level(DEFAULT_LOG_LEVEL);
  delay(20);
}

void onWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    Serial.println(F("Woke up by WAKE_UP_PIN during sleep"));
    accelWoke = true;
    delay(10);
  }
}

void initManualRun() {
  logger::set_level(logger::Debug);
  //LoRaWAN.enableDisplay();
  LoRaWAN.enableRgb();

  if (LoRaWAN.isRgbEnabled()) {
    Serial.println(F("LoRaWANp"));
  }

  turnOnRGB(0x005050, 500);
  turnOnRGB(0x002450, 500);
  turnOnRGB(0x000050, 500);
  turnOffRGB();
}

int measureDistance() {
  logger::debug("Distance Measuring started");
  unsigned long startTime = millis();
  samples.clear();
  startTime = millis();
  softwareSerial.setTimeout(TIMEOUT_MS);
  softwareSerial.flush();
  while (!samples.isFull()) {
    if ((startTime + MAX_READING_WAIT_TIME) < millis()) {
      logger::err("Error: Timed out reading distance sensor");
      break;
    }
    int result = readMaxSonarDistance();
    if (result >= 0) {
      samples.add(result);
    }
  }
  if (!samples.isFull()) {
    logger::err("Error: Could not measure distance");
    return 0;
  } else {
    logger::debug("Distance measuring finished!");
    return samples.getMedian();
  }
}

bool isValidNumber(const char* str) {
  if (strlen(str) > MAX_SONAR_SERIAL_DATA_LENGTH) {
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
  size_t bytesRead = softwareSerial.readBytesUntil('R', buffer, sizeof(buffer) - 1);
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

void setup() {
  Serial.begin(115200);
  logger::set_serial(Serial);
  softwareSerial.begin(9600);

  pinMode(Vext, OUTPUT);

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

#if (AT_SUPPORT)
  enableAt();
#endif

  initManualRun();

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
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
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
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          initManualRun();
          logger::debug(F("Start Sending Cylcle due to wakeup"));
          LoRaWAN.txNextPacket();
          accelWoke = false;
        } else {
          if (isTxConfirmed && LoRaWAN.hasReceivedAck()) {
            LoRaWAN.disableRgb();
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
void downLinkDataHandle(McpsIndication_t* mcpsIndication) {
  Serial.printf("+REV DATA: %s, RXSIZE: %d, PORT: %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA: ");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (mcpsIndication->Port == 4) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    appTxDutyCycle = newSleepTime * 1000;
    saveDr();
    Serial.print(F("new DutyCycle received: "));
    Serial.print(appTxDutyCycle);
    Serial.println(F("ms"));
  } else if (mcpsIndication->Port == 5) {
    recalibrateDuty = true;
    Serial.println(F("recalibrate the next time "));
  } else if (mcpsIndication->Port == 9) {
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
}
