#include "ASR_Arduino.h"

static TimerEvent_t watchDogTimer;

unsigned int maxFeedTime, watchDogCheckInterval;

static volatile unsigned long lastFeedTime = millis();

static void feedWatchDog() {
  lastFeedTime = millis();
}

static void checkWatchDog() {
  unsigned long currentTime = millis();
  if ((currentTime - lastFeedTime) >= maxFeedTime) {
    Serial.println(F("Watch Dog is Hungry! Reset Hardware"));
    delay(10);
    HW_Reset(0);
    return;
  }
  TimerStart(&watchDogTimer);
}

static void stopWatchDog() {
  Serial.println(F("Watch Dog stopped"));
  TimerStop(&watchDogTimer);
}

static void startWatchDog() {
  Serial.println(F("Watch Dog started"));
  TimerStart(&watchDogTimer);
  feedWatchDog();
}

static void initWatchDog(unsigned int interval, unsigned int maxTime = 0) {
  maxFeedTime = maxTime;
  watchDogCheckInterval = interval;
  TimerInit(&watchDogTimer, checkWatchDog);
  TimerSetValue(&watchDogTimer, watchDogCheckInterval);
  Serial.printf("Setup Watchdog: Interval %d sec | Max feeding time: %d sec \r\n", int(watchDogCheckInterval / 1000), int(maxFeedTime / 1000));
}

