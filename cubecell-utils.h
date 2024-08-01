void blinkRGB(uint32_t color, int times = 3, int blinkTime = 500) {
  if (!LoRaWAN.isRgbEnabled()) {
    return;
  }
  for (int i = 0; i < times; i++) {
    turnOnRGB(color, blinkTime);
    turnOnRGB(0, blinkTime);
  }
}

void turnVextOn() {
  if (digitalRead(Vext) == HIGH) {
    digitalWrite(Vext, LOW);
    delay(50);
  }
}

void turnVextOff() {
  if (digitalRead(Vext) == LOW) {
    digitalWrite(Vext, HIGH);
    delay(50);
  }
}

bool isValidNumber(const char* str, int maxSize) {
  if (strlen(str) > maxSize) {
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
