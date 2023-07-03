#include "Encoder.h"

// Constructor to connect encoder GPIO pins to microcontroller
Encoder::Encoder(uint8_t port)
  :port_(port)
{}  

void Encoder::init() {
  position = map(analogRead(port_), 0, 1023, 0, 100);
  last_position = position;
  printPort(); Serial.print("Initialized. "); printPosition(); printRotations();
}

int16_t Encoder::readEncoder() {
  raw = analogRead(port_);
  position = map(raw, 0, 4095, 0, 100);
  
  if (position != 100) {
    if (transitioning & (direction == FORWARD) & (position != 0)) {
      return getRotations() - offset;
    }
    else if (transitioning & (direction == REVERSE) & (position != 100)) {
      return getRotations() - offset;
    }  

    transitioning = false;
    return getRotations() + getPosition();
  }

  // Stopped at 100 so return current value since we have no direction
  if (direction == STOPPED) {
    return getRotations() - offset;
  }

  // We've taken care of the rotations already so return
  if (transitioning == true) {
    return getRotations() - offset;
  }
  
  // Transitioning so take care of business
  transitioning = true;

  if (direction == FORWARD) {
    rotations += 1;
  } else {
    rotations -= 1;  
  }

  // Only return position so as not to double count
  return getPosition();
}

void Encoder::resetEncoder() {
  rotations = 0;
  offset = position;
  Serial.print("Reset "); printInfo();
}