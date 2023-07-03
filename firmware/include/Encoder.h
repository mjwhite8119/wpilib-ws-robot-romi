#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <Arduino.h>

#define STOPPED 2
#define FORWARD 1
#define REVERSE 0

class Encoder
{  
   public:

    Encoder() {} // Default constructor  

    uint8_t direction = STOPPED;
    int16_t rotations = 0;
    int16_t position = 0;
    int16_t raw = 0;
    int16_t offset = 0;
    boolean transitioning = false;

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t port);

    void init();

    int16_t readEncoder();

    int16_t getRotations() {return rotations * 100;}
    int16_t getPosition() {return position - offset;}

    void resetEncoder();

    void printPort() {
      Serial.print("Port "); Serial.print(port_);Serial.print(": ");
    }

    void printPosition() {
      Serial.print(" Position "); Serial.println(position);
    }

    void printRotations() {
      Serial.print(" Rotations "); Serial.println(rotations);
    }

    void printInfo() {
      printPort(); Serial.print(rotations); Serial.print(":"); Serial.print(position);
      Serial.print(" Raw "); Serial.print(raw);Serial.print(" Offset "); Serial.println(offset);
    }

  private:

    uint8_t port_;
    int16_t last_position = 0;

    int16_t applyDeadband(int16_t input, const int16_t threshold) {
      if (input < -threshold || input > threshold) {
        return input;
      }
      return 0;
    }

};

#endif // _ENCODER_H_