#ifndef _MOTOR_H_
#define _MOTOR_H_

#ifndef Arduino
  #include <Arduino.h>
#endif

#ifndef Encoder
  #include "Encoder.h"
#endif

#define STOPPED 2
#define FORWARD 1
#define REVERSE 0

class Motor
{  
  public:

    Motor() {} // Default constructor

    // Constructor to connect Motor GPIO pins to microcontroller
    Motor(uint8_t in1Port, uint8_t in2Port, uint8_t encoderPort);

    // Encoder attached to the motor
    Encoder encoder;

    void init();

    void applyPower(int16_t speed);

    void applyPWMPower(int16_t speed);

  private:
    // Motor ports
    uint8_t in1Port_;
    uint8_t in2Port_;

     int DBSpeed_ = 0;

    double applyDeadband(double input, double threshold) {
      if (input < -threshold || input > threshold) {
        return input;
      }
      return 0.0;
    }

    void printPort() {
      Serial.print("Ports "); Serial.print(in1Port_); Serial.print(","); Serial.print(in2Port_);Serial.print(": ");
    }

    void printSpeed() {
      Serial.print("Speed "); Serial.println(DBSpeed_);
    }

};

#endif // _MOTOR_H_