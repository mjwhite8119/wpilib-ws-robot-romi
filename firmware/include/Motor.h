#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>

class Motor
{  
   public:

    // Class variables

    // Constructor to connect Motor GPIO pins to microcontroller
    Motor(uint8_t in1Port, uint8_t in2Port, Encoder encoder)
      :in1Port_(in1Port), in2Port_(in2Port), encoder_(encoder)
    {}  

    void init() {
      Serial.print("Motor initiated on "); printPort(); Serial.println("");
      Serial.print("Encoder "); encoder_.printPort(); Serial.println("");
    }

    /*                IN3/IN1           IN4/IN2
      Forward          HIGH              LOW
      Reverse          LOW               HIGH
      Stop             LOW               LOW
      Stop             HIGH              HIGH 
    */ 
    void applyPower(int16_t speed){
      DBSpeed = applyDeadband(speed, 20);
      if (DBSpeed == 0) {
        digitalWrite(in1Port_, LOW);
        digitalWrite(in2Port_, LOW);
      }
      else if( DBSpeed > 0) {
        // if (encoder_.getRotations() > 1) {
        //   Serial.print("Finger flexed ");encoder_.printInfo();
        // } else {
          digitalWrite(in1Port_, HIGH);
          digitalWrite(in2Port_, LOW);
          printPort(); printSpeed();
          Serial.print("Finger flexed ");encoder_.printInfo();
        // }
        
      }
      else {
        // if (encoder_.getRotations() == 0) {
        //   Serial.print("Finger extended "); encoder_.printInfo();
        // } else {
          digitalWrite(in1Port_, LOW);
          digitalWrite(in2Port_, HIGH);
          printPort(); printSpeed();
          Serial.print("Finger extended "); encoder_.printInfo();
        // }  
      }
    }

    void applyPWMPower(int16_t speed) {
      DBSpeed = applyDeadband(speed, 20);
      if (DBSpeed == 0) {
        digitalWrite(in1Port_, LOW);
        digitalWrite(in2Port_, LOW);
      } 
      else if (DBSpeed > 0) {
        analogWrite(in1Port_, DBSpeed);
        digitalWrite(in2Port_, LOW);
        // printPort(); printSpeed();
        // Serial.print("Finger flexed ");encoder_.printInfo();
      }
      else {
        digitalWrite(in1Port_, LOW);
        analogWrite(in2Port_, DBSpeed);
        // printPort(); printSpeed();
        // Serial.print("Finger extended "); encoder_.printInfo();
      }
    }

  private:

    uint8_t in1Port_;
    uint8_t in2Port_;
    int DBSpeed = 0;
    Encoder encoder_;

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
      Serial.print("Speed "); Serial.println(DBSpeed);
    }

};

#endif // _MOTOR_H_