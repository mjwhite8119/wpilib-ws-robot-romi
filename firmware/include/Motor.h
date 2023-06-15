#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>

class Motor
{  
   public:

    // Class variables

    // Constructor to connect Motor GPIO pins to microcontroller
    Motor(uint8_t in1Port, uint8_t in2Port)
      :in1Port_(in2Port), in2Port_(in2Port)
    {}  

    void applyPower(double speed){
      int db_speed = applyDeadband(speed, 20);
      if (db_speed == 0) {
        digitalWrite(in1Port_, LOW);
        digitalWrite(in2Port_, LOW);
      }
      else if( db_speed > 0) {
        digitalWrite(in1Port_, HIGH);
        digitalWrite(in2Port_, LOW);
      }
      else {
        digitalWrite(in1Port_, LOW);
        digitalWrite(in2Port_, HIGH);
      }
    }

  private:

    uint8_t in1Port_;
    uint8_t in2Port_;

    double applyDeadband(double input, double threshold) {
    if (input < -threshold || input > threshold) {
      return input;
    }
    return 0.0;
}

};

#endif // _MOTOR_H_