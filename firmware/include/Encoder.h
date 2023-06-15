#ifndef _ENCODER_H_
#define _ENCODER_H_

class Encoder
{  
   public:

    // Class variables

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t port)
      :port_(port)
    {}  

    int16_t getPosition() {
      position = map(analogRead(port_), 0, 1023, 0, 100);
      // Only apply a rotation if the difference is greater than 50 percent
      int16_t diff = applyDeadband(position - last_position, 50);
      if (diff == 0) {
        last_position = position;
        return position;
      }
      
      if (diff > 0) {
        rotations += 1;
      } else {
        rotations -= 1;
      }
      last_position = position;

      Serial.print("Pink rotations "); Serial.println(rotations);
  
      return position;
    }

    int16_t getRotations() {return rotations;}

    void resetEncoder() {rotations = 0;}

  private:

    uint8_t port_;
    int16_t position = 0;
    int16_t last_position = 0;
    int16_t rotations = 0;

    int16_t applyDeadband(int16_t input, const int16_t threshold) {
      if (input < -threshold || input > threshold) {
        return input;
      }
      return 0;
    }

};

#endif // _ENCODER_H_