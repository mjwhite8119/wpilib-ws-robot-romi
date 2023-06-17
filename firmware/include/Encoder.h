#ifndef _ENCODER_H_
#define _ENCODER_H_

class Encoder
{  
   public:

    Encoder() {} // Default constructor  

    int16_t rotations = 0;
    int16_t position = 0;

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t port)
      :port_(port)
    {}  

    void init() {
      position = map(analogRead(port_), 0, 1023, 0, 100);
      last_position = position;
      printPort(); Serial.print("Initialized. "); printPosition(); printRotations();
    }

    int16_t readEncoder() {
      position = map(analogRead(port_), 0, 1023, 0, 100);
      // Serial.print("readEncoder ");printInfo();
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

      Serial.print("readEncoder ");printInfo();
  
      return position;
    }

    int16_t getRotations() {return rotations;}
    int16_t getPosition() {return position;}

    void resetEncoder() {
      rotations = 0;
      printPort(); Serial.print("Reset "); printRotations();
    }

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
      printPort(); Serial.print(rotations); Serial.print(":"); Serial.println(position);
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