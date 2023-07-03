#ifndef _ENCODER_H_
#define _ENCODER_H_

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
    boolean transitioning = false;

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t port)
      :port_(port)
    {}  

    void init() {
      position = map(analogRead(port_), 0, 1023, 0, 100);
      last_position = position;
      printPort(); Serial.print("Initialized. "); printPosition(); printRotations();
    }

    uint16_t readEncoder() {
      raw = analogRead(port_);
      position = map(raw, 0, 4095, 0, 100);
      
      if (position != 100) {
        if (transitioning & (direction == FORWARD) & (position != 0)) {
          return rotations * 100;
        }
        transitioning = false;
        return (rotations * 100) + position;
      }

      // Stopped at 100 so return current value since we have no direction
      if (direction == STOPPED) {
        return rotations * 100;
      }

      // We've taken care of the rotations already so return
      if (transitioning == true) {
        return rotations * 100;
      }
      
      // Transitioning so take care of business
      transitioning = true;

      if (direction == FORWARD) {
        rotations += 1;
      } else {
        rotations -= 1;  
      }

      // Only return position so as not to double count
      return position;
    }

    int16_t getRotations() {return rotations;}
    int16_t getPosition() {return position;}

    void resetEncoder() {
      rotations = 0;
      Serial.print("Reset "); printInfo();
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
      printPort(); Serial.print(rotations); Serial.print(":"); Serial.print(position);Serial.print(" Raw "); Serial.println(raw);
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