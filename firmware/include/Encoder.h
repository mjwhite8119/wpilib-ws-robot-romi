#ifndef _ENCODER_H_
#define _ENCODER_H_

#define FORWARD 1
#define REVERSE 0

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
      if (position < last_position && motorDirection(rPiLink.buffer.pinkMotor) == FORWARD) {
        rotations += 1;
      } else if (position > last_position && motorDirection(rPiLink.buffer.pinkMotor) == REVERSE) {
        rotations -= 1;
      }
      last_position = position;
      return position;
    }

    int16_t getRotations() {return rotations;}

    void resetEncoder() {rotations = 0;}

  private:

    uint8_t port_;
    int16_t position = 0;
    int16_t last_position = 0;
    int16_t rotations = 0;

};

#endif // _ENCODER_H_