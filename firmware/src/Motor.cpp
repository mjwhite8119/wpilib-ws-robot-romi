#include "Motor.h"

// -------------------Constructors -----------------------------------

// Constructor to connect Motor GPIO pins to microcontroller
Motor::Motor(uint8_t encoderPort, uint8_t in1Port, uint8_t in2Port)
  :encoder(encoderPort), in1Port_(in1Port), in2Port_(in2Port) {}  


void Motor::init() {
  Serial.print("Motor initiated on "); printPort(); Serial.println("");
  encoder.resetEncoder();
  Serial.print("Encoder "); encoder.printPort(); Serial.println("");
}

/*                IN3/IN1           IN4/IN2
  Forward          HIGH              LOW
  Reverse          LOW               HIGH
  Stop             LOW               LOW
  Stop             HIGH              HIGH 
*/ 
void Motor::applyPower(int16_t speed){
  uint16_t continuous_position = encoder.readEncoder();
  
  DBSpeed_ = applyDeadband(speed, 20);
  if (DBSpeed_ > 400) {DBSpeed_ = 0;} // Take care of random values 
  
  if (DBSpeed_ == 0) {
    digitalWrite(in1Port_, LOW);
    digitalWrite(in2Port_, LOW);
    encoder.direction = STOPPED;
    // Serial.print("Flexing SPEED 0 ");encoder.printInfo();
  }
  else if( DBSpeed_ > 0) {
    digitalWrite(in1Port_, HIGH);
    digitalWrite(in2Port_, LOW);
    encoder.direction = FORWARD;
    // printPort(); printSpeed();
    Serial.print("Flexing ");encoder.printInfo();
    Serial.println(continuous_position);
  }
  else {
    digitalWrite(in1Port_, LOW);
    digitalWrite(in2Port_, HIGH);
    encoder.direction = REVERSE;
    printPort(); printSpeed();
    Serial.print("Extending "); encoder.printInfo();
    Serial.println(continuous_position);
  }
}

void Motor::applyPWMPower(int16_t speed) {
//   DBSpeed = applyDeadband(speed, 20);
//   if (DBSpeed == 0) {
//     digitalWrite(in1Port_, LOW);
//     digitalWrite(in2Port_, LOW);
//   } 
//   else if (DBSpeed > 0) {
//     analogWrite(in1Port_, DBSpeed);
//     digitalWrite(in2Port_, LOW);
//     // printPort(); printSpeed();
//     // Serial.print("Finger flexed ");encoder_.printInfo();
//   }
//   else {
//     digitalWrite(in1Port_, LOW);
//     analogWrite(in2Port_, DBSpeed);
//     // printPort(); printSpeed();
//     // Serial.print("Finger extended "); encoder_.printInfo();
//   }
}