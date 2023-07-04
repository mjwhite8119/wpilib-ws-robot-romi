#include "Motor.h"

// -------------------Constructors -----------------------------------

// Constructor to connect Motor GPIO pins to microcontroller
Motor::Motor(uint8_t encoderPort, uint8_t in1Port, uint8_t in2Port, uint8_t mode)
  :encoder(encoderPort), in1Port_(in1Port), in2Port_(in2Port), mode_(mode) 
  {
    // Make sure motors are off
    digitalWrite(in1Port, LOW);
    digitalWrite(in2Port, LOW);

    if (mode == 1) {
      // create a PWM channels
      ledcSetup(channel_0, freq, resolution); 
      ledcSetup(channel_1, freq, resolution); 

      // attach channels to pins
      ledcAttachPin(in1Port, channel_0); 
      ledcAttachPin(in2Port, channel_1);
      
    } else {
      pinMode(in1Port,OUTPUT);
      pinMode(in2Port,OUTPUT); 
    }
    
  }  


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
  int16_t continuous_position = encoder.readEncoder();
  
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

  DBSpeed_ = applyDeadband(speed, 20);
  
  if (DBSpeed_ == 0) {
    // // Make both pins digital
    // ledcDetachPin(in2Port_);
    // ledcDetachPin(in1Port_);

    // pinMode(in1Port_,OUTPUT); 
    // pinMode(in2Port_,OUTPUT); 
    // digitalWrite(in1Port_, LOW);
    // digitalWrite(in2Port_, LOW);
    ledcWrite(channel_0, DBSpeed_);
    ledcWrite(channel_1, DBSpeed_);
  } 
  else if (DBSpeed_ > 0) {
    // Setup the pins
    // ledcDetachPin(in2Port_);

    // pinMode(in2Port_,OUTPUT); 
    // digitalWrite(in2Port_, LOW);

    // ledcAttachPin(in1Port_, channel_0); 
    ledcWrite(channel_0, abs(DBSpeed_));
    ledcWrite(channel_1, 0);  // Write a LOW

    printPort(); printSpeed();
    Serial.print("Finger flexed ");encoder.printInfo();
  }
  else {
    // Setup the pins
    // ledcDetachPin(in1Port_);
    
    // pinMode(in1Port_,OUTPUT); 
    // digitalWrite(in1Port_, LOW);

    // ledcAttachPin(in2Port_, channel_1);
    ledcWrite(channel_1, abs(DBSpeed_));
    ledcWrite(channel_0, 0);  // Write a LOW

    printPort(); printSpeed();
    Serial.print("Finger extended "); encoder.printInfo();
  }
}