#include <Arduino.h>
#include <PololuRPiSlave.h>

#include "shmem_buffer.h"
#include "Encoder.h"
#include "Motor.h"

// Addresses for Arduinos
#define ARDUINO_1_ADDRESS 20 // I2C Address of Arduino 1
#define ARDUINO_2_ADDRESS 21 // I2C Address of Arduino 1

// Buffer and delay time
PololuRPiSlave<Data, 20> rPiLink;

// --- Define ENCODERS ---

#define PINK_ENCODER A0
#define RING_ENCODER A1
#define MIDDLE_ENCODER A2
#define INDEX_ENCODER A3

Encoder pinkEncoder = Encoder(PINK_ENCODER);
Encoder ringEncoder = Encoder(RING_ENCODER);

// Nano PWM pins 3,5,6,9,10 and 11. 
// The PINK finger does not use PWM because the Nano 
// only has 6 PWM pins.
#define PINK_IN1 8
#define PINK_IN2 7 
#define RING_IN3 3 
#define RING_IN4 5 
#define MIDDLE_IN1 11 
#define MIDDLE_IN2 10 
#define INDEX_IN3 9 
#define INDEX_IN4 6 

Motor pinkMotor = Motor(PINK_IN1, PINK_IN2, pinkEncoder);
Motor ringMotor = Motor(RING_IN3, RING_IN4, ringEncoder);

// int pink_encoder = 0;
// int ring_encoder = 0;
// int middle_encoder = 0;
// int index_encoder = 0;
// int pink_encoder_rotations = 0;
// int ring_encoder_rotations = 0;
// int middle_encoder_rotations = 0;
// int index_encoder_rotations = 0;

// int pink_encoder_last_value = 0;
// int ring_encoder_last_value = 0;

#define BUTTON_PIN 2
#define STOPPED 2
#define FORWARD 1
#define REVERSE 0

int loop_count = 0;
int print_count = 0;
int button_state = LOW;

void logOutput(double pinkSpeed, double ringSpeed, double middleSpeed, double indexSpeed) {
  if (loop_count > 2000) {
    // Serial.print(print_count); Serial.print(" Firmware "); Serial.println(rPiLink.buffer.firmwareIdent);
    // Serial.print("  pink speed ");Serial.println(pinkSpeed);
    // Serial.print("  pink DEADBAND speed ");Serial.println(ringSpeed);
    // Serial.print("  ring ");Serial.println(ringSpeed);
    // Serial.print("middle ");Serial.println(middleSpeed);
    // Serial.print(" index ");Serial.println(indexSpeed);
    // Serial.println(rPiLink.buffer.firmwareIdent);
    // Serial.println(rPiLink.buffer.status);
    loop_count = 0;
    print_count += 1;
  }
  loop_count += 1;
}

double applyDeadband(double input, double threshold) {
  if (input < -threshold || input > threshold) {
    return input;
  }
  return 0.0;
}

// --------------------------------------------------
/*                IN3/IN1           IN4/IN2
   Forward          HIGH              LOW
   Reverse          LOW               HIGH
   Stop             LOW               LOW
   Stop             HIGH              HIGH 
*/ 
// void applyPower(double speed, int in1, int in2){
//   int db_speed = applyDeadband(speed, 20);
//   if (db_speed == 0) {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//   }
//   else if( db_speed > 0) {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//   }
//   else {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//   }
// }

void startMotors(double pinkSpeed, double ringSpeed, double middleSpeed, double indexSpeed) {

  pinkMotor.applyPower(rPiLink.buffer.pinkMotor);
  ringMotor.applyPower(rPiLink.buffer.ringMotor);
  

  // int speed = 0;
  // logOutput(pinkSpeed, ringSpeed, middleSpeed, indexSpeed);

  // applyPower(pinkSpeed, PINK_IN1, PINK_IN2);
  // applyPower(ringSpeed, RING_IN3, RING_IN4);
  // applyPower(middleSpeed, MIDDLE_IN1, MIDDLE_IN2);
  // applyPower(indexSpeed, INDEX_IN3, INDEX_IN4);
}

int pot_loop_count = 0;
void logEncoderOutput(double pink, double ring, double middle, double index) {
  if (pot_loop_count > 100) {
    Serial.print("Pink encoder "); Serial.println(pink);
    Serial.print("Last value "); Serial.println(middle);
    Serial.print("Pink rotations "); Serial.println(ring);
    // Serial.print("Middle encoder "); Serial.println(middle);
    // Serial.print("Index encoder "); Serial.println(index);
    pot_loop_count = 0;
  }
  pot_loop_count += 1;
}

int motorDirection(double motor) {
  int dir = applyDeadband(motor, 20);
  if (dir == 0) {return FORWARD;}
  if (dir > 0) {return FORWARD;}
  return REVERSE;
}

void resetEncoders() {
  // pink_encoder_rotations = 0;
  // ring_encoder_rotations = 0;
  // middle_encoder_rotations = 0;
  // index_encoder_rotations = 0;

  // pink_encoder_last_value = 0;
  // ring_encoder_last_value = 0;
}

// int readPinkEncoder()
// {
//   pink_encoder = map(analogRead(PINK_ENCODER), 0, 1023, 0, 100);
//   int diff = applyDeadband(pink_encoder - pink_encoder_last_value, 50);
//   if (diff == 0) {
//     pink_encoder_last_value = pink_encoder;
//     return pink_encoder;
//   }
  
//   if (diff > 0) {
//     pink_encoder_rotations += 1;
//   } else {
//     pink_encoder_rotations -= 1;
//   }

//   Serial.print("Pink rotations "); Serial.println(pink_encoder_rotations);
  
//   // logEncoderOutput(pink_encoder, pink_encoder_rotations, pink_encoder_last_value, ring_encoder_rotations);
//   pink_encoder_last_value = pink_encoder;

//   return pink_encoder;

//   // ring_encoder = map(analogRead(RING_ENCODER), 0, 1023, 0, 100);
//   // if (ring_encoder < ring_encoder_last_value && motorDirection(rPiLink.buffer.ringMotor) == FORWARD) {
//   //   ring_encoder_rotations += 1;
//   // } else if (ring_encoder > ring_encoder_last_value && motorDirection(rPiLink.buffer.ringMotor) == REVERSE) {
//   //   ring_encoder_rotations -= 1;
//   // }
//   // ring_encoder_last_value = ring_encoder;

//   // middle_encoder = map(analogRead(MIDDLE_ENCODER), 0, 1023, 0, 100);
//   // index_encoder = map(analogRead(INDEX_ENCODER), 0, 1023, 0, 100);

  
// }

void setupMotors() {
  // Setup motor pins
  pinMode(PINK_IN1,OUTPUT);
  pinMode(PINK_IN2,OUTPUT); 
  pinMode(RING_IN3,OUTPUT);
  pinMode(RING_IN4,OUTPUT); 
  pinMode(MIDDLE_IN1,OUTPUT);
  pinMode(MIDDLE_IN2,OUTPUT); 
  pinMode(INDEX_IN3,OUTPUT);
  pinMode(INDEX_IN4,OUTPUT); 

  // For DRV8833 set all pins LOW
  digitalWrite(PINK_IN1, LOW);
  digitalWrite(PINK_IN2, LOW);
  digitalWrite(RING_IN3, LOW);
  digitalWrite(RING_IN4, LOW);
  digitalWrite(MIDDLE_IN1, LOW);
  digitalWrite(MIDDLE_IN2, LOW);
  digitalWrite(INDEX_IN3, LOW);
  digitalWrite(INDEX_IN4, LOW);

  pinkMotor.init();
  ringMotor.init();
}

void setupEncoders() {
  pinkEncoder.init();
  ringEncoder.init();
}

// -------------------------------------------------- //
// Setup and Main                                     //
// -------------------------------------------------- //
void setup()
{
  Serial.begin(9600);
  Serial.println("Setting Up..."); 

  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2
  rPiLink.init(ARDUINO_1_ADDRESS);
  // Wire.begin(0x05);

  // RPi wants the status to be 1 otherwise it will report a brownout.
  rPiLink.buffer.status = 1;
  rPiLink.buffer.pinkMotor = 0;
  rPiLink.buffer.ringMotor = 0;
  
  // Setup pin 13 as output and turn LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  setupMotors();
  setupEncoders();
}

void loop() {

  // Get the latest data including recent i2c master writes
  rPiLink.updateBuffer();

  // Constantly write the firmware ident
  rPiLink.buffer.firmwareIdent = FIRMWARE_IDENT;
  rPiLink.buffer.status = 1;

  // Update the built-ins.  These are 4 boolean values
  rPiLink.buffer.builtinDioValues[0] = digitalRead(BUTTON_PIN);

  // Check if button A is pressed
  // if (rPiLink.buffer.builtinDioValues[0] == HIGH) {
  //   Serial.println("ButtonA is pressed...");
  //   rPiLink.buffer.pinkMotor = 200;
  //   digitalWrite(PINK_IN1, HIGH);
  //   digitalWrite(PINK_IN2, LOW);
  // } 
  // else {
  //   rPiLink.buffer.pinkMotor = 0;
  //   digitalWrite(PINK_IN1, LOW);
  //   digitalWrite(PINK_IN2, LOW);
  // }

  // digitalWrite(LED_BUILTIN, rPiLink.buffer.builtinDioValues[3]);
  // if (rPiLink1.buffer.builtinDioValues[3] == true) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  // } else {
  //   digitalWrite(LED_BUILTIN, LOW);
  // }
  
  startMotors(rPiLink.buffer.pinkMotor,
              rPiLink.buffer.ringMotor,
              rPiLink.buffer.middleMotor,
              rPiLink.buffer.indexMotor); 

  // Encoders
  if (rPiLink.buffer.resetLeftEncoder) {
    rPiLink.buffer.resetLeftEncoder = false;
    pinkEncoder.resetEncoder();
  }

  if (rPiLink.buffer.resetRightEncoder) {
    rPiLink.buffer.resetRightEncoder = false;
    ringEncoder.resetEncoder();
  }

  // rPiLink.buffer.leftEncoder = encoders.getCountsLeft();
  // rPiLink.buffer.rightEncoder = encoders.getCountsRight();

  // readPinkEncoder();
  pinkEncoder.readEncoder();
  ringEncoder.readEncoder();

  // Write to buffer
  rPiLink.finalizeWrites();
}
