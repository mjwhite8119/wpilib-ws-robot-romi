#include <Arduino.h>
#include <PololuRPiSlave.h>

#include "shmem_buffer.h"

// Addresses for Arduinos
#define ARDUINO_1_ADDRESS 20 // I2C Address of Arduino 1
#define ARDUINO_2_ADDRESS 21 // I2C Address of Arduino 1

// Buffer and delay time
PololuRPiSlave<Data, 20> rPiLink;

// Nano PWM pins 3,5,6,9,10 and 11. 
#define PINK_PHASE_IN1 2 // direction
#define PINK_ENABLE_IN2 3 // PWM
#define RING_PHASE_IN3 4 // direction
#define RING_ENABLE_IN4 5 // PWM
#define MIDDLE_PHASE_IN1 8 // direction
#define MIDDLE_ENABLE_IN2 9 // PWM
#define INDEX_PHASE_IN3 10 // direction
#define INDEX_ENABLE_IN4 11 // PWM

// Used for the DRV8835
// #define MODE_PIN 8

// --- Define ENCODERS ---

#define POTENTIOMETER_PIN A0
#define BUTTON_PIN 2

int loop_count = 0;
int button_state = LOW;

// --------------------------------------------------
/*                IN3/IN1           IN4/IN2
   Forward          HIGH              LOW
   Reverse          LOW               HIGH
   Stop             LOW               LOW
   Stop             HIGH              HIGH 
*/ 
void startMotors(int pinkSpeed, int ringSpeed, int middleSpeed, int indexSpeed) {
  // FORWARD
  // Serial.println(speed);
  digitalWrite(PINK_PHASE_IN1,LOW); 
  analogWrite(PINK_ENABLE_IN2,pinkSpeed);
  digitalWrite(RING_PHASE_IN3,LOW); 
  analogWrite(RING_ENABLE_IN4,ringSpeed);

  digitalWrite(MIDDLE_PHASE_IN1,LOW); 
  analogWrite(MIDDLE_ENABLE_IN2,middleSpeed);
  digitalWrite(INDEX_PHASE_IN3,LOW); 
  analogWrite(INDEX_ENABLE_IN4,indexSpeed);
}

void stopMotors() {
  // STOP
  digitalWrite(INDEX_PHASE_IN3,LOW); 
  analogWrite(INDEX_ENABLE_IN4,0);
}

void reverseMotors(int speed) {
  // REVERSE
  digitalWrite(INDEX_PHASE_IN3,HIGH); 
  analogWrite(INDEX_ENABLE_IN4,speed);
}

void readPot()
{
  int data = analogRead(POTENTIOMETER_PIN);
  int percentage = map(data, 0, 1023, 0, 100);
  Serial.print("Potentiometer at ");
  Serial.print(percentage);
  Serial.println("%");
  delay(500);
}

// -------------------------------------------------- //
// Setup and Main                                     //
// -------------------------------------------------- //
void setup()
{
  Serial.begin(9600);
  Serial.print("Setting Up..."); 

  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2
  rPiLink.init(ARDUINO_1_ADDRESS);
  // Wire.begin(0x05);

  // RPi wants the status to be 1 otherwise it will report a brownout.
  rPiLink.buffer.status = 1;
  rPiLink.buffer.pinkMotor = 0;
  
  // Setup pin 13 as output and turn LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup motor pins
  pinMode(PINK_PHASE_IN1,OUTPUT);
  pinMode(PINK_ENABLE_IN2,OUTPUT); 
  pinMode(RING_PHASE_IN3,OUTPUT);
  pinMode(RING_ENABLE_IN4,OUTPUT); 
  pinMode(MIDDLE_PHASE_IN1,OUTPUT);
  pinMode(MIDDLE_ENABLE_IN2,OUTPUT); 
  pinMode(INDEX_PHASE_IN3,OUTPUT);
  pinMode(INDEX_ENABLE_IN4,OUTPUT); 

  // For DRV8833 set all pins LOW
  digitalWrite(PINK_PHASE_IN1, LOW);
  digitalWrite(PINK_ENABLE_IN2, LOW);
  digitalWrite(RING_PHASE_IN3, LOW);
  digitalWrite(RING_ENABLE_IN4, LOW);
  digitalWrite(MIDDLE_PHASE_IN1, LOW);
  digitalWrite(MIDDLE_ENABLE_IN2, LOW);
  digitalWrite(INDEX_PHASE_IN3, LOW);
  digitalWrite(INDEX_ENABLE_IN4, LOW);

  // For Polulu DRV8835
  // pinMode(MODE_PIN,OUTPUT);
  // digitalWrite(MODE_PIN, HIGH);
}

void loop() {

  // Get the latest data including recent i2c master writes
  rPiLink.updateBuffer();

  // Constantly write the firmware ident
  rPiLink.buffer.firmwareIdent = FIRMWARE_IDENT;

  // Update the built-ins.  These are 4 boolean values
  rPiLink.buffer.builtinDioValues[0] = digitalRead(BUTTON_PIN);

  // Check if button A is pressed
  // if (rPiLink.buffer.builtinDioValues[0] == HIGH) {
  //   Serial.println("ButtonA is pressed...");
  //   rPiLink.buffer.pinkMotor = 200;
  //   rPiLink.buffer.ringMotor = 0;
  // } 
  // else {
  //   rPiLink.buffer.leftMotor = 0;
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

  // readPot();

  // Write to buffer
  rPiLink.finalizeWrites();
}
