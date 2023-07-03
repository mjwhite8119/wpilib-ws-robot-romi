#include <Arduino.h>

#include "shmem_buffer.h"
#include "Motor.h"
#include "Wire.h"
#include "ESP32RPiSlave.h"

// Addresses for esp32s
#define I2C_DEV_ADDR 0x55
// #define ARDUINO_1_ADDRESS 20 // I2C Address of Arduino 1
// #define ARDUINO_2_ADDRESS 21 // I2C Address of Arduino 1
#define LED_BUILTIN 2

// Buffer and delay time
ESP32RPiSlave<Data, 20> rPiLink;

// --- Define ENCODERS ---
// These refer to the GPIO numbers
#define PINK_ENCODER 34
#define RING_ENCODER 35
#define MIDDLE_ENCODER 32
#define INDEX_ENCODER 33

// --- Define MOTORS ---
// These refer to the GPIO numbers
#define PINK_IN1 26
#define PINK_IN2 27 
#define RING_IN3 19 
#define RING_IN4 18 
#define MIDDLE_IN1 5 
#define MIDDLE_IN2 17 
#define INDEX_IN3 16 
#define INDEX_IN4 4 

Motor pinkMotor = Motor(PINK_ENCODER, PINK_IN1, PINK_IN2);
// Motor ringMotor = Motor(RING_IN3, RING_IN4, RING_ENCODER);
// Motor middleMotor = Motor(MIDDLE_IN1, MIDDLE_IN2, MIDDLE_ENCODER);
// Motor indexMotor = Motor(INDEX_IN3, INDEX_IN4, INDEX_ENCODER);

#define BUTTON_PIN 14
#define BUTTON_PIN2 12

int loop_count = 0;
int print_count = 0;
int button_state = LOW;

uint32_t i = 0;

void onRequest(){
  // Wire.print(i++);
  // Wire.print(" Packets.");
  // Serial.println("onRequest");
  rPiLink.transmit();
}

void onReceive(int len){
  rPiLink.receive(len);
  // Serial.printf("onReceive[%d]: ", len);
  // while(Wire.available()){
  //   Serial.write(Wire.read());
  // }
  // Serial.println();
}

void logOutput(double pinkSpeed, double ringSpeed, double middleSpeed, double indexSpeed) {
  if (loop_count > 2000) {
    // Serial.print(print_count); Serial.print(" Firmware "); Serial.println(rPiLink.buffer.firmwareIdent);
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

void setupMotors() {
  // Setup motor pins
  // pinMode(PINK_IN1,OUTPUT);
  // pinMode(PINK_IN2,OUTPUT); 
  // pinMode(RING_IN3,OUTPUT);
  // pinMode(RING_IN4,OUTPUT); 
  // pinMode(MIDDLE_IN1,OUTPUT);
  // pinMode(MIDDLE_IN2,OUTPUT); 
  // pinMode(INDEX_IN3,OUTPUT);
  // pinMode(INDEX_IN4,OUTPUT); 

  // For DRV8833 set all pins LOW
  // digitalWrite(PINK_IN1, LOW);
  // digitalWrite(PINK_IN2, LOW);
  // digitalWrite(RING_IN3, LOW);
  // digitalWrite(RING_IN4, LOW);
  // digitalWrite(MIDDLE_IN1, LOW);
  // digitalWrite(MIDDLE_IN2, LOW);
  // digitalWrite(INDEX_IN3, LOW);
  // digitalWrite(INDEX_IN4, LOW);

  pinkMotor.init();
  // ringMotor.init();
  // middleMotor.init();
  // indexMotor.init();
}

// -------------------------------------------------- //
// Setup and Main                                     //
// -------------------------------------------------- //
void setup()
{
  Serial.begin(115200);
  Serial.println("Setting Up..."); 

  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2
  // rPiLink.init(I2C_DEV_ADDR);
  // Wire.onReceive(onReceive);
  // Wire.onRequest(onRequest);
  // Wire.begin((uint8_t)I2C_DEV_ADDR);

  // RPi wants the status to be 1 otherwise it will report a brownout.
  rPiLink.buffer.status = 1;
  rPiLink.buffer.pinkMotor = 0;
  rPiLink.buffer.ringMotor = 0;
  rPiLink.buffer.middleMotor = 0;
  rPiLink.buffer.indexMotor = 0;
  
  // Setup pin 13 as output and turn LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);

  setupMotors();

// #if CONFIG_IDF_TARGET_ESP32
//   char message[64];
//   snprintf(message, 64, "%u Packets.", i++);
//   Wire.slaveWrite((uint8_t *)message, strlen(message));
// #endif
}

void loop() {

  // Get the latest data including recent i2c master writes
  rPiLink.updateBuffer();

  // Constantly write the firmware ident
  rPiLink.buffer.firmwareIdent = FIRMWARE_IDENT;
  rPiLink.buffer.status = 1;

  // Update the built-ins.  These are 4 boolean values
  // rPiLink.buffer.builtinDioValues[0] = digitalRead(BUTTON_PIN);

  if (digitalRead(BUTTON_PIN) == LOW) {
    pinkMotor.encoder.resetEncoder();
    // ringMotor.encoder.resetEncoder();
    // middleMotor.encoder.resetEncoder();
    // indexMotor.encoder.resetEncoder();
  }

  if (digitalRead(BUTTON_PIN2) == LOW) {
    rPiLink.buffer.pinkMotor = 200;
    // pinkMotor.encoder.readEncoder();
  }
  else {
    rPiLink.buffer.pinkMotor = 0;
  }
  
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

  pinkMotor.applyPower(rPiLink.buffer.pinkMotor);
  // pinkMotor.applyPWMPower(rPiLink.buffer.pinkMotor);
  // ringMotor.applyPower(rPiLink.buffer.ringMotor);
  // middleMotor.applyPower(rPiLink.buffer.middleMotor);
  // indexMotor.applyPower(rPiLink.buffer.indexMotor);

  // Encoders
  // if (rPiLink.buffer.resetLeftEncoder) {
  //   rPiLink.buffer.resetLeftEncoder = false;
  //   pinkMotor.encoder.resetEncoder();
  // }

  // if (rPiLink.buffer.resetRightEncoder) {
  //   rPiLink.buffer.resetRightEncoder = false;
  //   ringMotor.encoder.resetEncoder();
  // }

  // rPiLink.buffer.leftEncoder = encoders.getCountsLeft();
  // rPiLink.buffer.rightEncoder = encoders.getCountsRight();

  // pinkMotor.encoder.readEncoder();
  // ringMotor.encoder.readEncoder();
  // middleMotor.encoder.readEncoder();
  // indexMotor.encoder.readEncoder();

  // Write to buffer
  rPiLink.finalizeWrites();
}
