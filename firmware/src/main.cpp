#include <Arduino.h>

#include "shmem_buffer.h"
#include "Motor.h"
#include "Wire.h"
#include "ESP32RPiSlave.h"

// TwoWire I2CBME = TwoWire(0); 
// #define ARDUINO_1_ADDRESS 20 // I2C Address of Arduino 1
// #define ARDUINO_2_ADDRESS 21 // I2C Address of Arduino 1

// Addresses for esp32s
// #define I2C_DEV_ADDR 0x55
#define I2C_DEV_ADDR 20
#define I2C_SDA 32
#define I2C_SCL 33

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

// Define the motor mode here!
#define DIGITAL 0
#define PWM 1
#define MOTOR_MODE PWM

Motor pinkMotor = Motor(PINK_ENCODER, PINK_IN1, PINK_IN2, MOTOR_MODE);
// Motor ringMotor = Motor(RING_IN3, RING_IN4, RING_ENCODER);
// Motor middleMotor = Motor(MIDDLE_IN1, MIDDLE_IN2, MIDDLE_ENCODER);
// Motor indexMotor = Motor(INDEX_IN3, INDEX_IN4, INDEX_ENCODER);

#define LED_BUILTIN 2
#define BUTTON_PIN1 12
#define BUTTON_PIN2 14
#define BUTTON_PIN3 13

// int loop_count = 0;
// int print_count = 0;
// int button_state = LOW;

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

// void logOutput(double pinkSpeed, double ringSpeed, double middleSpeed, double indexSpeed) {
//   if (loop_count > 2000) {
//     // Serial.print(print_count); Serial.print(" Firmware "); Serial.println(rPiLink.buffer.firmwareIdent);
//     loop_count = 0;
//     print_count += 1;
//   }
//   loop_count += 1;
// }

// double applyDeadband(double input, double threshold) {
//   if (input < -threshold || input > threshold) {
//     return input;
//   }
//   return 0.0;
// }

void setupMotors() {
  
  pinkMotor.init();
  // ringMotor.init();
  // middleMotor.init();
  // indexMotor.init();
}

void setupI2C() {
  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2

  // rPiLink.init(I2C_DEV_ADDR);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(I2C_DEV_ADDR, I2C_SDA, I2C_SCL);

// #if CONFIG_IDF_TARGET_ESP32
//   char message[64];
//   snprintf(message, 64, "%u Packets.", i++);
//   Wire.slaveWrite((uint8_t *)message, strlen(message));
// #endif
}

void i2cScan() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

// -------------------------------------------------- //
// Setup and Main                                     //
// -------------------------------------------------- //
void setup()
{
  Serial.begin(115200);
  Serial.println("Setting Up..."); 

  // setupI2C();
  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2
  // rPiLink.init(I2C_DEV_ADDR);

  // RPi wants the status to be 1 otherwise it will report a brownout.
  rPiLink.buffer.status = 1;
  rPiLink.buffer.pinkMotor = 0;
  rPiLink.buffer.ringMotor = 0;
  rPiLink.buffer.middleMotor = 0;
  rPiLink.buffer.indexMotor = 0;
  
  // Setup pin 13 as output and turn LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN3, INPUT_PULLUP);

  setupMotors();
}

void loop() {

  // Get the latest data including recent i2c master writes
  rPiLink.updateBuffer();

  // Constantly write the firmware ident
  rPiLink.buffer.firmwareIdent = FIRMWARE_IDENT;
  rPiLink.buffer.status = 1;

  // Update the built-ins.  These are 4 boolean values
  // rPiLink.buffer.builtinDioValues[0] = digitalRead(BUTTON_PIN);

  if (digitalRead(BUTTON_PIN1) == LOW) {
    pinkMotor.encoder.resetEncoder();
    // i2cScan();
    // ringMotor.encoder.resetEncoder();
    // middleMotor.encoder.resetEncoder();
    // indexMotor.encoder.resetEncoder();
  }

  if (digitalRead(BUTTON_PIN2) == LOW) {
    rPiLink.buffer.pinkMotor = 200;
  }
  else if (digitalRead(BUTTON_PIN3) == LOW) {
    rPiLink.buffer.pinkMotor = -200;
  } else {
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

  if (MOTOR_MODE == PWM ) {
    pinkMotor.applyPWMPower(rPiLink.buffer.pinkMotor);
  } else {
    pinkMotor.applyPower(rPiLink.buffer.pinkMotor);
  }
  
  
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
