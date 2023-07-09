#include <Arduino.h>

#include "shmem_buffer.h"
#include "Motor.h"
#include "Wire.h"
#include "ESP32RPiSlave.h"

// TwoWire I2CBME = TwoWire(0); 
// #define ARDUINO_1_ADDRESS 20 // I2C Address of Arduino 1
// #define ARDUINO_2_ADDRESS 21 // I2C Address of Arduino 1

// Addresses for esp32s
#define I2C_DEV_ADDR 0x55
#define I2C_SDA 21
#define I2C_SCL 22
#define SERVER 0
#define CLIENT 1
#define ANSWERSIZE 5
const uint8_t mode = SERVER;

// Define string with response to Master
String answer = "Hello";

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

// Motor pinkMotor = Motor(PINK_ENCODER, PINK_IN1, PINK_IN2, MOTOR_MODE);
// Motor ringMotor = Motor(RING_IN3, RING_IN4, RING_ENCODER);
// Motor middleMotor = Motor(MIDDLE_IN1, MIDDLE_IN2, MIDDLE_ENCODER);
// Motor indexMotor = Motor(INDEX_IN3, INDEX_IN4, INDEX_ENCODER);

// #define LED_BUILTIN 2
#define BUTTON_PIN1 12
#define BUTTON_PIN2 14
#define BUTTON_PIN3 13

// int loop_count = 0;
// int print_count = 0;
// int button_state = LOW;

uint32_t i = 0;

void setupMotors() {
  
  // pinkMotor.init();
  // ringMotor.init();
  // middleMotor.init();
  // indexMotor.init();
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

void requestEvent(){
  // Setup byte variable in the correct size
  byte response[ANSWERSIZE];
  
  // Format answer as array
  for (byte i=0;i<ANSWERSIZE;i++) {
    response[i] = (byte)answer.charAt(i);
  }
  
  // Send response back to Server
  Wire.write(response,sizeof(response));
  
  // Print to Serial Monitor
  Serial.println("Request event");
  // // Wire.print(i++);
  // // Wire.print(" Packets.");
  // // Serial.println("onRequest");
  // rPiLink.transmit();
}

void receiveEvent(int len){

  // rPiLink.receive(len);

  // Print to Serial Monitor
  Serial.println("Receive event");
  Serial.printf("onReceive[%d]: ", len);
  while(Wire.available()){
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setupI2C() {
  // Join I2C bus as slave with address 0x20 Arduino 1
  // or 0x21 for Arduino 2

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);

  if (mode == SERVER) {
    Wire.begin(I2C_SDA, I2C_SCL);
  } else {
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Wire.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL);
  }

}

void requestFromClient() {

  // Write a character to the client
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  // Read response from Slave
  // Read back 5 characters
  Wire.requestFrom(I2C_DEV_ADDR,ANSWERSIZE);
  
  // Add characters to string
  String response = "";
  while (Wire.available()) {
      char b = Wire.read();
      response += b;
  } 
  
  // Print to Serial Monitor
  Serial.println(response);
  
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
  setupI2C();

  // RPi wants the status to be 1 otherwise it will report a brownout.
  rPiLink.buffer.status = 1;
  rPiLink.buffer.pinkMotor = 0;
  rPiLink.buffer.ringMotor = 0;
  rPiLink.buffer.middleMotor = 0;
  rPiLink.buffer.indexMotor = 0;
  
  // Setup pin 13 as output and turn LED off
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN3, INPUT_PULLUP);

  setupMotors();
}

void loop() {

  // Get the latest data including recent i2c master writes
  rPiLink.updateBuffer();

  if (mode == SERVER) {
    requestFromClient();
    i2cScan();
  }  

  // Constantly write the firmware ident
  rPiLink.buffer.firmwareIdent = FIRMWARE_IDENT;
  rPiLink.buffer.status = 1;

  if (digitalRead(BUTTON_PIN1) == LOW) {
    // pinkMotor.encoder.resetEncoder();
  }

  if (digitalRead(BUTTON_PIN2) == LOW) {
    rPiLink.buffer.pinkMotor = 200;
  }
  else if (digitalRead(BUTTON_PIN3) == LOW) {
    rPiLink.buffer.pinkMotor = -200;
  } else {
    rPiLink.buffer.pinkMotor = 0;
  }

  if (MOTOR_MODE == PWM ) {
    // pinkMotor.applyPWMPower(rPiLink.buffer.pinkMotor);
  } else {
    // pinkMotor.applyPower(rPiLink.buffer.pinkMotor);
  }
  
  // Write to buffer
  rPiLink.finalizeWrites();
}
