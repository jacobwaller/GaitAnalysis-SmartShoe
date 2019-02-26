#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Pins for pressure sensors
#define HEEL_PIN 0
#define MEDIAL_MID 2
#define LATERAL_MID 4
#define MEDIAL_FOREFOOT 5

#define DATA_RATE 30.0 //Number of readings you want per second

#define LOW_VAL 0
#define HIGH_VAL 1024

RF24 radio(6,5); // CE, CSN
const byte address[6] = "00001";

double vals[4];
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

void setup() {
  Serial.begin(9600);   // Starts the serial port at 9600 baud 
  Wire.begin();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
  /* Reset LEDs to off */
  digitalWrite(6,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);

  /* Get total acceleration */
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  long sum_Accel = pow(pow(ax,2) + pow(ay,2) + pow(az,2),0.5);
  
  vals[0] = map(analogRead(HEEL_PIN),LOW_VAL,HIGH_VAL, 0,1000);
  vals[1] = map(analogRead(MEDIAL_MID),LOW_VAL,HIGH_VAL, 0,1000);
  vals[2] = map(analogRead(LATERAL_MID),LOW_VAL,HIGH_VAL, 0,1000);
  vals[3] = map(analogRead(MEDIAL_FOREFOOT),LOW_VAL,HIGH_VAL, 0,1000);

  /* Print 5 values: 4 Pressure sensors and 1 acceration */
  String sender = "";
  for(int i=0; i<4;i++) {
    sender+=vals[i];
    sender+='\t';
  }
  sender+=(sum_Accel-15300);

  /* Send data to the radio */
  radio.write(sender.c_str(), sender.length());

  /* Write Values to LEDs */
  analogWrite(5,vals[0]/4);
  analogWrite(6,vals[1]/4);
  analogWrite(9,vals[2]/4);
  analogWrite(10,vals[3]/4);

  
//  for(int i=9;i<12;i++) {
//    analogWrite(i,vals[i-8]/4);
//  }
  
  delay(1000.0/DATA_RATE);
}
