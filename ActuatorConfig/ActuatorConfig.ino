#include <EEPROM.h>
#include "Motor.h"
#include "ems22a.h"

// User-Set Value
int I2CBusAddress = 0x21;
float FSRThresholdCoef = 1.00;

// EEPROM Addresses
int EAI2CAddress = 0;
int EAEncoderLap = 1;
int EAFsr1MinThreshold1 = 2;
int EAFsr1MinThreshold2 = 3;
int EAFsr1MaxThreshold1 = 4;
int EAFsr1MaxThreshold2 = 5;
int EAFsr2MinThreshold1 = 6;
int EAFsr2MinThreshold2 = 7;
int EAFsr2MaxThreshold1 = 8;
int EAFsr2MaxThreshold2 = 9;
int EAEncoderOffset1 = 10;
int EAEncoderOffset2 = 11;
int EAMotorP1 = 12;
int EAMotorP2 = 13;
int EAMotorP3 = 14;
int EAEmsP1 = 15;
int EAEmsP2 = 16;
int EAEmsP3 = 17;
int EAFsrDifMinTorque1 = 18;
int EAFsrDifMinTorque2 = 19;
int EAFsrDifMaxTorque1 = 20;
int EAFsrDifMaxTorque2 = 21;

// Pins for components, can be overwritten in EEPROM
// EEPROM integration is in case of arduino-level pin failure
// while maintaining a single code-base for actuators
int MotorP1 = 5;
int MotorP2 = 6;
int MotorP3 = 7;
int EmsP1 = 8;
int EmsP2 = 9;
int EmsP3 = 10;

int fsrPin1 = A6; // right side of PCB
int fsrPin2 = A7; // left side of PCB

Motor motor;
Ems22a ems22a;

int fsr1MinThreshold = 0;
int fsr1MaxThreshold = 0;
int fsr2MinThreshold = 0;
int fsr2MaxThreshold = 0;
long fsrDifMin = 0;
long fsrDifMax = 0;

long offsetBinary16 = 32768;

const int prevFsrReadN = 250;
int prevFsr1Read[prevFsrReadN];
int prevFsr2Read[prevFsrReadN];
void calibrate() {
  long fsr1Sum = 0;
  long fsr2Sum = 0;

  int fsr1Min = 1023;
  int fsr1Max = 0;

  int fsr2Min = 1023;
  int fsr2Max = 0;
  
  for (int i = 0; i < prevFsrReadN; i++) {
    prevFsr1Read[i] = analogRead(fsrPin1);
    prevFsr2Read[i] = analogRead(fsrPin2);

    int dif = prevFsr1Read[i] - prevFsr2Read[i];
    if (dif > fsrDifMax) {
      fsrDifMax = dif;
    } else if (dif < fsrDifMin) {
      fsrDifMin = dif;
    }

    fsr1Sum += prevFsr1Read[i];
    fsr2Sum += prevFsr2Read[i];

    if (prevFsr1Read[i] < fsr1Min) fsr1Min = prevFsr1Read[i];
    if (prevFsr1Read[i] > fsr1Max) fsr1Max = prevFsr1Read[i];
    if (prevFsr2Read[i] < fsr2Min) fsr2Min = prevFsr2Read[i];
    if (prevFsr2Read[i] > fsr2Max) fsr2Max = prevFsr2Read[i];

    float percentComplete = (i + 1.0) / prevFsrReadN;
    Serial.print(percentComplete * 100.0);
    Serial.print("% F1:");
    Serial.print(prevFsr1Read[i]);
    Serial.print(" F2:");
    Serial.println(prevFsr2Read[i]);
    
    delay(50);
  }
  
  delay(2000);

  int fsr1Avg = floor(fsr1Sum / prevFsrReadN);
  int fsr2Avg = floor(fsr2Sum / prevFsrReadN);

  Serial.print("F1 Avg:");
  Serial.print(fsr1Avg);
  Serial.print(" Min:");
  Serial.print(fsr1Min);
  Serial.print(" Max:");
  Serial.println(fsr1Max);
  
  Serial.print("F2 Avg: ");
  Serial.print(fsr2Avg);
  Serial.print(" Min: ");
  Serial.print(fsr2Min);
  Serial.print(" Max: ");
  Serial.println(fsr2Max);

  float fsr1Variance = 0;
  float fsr2Variance = 0;

  for (int i = 0; i < prevFsrReadN; i++) {
    fsr1Variance += pow(prevFsr1Read[i] - fsr1Avg, 2);
    fsr2Variance += pow(prevFsr2Read[i] - fsr2Avg, 2);
  }

  float fsr1StdDev = sqrt(fsr1Variance / prevFsrReadN - 1);
  float fsr2StdDev = sqrt(fsr2Variance / prevFsrReadN - 1);

  Serial.println();
  Serial.print("F1 STDEV:");
  Serial.println(fsr1StdDev);
  Serial.print("F2 STDEV:");
  Serial.println(fsr2StdDev);

  delay(2000);

  Serial.println();

  int minRead = 0;
  int maxRead = 1023;
  
  fsr1MinThreshold = fsr1Avg - (fsr1StdDev * FSRThresholdCoef);//floor(FSRThresholdBuffer * (fsr1Avg - minRead));
  fsr1MaxThreshold = fsr1Avg + (fsr1StdDev * FSRThresholdCoef);//floor(FSRThresholdBuffer * (maxRead - fsr1Avg));
  fsr2MinThreshold = fsr2Avg - (fsr2StdDev * FSRThresholdCoef);//floor(FSRThresholdBuffer * (fsr2Avg - minRead));
  fsr2MaxThreshold = fsr2Avg + (fsr2StdDev * FSRThresholdCoef);//floor(FSRThresholdBuffer * (maxRead - fsr2Avg));

  if (fsr1MinThreshold < 0) fsr1MinThreshold = 0;
  if (fsr1MaxThreshold > 1023) fsr1MaxThreshold = 1023;
  if (fsr2MinThreshold < 0) fsr2MinThreshold = 0;
  if (fsr2MaxThreshold > 1023) fsr2MaxThreshold = 1023;

  Serial.print("F1 Min:");
  Serial.println(fsr1MinThreshold);
  Serial.print("F1 Max:");
  Serial.println(fsr1MaxThreshold);
  Serial.print("F2 Min:");
  Serial.println(fsr2MinThreshold);
  Serial.print("F2 Max:");
  Serial.println(fsr2MaxThreshold);
  
  Serial.print("Dif Min:");
  Serial.println(fsrDifMin);
  Serial.print("Dif Max:");
  Serial.println(fsrDifMax);
}

void setup() {
  Serial.begin(115200);

  pinMode(fsrPin1, INPUT_PULLUP);
  pinMode(fsrPin2, INPUT_PULLUP);
  
  int mp1 = EEPROM.read(EAMotorP1);
  int mp2 = EEPROM.read(EAMotorP2);
  int mp3 = EEPROM.read(EAMotorP3);
  if (mp1 >= 2 && mp1 <= 13) MotorP1 = mp1;
  if (mp2 >= 2 && mp2 <= 13) MotorP2 = mp2;
  if (mp3 >= 2 && mp3 <= 13) MotorP3 = mp3;
  motor = Motor(1, MotorP1, MotorP2, MotorP3);

  int ep1 = EEPROM.read(EAEmsP1);
  int ep2 = EEPROM.read(EAEmsP2);
  int ep3 = EEPROM.read(EAEmsP3);
  if (ep1 >= 2 && ep1 <= 13) EmsP1 = ep1;
  if (ep2 >= 2 && ep2 <= 13) EmsP2 = ep2;
  if (ep3 >= 2 && ep3 <= 13) EmsP3 = ep3;
  ems22a = Ems22a(0, EmsP1, 0, EmsP2, 0, EmsP3);
  
  motor.setUp();
  ems22a.setUp();

  delay(1000);
  
  EEPROM.write(EAI2CAddress, I2CBusAddress);
  Serial.print("I2C Addr: 0x");
  Serial.println(I2CBusAddress, HEX);

  delay(1000);

  int encoderLap = 0;
  EEPROM.write(EAEncoderLap, encoderLap);

  int emsRead = ems22a.readPosition();
  EEPROM.write(EAEncoderOffset1, emsRead % 256);
  EEPROM.write(EAEncoderOffset2, floor(emsRead / 256));
  Serial.print("EMS Offset:");
  Serial.println(emsRead);
  Serial.println();
  
  delay(500);

  Serial.println("Preparing FSR");
  Serial.println("Gently twist actuator back-and-forth until read complete");

  delay(1000);
  
  Serial.println();
  Serial.println("Will continue in 7 sec");
  delay(4000);

  Serial.println(3);
  delay(1000);
  Serial.println(2);
  delay(1000);
  Serial.println(1);
  delay(1000);
  
  Serial.println();
  Serial.println("Calibrating FSR");
  Serial.println();
  
  calibrate();
  
  delay(2000);

  EEPROM.write(EAFsr1MinThreshold1, fsr1MinThreshold % 256);
  EEPROM.write(EAFsr1MinThreshold2, floor(fsr1MinThreshold / 256));
  
  EEPROM.write(EAFsr1MaxThreshold1, fsr1MaxThreshold % 256);
  EEPROM.write(EAFsr1MaxThreshold2, floor(fsr1MaxThreshold / 256));
  
  EEPROM.write(EAFsr2MinThreshold1, fsr2MinThreshold % 256);
  EEPROM.write(EAFsr2MinThreshold2, floor(fsr2MinThreshold / 256));
  
  EEPROM.write(EAFsr2MaxThreshold1, fsr2MaxThreshold % 256);
  EEPROM.write(EAFsr2MaxThreshold2, floor(fsr2MaxThreshold / 256));

  fsrDifMin += offsetBinary16;
  EEPROM.write(EAFsrDifMinTorque1, fsrDifMin % 256);
  EEPROM.write(EAFsrDifMinTorque2, floor(fsrDifMin / 256));
  
  fsrDifMax += offsetBinary16;
  EEPROM.write(EAFsrDifMaxTorque1, fsrDifMax % 256);
  EEPROM.write(EAFsrDifMaxTorque2, floor(fsrDifMax / 256));

  Serial.println();
  Serial.println("Config complete.");
  Serial.println("Now upload main sketch.");
}

void loop() {
  delay(50);
}
