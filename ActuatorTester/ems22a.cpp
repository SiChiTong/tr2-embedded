#include <Arduino.h>
#include "ems22a.h"
#include <EEPROM.h>

Ems22a::Ems22a() {
}

Ems22a::Ems22a(int p1, int p2, int p3, int p4, int p5, int p6) {
  this->PIN_INPUT = p1;
  this->PIN_CLOCK = p2;
  this->PIN_GND = p3;
  this->PIN_DATA = p4;
  this->PIN_VCC = p5;
  this->PIN_CS = p6;
}

Ems22a::~Ems22a() {
}

void Ems22a::setUp() {
  pinMode(PIN_INPUT, INPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT_PULLUP);
  pinMode(PIN_CS, OUTPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);
  
  readPosition();
}

void Ems22a::setOffset(int pos) {
  offset = pos;
}

float Ems22a::readDegree(bool reverseAngle = false) {
  int pos = readPosition(reverseAngle);
  return ((pos + 1.0) / 1024.0) * 360.0;
}


int Ems22a::readPosition(bool noOffset = false) {
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CS, LOW);
  int pos = 0;
  for (int i=0; i<10; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);
   
    byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
    pos += b * pow(2, 10-(i+1));
  }

  for (int i=0; i<6; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);
  }
  
  digitalWrite(PIN_CLOCK, LOW);
  digitalWrite(PIN_CLOCK, HIGH);

  if (noOffset == true) {
    return pos;
  }
  
  pos -= offset;
  
  if (pos < 0) pos += encoderResolution;
  else if (pos > encoderResolution) pos -= encoderResolution;

  // set previous positions
  for (int i = prevPositionN - 1; i > 0; i--) {
    prevPosition[i] = prevPosition[i - 1];
  }
  prevPosition[0] = pos;
  
  return pos;
}

void Ems22a::setLap(int i) {
  if (i < maxLap && i >= 0) {
    lapNumber = i;
  }
}

void Ems22a::changeLap(int i) {
  if (i > 0) {
    if (lapNumber >= 2) {
      lapNumber = 0;
    } else {
      lapNumber++;
    }
  } else if (i < 0) {
    if (lapNumber <= 0) {
      lapNumber = 2;
    } else {
      lapNumber--;
    }
  }

  EEPROM.write(1, lapNumber);
}

float Ems22a::getAngleDegrees() {
  float pos = prevPosition[0] + (lapNumber * encoderResolution);
  float maxPos = maxLap * encoderResolution;
  return 360.0 *  pos / maxPos;
}

float Ems22a::getAngleRadians() {
  float pos = prevPosition[0] + (lapNumber * encoderResolution);
  float maxPos = maxLap * encoderResolution;
  return PI * 2.0 *  pos / maxPos;
}

int Ems22a::step() {
  readPosition();

  bool read1Small = prevPosition[0] < encoderResolution / 2.0;
  bool read2Small = prevPosition[1] < encoderResolution / 2.0;

  if (prevPosition[1] > 1000 && (read1Small && !read2Small)) {
    changeLap(1);
  } else if (prevPosition[0] > 1000 && (!read1Small && read2Small)) {
    changeLap(-1);
  }

  return prevPosition[0] + (lapNumber * encoderResolution);
}
