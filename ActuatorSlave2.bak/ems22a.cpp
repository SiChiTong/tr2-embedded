#include <Arduino.h>
#include "ems22a.h"

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

void Ems22a::setOffset(uint16_t pos) {
  if (pos > encoderResolution || pos < 0) {
    pos = 0;
  }
  offset = pos;
}

int Ems22a::readPosition(bool noOffset) {
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

int Ems22a::getOffset() {
  return offset;
}

int Ems22a::getLap() {
  return lapNumber;
}

void Ems22a::setMaxLap(uint8_t i) {
  maxLap = i;
  if (lapNumber > i) {
    lapNumber = i;
  }
}

void Ems22a::setLap(uint8_t i) {
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
}

float Ems22a::getAngleRadians() {
  float pos = prevPosition[0] + (lapNumber * encoderResolution);
  
  if (maxLap <= 1) {
    pos = prevPosition[0];
  }
  
  float maxPos = maxLap * encoderResolution;
  return PI * 2.0 *  pos / maxPos;
}

int Ems22a::getPosition() {
  if (maxLap <= 1) {
    return prevPosition[0];
  }
  
  return prevPosition[0] + (lapNumber * encoderResolution);
}

int Ems22a::step() {
  readPosition();

  if (maxLap <= 1) {
    return prevPosition[0];
  }

  bool read1Small = prevPosition[0] < encoderResolution / 2.0;
  bool read2Small = prevPosition[1] < encoderResolution / 2.0;

  if (prevPosition[1] > 950 && (read1Small && !read2Small)) {
    changeLap(1);
  } else if (prevPosition[0] > 950 && (!read1Small && read2Small)) {
    changeLap(-1);
  }

  return prevPosition[0] + (lapNumber * encoderResolution);
}
