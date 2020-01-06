#include <Arduino.h>
#include "ems22a.h"

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

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
  readPosition();
}

void Ems22a::setOffset(uint16_t pos) {
  if (pos > encoderResolution || pos < 0) {
    pos = 0;
  }
  offset = pos;
}

int Ems22a::readPosition() {
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

  // set previous positions
  for (int i = prevPositionN - 1; i > 0; i--) {
    prevPosition[i] = prevPosition[i - 1];
  }
  prevPosition[0] = pos;

  handleLapChange();
  
  return pos;
}

int Ems22a::getOffset() {
  return offset;
}

int Ems22a::getLap() {
  return lapNumber;
}

void Ems22a::setEqualTo(float posSet) {  
  int encRead = prevPosition[0];
  float posCur = (float)encRead / (encoderResolution * maxLap) * TAU;
  float posDif = fabs(posSet - posCur);
  
  float lap = floor(posDif / TAU * maxLap) + 1;
  posCur = (((encoderResolution * lap) + encRead) / (encoderResolution * maxLap)) * TAU;
  posDif = fabs(posSet - posCur);
  
  int offset = floor(posDif / TAU * encoderResolution * maxLap);
  
  int recCount = 0;
  while (offset > encoderResolution && recCount < 10) {
      recCount++;
      if (offset > encoderResolution) {
          lap -= 1;
          offset -= encoderResolution;
      }
  }
  
  if (lap >= maxLap) {
      lap -= maxLap;
  }

  setOffset(offset);
  setLap(lap);
  
  readPosition();
  readPosition();
}

int Ems22a::getMaxResolution() {
  return encoderResolution * maxLap;
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
  float encRead = prevPosition[0];
  float pos = (encoderResolution * lapNumber) + encRead - offset;
  float maxPos = encoderResolution * maxLap;
  float r = pos / maxPos * TAU;

  int recCount = 0;
  while ((r < 0 || r > TAU) && recCount < 10) {
    recCount++;
    if (r < 0) {
      r += TAU;
    } else if (r > TAU) {
      r -= TAU;
    }
  }
  
  return r;
}

int Ems22a::getPosition() {
  if (maxLap <= 1) {
    return prevPosition[0];
  }
  
  return prevPosition[0] + (lapNumber * encoderResolution);
}

void Ems22a::handleLapChange() {
  if (maxLap <= 1) {
    return;
  }

  int posCur = prevPosition[0];
  int posPrv = prevPosition[1];
  int posTHi = encoderResolution - 75;
  int posTLo = 75;

  bool posCurHi = posCur >= posTHi;
  bool posCurLo = posCur <= posTLo;
  bool posPrvHi = posPrv >= posTHi;
  bool posPrvLo = posPrv <= posTLo;

  if (posCurLo && posPrvHi) {
    changeLap(1);
  } else if (posPrvLo && posCurHi) {
    changeLap(-1);
  }
}

void Ems22a::step() {
  readPosition();
}
