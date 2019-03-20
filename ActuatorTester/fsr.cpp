#include <Arduino.h>
#include "fsr.h"

FSR::FSR() {
}

FSR::FSR(int p1) {
  this->PIN_INPUT = p1;
}

FSR::~FSR() {
}

void FSR::setUp() {
  pinMode(PIN_INPUT, INPUT_PULLUP);
}

void FSR::setThreshold(int _min, int _max) {
  minThreshold = _min;
  maxThreshold = _max;
}

void FSR::setFilterMode(int m) {
  mode = m;
}

int FSR::applyThreshold (int reading) {
  if (reading >= minThreshold && reading <= maxThreshold) {
    reading = 0;
  } else if (reading <= minThreshold) {
    reading -= minThreshold;
  } else if (reading >= maxThreshold) {
    reading -= maxThreshold;
  } else {
    reading = 0;
  }
  return reading;
}

void FSR::recordReading(int currentRead) {
  _readCount++;
  for (int i = prevReadN - 1; i > 0; i--) {
    prevRead[i] = prevRead[i - 1];
  }
  prevRead[0] = currentRead;
}

float FSR::getPrevReadAvg() {
  int avgSum = 0;

  int n = prevReadN;
  if (_readCount < prevReadN) n = _readCount;

  for (int i = 0; i < n; i++) {
    avgSum += applyThreshold(prevRead[i]);
  }
  return avgSum / n;
}

int FSR::read() {
  int reading = analogRead(PIN_INPUT);
  recordReading(reading);
  
  if (mode == FSR_FILTER_THRESHOLD) {
    return applyThreshold(reading);
  }

  if (mode == FSR_FILTER_THRESHOLD_AVG) {
    return floor(getPrevReadAvg());
  }

  return reading;
}

void FSR::setFilter(int filterMode) {
  mode = filterMode;
}

int FSR::getRead(bool noFilter = false) {
  int reading = prevRead[0];

  if (noFilter == true) {
    return reading;
  }
  
  if (mode == FSR_FILTER_THRESHOLD) {
    return applyThreshold(reading);
  }

  if (mode == FSR_FILTER_THRESHOLD_AVG) {
    return floor(getPrevReadAvg());
  }

  return reading;
}

void FSR::step() {
  read();
}

bool FSR::isMaxRead() {
  return (prevRead[0] > maxRead);
}

void FSR::setMaxRead(int m) {
  maxRead = m;
}

int FSR::getMaxRead() {
  return maxRead;
}
