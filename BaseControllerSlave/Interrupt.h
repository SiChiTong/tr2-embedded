#ifndef INTERRUPT_H
#define INTERRUPT_H

volatile int interruptL_pulses, interruptR_pulses;

static void interruptL() {
  int interruptL_A = 2;
  int interruptL_B = 22;
  if (digitalRead(interruptL_B) == 0) {
    if (digitalRead(interruptL_A) == 0) {
      interruptL_pulses--;
    } else {
      interruptL_pulses++;
    }
  } else {
    if (digitalRead(interruptL_A) == 0) {
      interruptL_pulses++;
    } else {
      interruptL_pulses--;
    }
  }
}

static void interruptR() {
  int interruptR_A = 3;
  int interruptR_B = 23;
  if (digitalRead(interruptR_A) == 0) {
    if (digitalRead(interruptR_A) == 0) {
      interruptR_pulses--;
    } else {
      interruptR_pulses++;
    }
  } else {
    if (digitalRead(interruptR_A) == 0) {
      interruptR_pulses++;
    } else {
      interruptR_pulses--;
    }
  }
}

#endif
