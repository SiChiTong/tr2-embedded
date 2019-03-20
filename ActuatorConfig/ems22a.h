#ifndef EMS22A_H
#define EMS22A_H

class Ems22a {
  private:
    uint8_t PIN_INPUT;
    uint8_t PIN_CLOCK;
    uint8_t PIN_GND;
    uint8_t PIN_DATA;
    uint8_t PIN_VCC;
    uint8_t PIN_CS;
  
  public:
    void setUp();
    float readDegree(bool reverseAngle = false);
    int readPosition(bool reverseAngle = false);
    Ems22a();
    Ems22a(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    ~Ems22a();
};

#endif
