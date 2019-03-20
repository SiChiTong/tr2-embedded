#ifndef EMS22A_H
#define EMS22A_H

class Ems22a {
  private:
    int PIN_INPUT;
    int PIN_CLOCK;
    int PIN_GND;
    int PIN_DATA;
    int PIN_VCC;
    int PIN_CS;

    int encoderResolution = 1021;
    int lapNumber = 0;
    int maxLap = 3;
    static const int prevPositionN = 3;
    int prevPosition[prevPositionN];
    int offset = 0;

    void changeLap(int i);
  
  public:
    void setUp();
    void setLap(int i);
    void setOffset(int pos);
    int readPosition(bool reverseAngle = false);
    int step();
    float getAngleRadians();
    Ems22a();
    Ems22a(int, int, int, int, int, int);
    ~Ems22a();
};

#endif
