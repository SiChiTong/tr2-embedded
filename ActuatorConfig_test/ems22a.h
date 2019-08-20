#ifndef EMS22A_H
#define EMS22A_H

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

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
    static const int prevPositionN = 10;
    int prevPosition[prevPositionN];
    int offset = 0;

    void changeLap(int i);
  
  public:
    void setUp();
    void setLap(int i);
    void setOffset(int pos);
    float readDegree(bool reverseAngle = false);
    int readPosition(bool reverseAngle = false);
    int step();
    float getAngleDegrees();
    float getAngleRadians();
    Ems22a();
    Ems22a(int, int, int, int, int, int);
    ~Ems22a();
};

#endif
