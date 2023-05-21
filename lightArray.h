#include <QTRSensors.h>
#include "Wire.h"

extern QTRSensors qtrSensor;
#define SENSOR_COUNT 8
extern uint16_t qtrSensorValues[SENSOR_COUNT];
#define PIN_ONE 24
#define PIN_TWO 26
#define PIN_THREE 28
#define PIN_FOUR 30
#define PIN_FIVE 39
#define PIN_SIX 29
#define PIN_SEVEN 27
#define PIN_EIGHT 25
#define EMITTER_PIN 22
#define EYE_ONE_EIGHT_ERROR 0
#define EYE_TWO_SEVEN_ERROR 0
#define EYE_THREE_SIX_ERROR 0
#define EYE_FOUR_FIVE_ERROR 0
#define KP 0.03
#define KD 0
#define BLACK_THRESH 12v00
void qtrBegin();
bool onLine();
bool allOnLine();
bool sensorNumOnLine(int sensorNum);
bool amtOnLine(int amt);
bool noSensorsOnLine();
int whichSensorsOnLine();
void printQtr();
