#include "qtrMeMegaPi.h"
uint16_t qtrSensorValues[SENSOR_COUNT];
QTRSensors qtrSensor;
void qtrBegin(){
  qtrSensor.setTypeRC();
  qtrSensor.setSensorPins((const uint8_t[]){PIN_ONE, PIN_TWO, PIN_THREE, PIN_FOUR, PIN_FIVE, PIN_SIX, PIN_SEVEN, PIN_EIGHT}, SENSOR_COUNT);
  qtrSensor.setEmitterPin(EMITTER_PIN);
}
bool onLine() {
  qtrSensor.read(qtrSensorValues);
  for(int i=0;i<8;i++){
    if(qtrSensorValues[i]>1200)
      return true;
  }
  return false;
}
bool allOnLine(){
  qtrSensor.read(qtrSensorValues);
  for(int i=0;i<8;i++){
    if(qtrSensorValues[i]<1200)
      return false;
  }
  return true;
}
bool sensorNumOnLine(int sensorNum){
  qtrSensor.read(qtrSensorValues);
  if(qtrSensorValues[sensorNum]>1200)
    return true;
  return false;
}
bool amtOnLine(int amt){
  qtrSensor.read(qtrSensorValues);
  int temp=0;
  for(int i=0;i<8;i++){
    if(qtrSensorValues[i]>1200)
      temp++;
  }
  if(temp>=amt)return true;
  return false;
}
bool noSensorsOnLine(){
  qtrSensor.read(qtrSensorValues);
  int temp=0;
  for(int i=0;i<8;i++){
    if(qtrSensorValues[i]>1200)
      return false;
  }
  return true;
}
int whichSensorsOnLine(){
  qtrSensor.read(qtrSensorValues);
  if(allOnLine()){ return 0; }
  int left = 0, right = 0;
  for(int i = 0; i < 8; i++){
    if(i <= 3 && qtrSensorValues[i] > 1200){ right++; }
    if(i >= 4 && qtrSensorValues[i] > 1200){ left++; }
  }
  if(left >= right){
    return -1;
  }
  else if(right > left){
    return 1;
  }
  return 0;
}
void printQtr(){
  
}
