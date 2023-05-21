#include "qtrMeMegaPi.h"
#include "MeMegaPi.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Wire.h"
#define RIGHT PORT2B
#define LEFT PORT1B
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define KP 0.03
#define KD -0.008
#define KI 0.0003

#define TCAADDR 0x70


volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;

Servo rClaw; Servo lClaw; Servo rMount; Servo lMount;

void grab(){
  rClaw.write(45); lClaw.write(0); 
  delay(2000);
}

void releaseC(){
  rClaw.write(0); lClaw.write(25);
  delay(2000);
}

void raiseMount(){
  for (int pos = 0; pos <= 140; pos += 70) { 
    lMount.write(pos); 
    delay(10);                       
  }
delay(2000);
}

void lowerMount(){
  for (int pos = 140; pos >= 0; pos -= 5) { 
    lMount.write(pos);
    delay(15);                       
  }
  delay(2000);
}

#define pin A11 //Front ultrasonic sensor pin
#define pin2 A13
//Side ultrasonic sensor pin

MeMegaPiDCMotor motor1(LEFT);
MeMegaPiDCMotor motor2(RIGHT);

#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> rot;

double errorArray[100]={0};
int errArrIndex=0;

volatile int encCount = 0;

int baseSpeed = 30;
int minSpeed = 30;

void Interruptfunc(){
  if(digitalRead(31) == HIGH){
    encCount++;
  }
  else{
    encCount--;
  }
}
double wheelRadius = 3.125;
double widthBetweenWheels = 17.78;
double ultrasonicToWheelLength = 21;
void forward_enc(int enc){
  encCount=0;
  while(encCount >= -enc){
    ////Serial.println(encCount);
    motor1.run(75); 
    motor2.run(-75);
  }
  motor1.stop();
  motor2.stop();
}
double cmToEnc(double cm){
  return 180*cm/(PI*wheelRadius);
}
double encToCm(int enc){
  return (enc*PI*wheelRadius)/180;
}
void forward_cm(double cm, double speed){
  encCount=0;
  int encTarget=-cmToEnc(cm);
  while(encCount>=encTarget){
    ////Serial.println(encCount);
    leftMotorSpeed(speed);
    rightMotorSpeed(speed);
  }
  motor1.stop();
  motor2.stop();
}


void forward(int n){
  motor1.run(n);
  motor2.run(-n);
}

void backward(int n){
  motor1.run(-n);
  motor2.run(n);
}

void basic_turn(int n) {
  motor1.run(n);
  motor2.run(n);
}

void stop_motors() {
  motor1.run(0);
  motor2.run(0);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void getBNO() {
  tcaselect(0);
  rot = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

void leftMotorSpeed(double motorSpeed){
  motor1.run(motorSpeed);
}
void rightMotorSpeed(double motorSpeed){
  motor2.run(-motorSpeed);
}

int get_rot(){
  getBNO();
  return rot.x();
}
int get_rotY(){
  getBNO();
  return rot.y();
}

void turn_to(int direction) {
  tcaselect(2);
  const int exSpeed = 90;
  getBNO();

  // calculate, well, difference from the intended rotation
  int difference = rot.x() - direction;
  if (difference  < -180) difference += 360;
  if (difference > 180) difference -= 360;
  if (abs(difference) <= 1) return;

  // figure out the speed
  int speed = map(difference, -180, 180, exSpeed, -exSpeed) + (difference > 0 ? -minSpeed : minSpeed);

  ////Serial.println(speed);
  if(speed < 0){
    speed -= 60;
  }
  if(speed > 0){
    speed += 60;
  }
  basic_turn(speed); 
  turn_to(direction);
}

void setupBNO() {
  tcaselect(0);
  ////Serial.println("Initializing BNO055");
  if(!bno.begin(0x08)) { // turn off the magnetometer 
    ////Serial.println("you're dumb theres no BNO055 here :( ");
    while(1);
  }
  bno.setExtCrystalUse(true);
  ////Serial.println("BNO055 initialized");
}

void resetSerial(){
  Serial.end(); //Reset the Serial Buffer!
  Serial.begin(115200);
  ////Serial.println("\n\n\n");
}

int num = 0;
String inString = "";    // string to hold input

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  attachInterrupt(digitalPinToInterrupt(18), Interruptfunc, RISING);
//  pinMode(pin, INPUT); //Front ultrasonic pin
//  attachInterrupt(digitalPinToInterrupt(pin), interfunc, CHANGE); //Interrupts instead of pulseIn
  rClaw.attach(9); lClaw.attach(3);
  rMount.attach(49); lMount.attach(6);
  rMount.write(0); lMount.write(0);
  rClaw.write(0); lClaw.write(20);
  raiseMount();
  grab();
  lMount.detach();
  rClaw.detach();
  lClaw.detach();
  //releaseC();
  setupBNO();
  qtrBegin();
}

double err[4] = {0}, comberr, preverr=0;
double speeds[2]={0};

//void qtrTrace(){
//  preverr=comberr;
//  qtrSensor.read(qtrSensorValues);
//
//  for(int i=0;i<4;i++)
//    err[i]=(double)qtrSensorValues[i]-(double)qtrSensorValues[7-i];
//err[0]-=EYE_ONE_EIGHT_ERROR;
//err[1]-=EYE_TWO_SEVEN_ERROR;
//err[2]-=EYE_THREE_SIX_ERROR;
//err[3]-=EYE_FOUR_FIVE_ERROR;
//
//comberr=err[0]*6+err[1]*2+err[2]*1.2+err[3];
//speeds[RIGHT_MOTOR]=constrain(-80-(comberr*KP)+(KD*(comberr-preverr)), -160, 160);
//speeds[LEFT_MOTOR]=constrain(80-(comberr*KP)+(KD*(comberr-preverr)), -160, 160);
//
//motor2.run(speeds[RIGHT_MOTOR]);
//motor1.run(speeds[LEFT_MOTOR]);
////  for(int i=0;i<4;i++){
////    //Serial.print(i);
////    //Serial.print(": ");
////    //Serial.print(err[i]);
////    //Serial.print(" ");
////  }
////  //Serial.println();
//}
double sumOfArray(double arr[],int n){
  double sum=0;
  for(int i=0;i<n;i++){
    sum+=arr[i];
  }
  return sum;
}
bool startedLine=false;
int ignore=0;
void qtrTrace(int baseTraceSpeed, bool fakeInter){
  double err[4] = {0}, comberr, preverr=0;
  double speeds[2]={0};
  preverr=comberr;
  qtrSensor.read(qtrSensorValues);

  for(int i=0;i<4;i++)
    err[i]=(double)qtrSensorValues[i]-(double)qtrSensorValues[7-i];
  err[0]-=EYE_ONE_EIGHT_ERROR;
  err[1]-=EYE_TWO_SEVEN_ERROR;
  err[2]-=EYE_THREE_SIX_ERROR;
  err[3]-=EYE_FOUR_FIVE_ERROR;
  if(!onLine()&&get_rotY()<10){
    if(!startedLine){
      //Serial.println("Off Line");
      if(nearestNinety(get_rot())-get_rot()>20||nearestNinety(get_rot())-get_rot()<-20){
        forward(65); delay(600);
      }
      stop_motors(); delay(200);
      turn_to(nearestNinety(get_rot()));
      stop_motors(); delay(200);
      startedLine=true;
    }
    rightMotorSpeed(baseTraceSpeed+20);
    leftMotorSpeed(baseTraceSpeed+20);
    return;
  }
  if(ignore<=1&&amtOnLine(5)&&fakeInter){
    forward(60); delay(500);
    stop_motors(); delay(200);
    if(!sensorNumOnLine(5)&& !sensorNumOnLine(4)){
      backward(60); delay(750);
      stop_motors(); delay(200);
      ignore=300;
    }
  }
  startedLine=false;
  comberr=err[0]*5+err[1]*1.5+err[2]*1+err[3]*0.8;
  speeds[RIGHT_MOTOR]=constrain(baseTraceSpeed+(comberr*KP)+(KD*(comberr-preverr)), -180, 150);
  speeds[LEFT_MOTOR]=constrain(baseTraceSpeed-(comberr*KP)+(KD*(comberr-preverr)), -180, 150);
//  if(speeds[RIGHT_MOTOR]<0) {
//    speeds[RIGHT_MOTOR]-=comberr*0.01;
//  }
//  else if(speeds[LEFT_MOTOR]<0) {
//    speeds[LEFT_MOTOR]-=comberr*0.01;
//  }
  if(get_rotY()>10){
    speeds[LEFT_MOTOR]+=50;
    speeds[RIGHT_MOTOR]+=50;
  }
  rightMotorSpeed(speeds[LEFT_MOTOR]);
  leftMotorSpeed(speeds[RIGHT_MOTOR]);
  
  if(ignore>0)ignore--;
}

int nearestNinety(int degree){
  int arr[5] = {degree, abs(degree - 90), abs(degree - 180), abs(degree - 270), abs(degree - 360)};
  int minD = degree, minI = 0;
  for(int i = 0; i < 5; i++){
    if(minD > arr[i]){
      minD = arr[i]; minI = i;
    }
  }
  return minI * 90;
}


void buttonPinInterrupt(){
  ////Serial.println("IN");
  if(digitalRead(pin) == HIGH){
    //Serial.println("HIGH");
    pulseInTimeBegin = micros();
  }
  else {
    // stop measuring
    //Serial.println("LOW");
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

unsigned long anVolt, cm;

int readSensor(){
  anVolt = analogRead(pin);
  cm = anVolt / 2 - 30;
  //Serial.println(cm);
  return cm;
}

int isObstacle2(){ //Check side ultrasonic
  anVolt = analogRead(pin2);
  cm = anVolt / 2 - 30;
  //Serial.print("US2: "); //Serial.print(cm);
  return cm;
}

volatile unsigned long lastPulse;

void avoid(){
  if(readSensor() < 5){
    int dist=readSensor();
    //Serial.println("IN AVOID");
    if(isObstacle2() < 15){ circleAround(dist, -1); }
    else{ circleAround(dist, 1); }
    encCount=0;
    while(!amtOnLine(1)){
      motor1.run(-60);
      motor2.run(60);
    }
  }
}
bool meetsTarget(double curr,double target, double err){
  if(curr>=target-err&&curr<=target+err) return true;
  return false;
}
void circleAround(int dist, int check){
  
  //Serial.println("IN AROUND");
  if(check == 1){
    turn_to(nearestNinety(get_rot())-90);
  }
  else{
    turn_to(nearestNinety(get_rot())+90);
  }
  stop_motors(); delay(1000);
  double rightSpeed = 0.0; double leftSpeed = 0.0;
  if(check == 1){
    rightSpeed = 90;
    leftSpeed = rightSpeed*(dist+((ultrasonicToWheelLength+widthBetweenWheels)/2))/(dist+((ultrasonicToWheelLength-widthBetweenWheels)/2));
  }
  else{
    leftSpeed = 90;
    rightSpeed = leftSpeed*(dist+((ultrasonicToWheelLength+widthBetweenWheels)/2))/(dist+((ultrasonicToWheelLength-widthBetweenWheels)/2));
  }
  if(leftSpeed<60){
    rightSpeed=(rightSpeed/leftSpeed)*70;
    leftSpeed=(leftSpeed/leftSpeed)*70;
  }
  else if(rightSpeed<60){
    leftSpeed=(leftSpeed/rightSpeed)*70;
    rightSpeed=(rightSpeed/rightSpeed)*70;
  }
  else if(leftSpeed>180){    
    rightSpeed=(rightSpeed/leftSpeed)*180;
    leftSpeed=(leftSpeed/leftSpeed)*180;
  }
  else if(rightSpeed>180){    
    leftSpeed=(leftSpeed/rightSpeed)*180;
    rightSpeed=(rightSpeed/rightSpeed)*180;
  }
  
  ////Serial.println(leftSpeed);
  ////Serial.println(rightSpeed);
  
  bool foundLine=false;
  for(;true;){
    turn_to(nearestNinety(get_rot()));
  double targetAngle = (get_rot()+(90*check))%360;
  if(targetAngle<0) targetAngle+=360;
    for(;true;){
      if(meetsTarget(get_rot(),targetAngle,1)) break;
      if(check == 1){
        leftMotorSpeed(leftSpeed);
        rightMotorSpeed(rightSpeed);
      }
      if(check == -1){
        leftMotorSpeed(leftSpeed - 30);
        rightMotorSpeed(rightSpeed);
      }
    }
    stop_motors(); delay(400);
    for(int i=0;i<300;i++){
      backward(80);
      if(amtOnLine(5)){
        foundLine=true;
        break;
      }
    }
    stop_motors(); delay(400);
    if(foundLine) break;
    forward(80); delay(1000);
  }
  stop_motors(); delay(1000);
  //backward(60); delay(500);
  //stop_motors(); delay(1000);
  forward(60); delay(300);
  turn_to(nearestNinety(get_rot())-(90*check));
  stop_motors(); delay(1000);
//  backward(60); delay(500);
//  stop_motors(); delay(1000);
  resetSerial();
}
bool validGreen(char ch){
  if(ch>='0'&&ch<='3') return true;
  return false;
}
int iter = 0;

void rightTurn(){
  if(amtOnLine(5)){
    stop_motors(); delay(200);
    resetSerial();
    turn_to(nearestNinety(get_rot())); stop_motors(); delay(200);
    //Serial.println("I");
    String temp = Serial.readStringUntil('\n');
    if(temp == "l" || (sensorNumOnLine(2) && sensorNumOnLine(7))){
      return;
    }
    else if(sensorNumOnLine(2)){ //Right
      forward(60); delay(1500);
      turn_to(nearestNinety(get_rot()) + 90); stop_motors(); delay(100);
    }
    else if(sensorNumOnLine(7)){ //Left
      forward(60); delay(1500);
      turn_to(nearestNinety(get_rot()) - 90); stop_motors(); delay(100);
    }
    backward(50); delay(500); stop_motors(); delay(100);
    while(!amtOnLine(3)){
      backward(40);
    }
    stop_motors(); delay(100);
    return;
    /*for(int i=0;i<8;i++){
      //Serial.print(qtrSensorValues[i]); //Serial.print(" ");
    }
    //Serial.println();*/
//    int sens = whichSensorsOnLine();
    ////Serial.println(sens); 
//    int temp=0;
//    if(sensorNumOnLine(2)){//right
//      while(sensorNumOnLine(4)){
//        if(temp>=500) return;
//        forward(60);
//        temp++;
//      }
//      forward(50); delay(1000);
//      turn_to(nearestNinety(get_rot()+90));
//    }
//    else if(sensorNumOnLine(7)){//left
//      while(sensorNumOnLine(5)){
//        if(temp>=500) return;
//        forward(60);
//        temp++;
//      }
//      forward(50); delay(1000);
//      turn_to(nearestNinety(get_rot()-90));
//    }
//    else return;
//    while(!amtOnLine(4)){
//                backward(60);
//              }
//              qtrTrace(60,false); delay(200);
//    forward(75); delay(950);


//    if(sens == 0){
//      forward(50); delay(100);
//      return;
//    }
//    turn_to(nearestNinety(get_rot()) + (90 * sens)); stop_motors(); delay(100);
//    backward(75); delay(500);
//    while(!amtOnLine(3)){
//      backward(50);
//    }
//    /*if(sens == 1){
//      while(!sensorNumOnLine(4)){
//          motor1.run(-20);
//          motor2.run(-80);
//      }
//    }
//    else if(sens == -1){
//      while(!sensorNumOnLine(5)){
//        motor1.run(80);
//        motor2.run(20);
//      }
//    }*/
//    //backward(50); delay(1100);
//    stop_motors(); delay(1000);
//    forward(50); delay(200);
//    resetSerial();
  }
}

bool checkForNoGreen(){
  if(Serial.available() > 0){
    inString = Serial.readStringUntil('\n');
    if(inString[0] == 'n'){ return true; }
  }
  return false;
}

bool checkForGreen(){
  inString = Serial.readStringUntil('\n');
  if(inString[0] == 'g'){ return true; }
  return false;
}
double target90=0;

bool first = true; //For evac

void pickUpBallSequence(){
  stop_motors(); delay(1000);
  motor1.run(-50);
  motor2.run(50); 
  delay(750);
  turn_to(get_rot() + 180); stop_motors(); delay(1000);
  lowerMount();
  motor1.run(-50);
  motor2.run(50); 
  delay(1500); 
  stop_motors(); delay(5);
  grab();
  raiseMount();
  turn_to(get_rot() - 180); stop_motors(); delay(1000);
  first = false;
}

boolean inSquare = false;
int lastSquare = 0; //Tracks the last square case we had
void squareCase(int turnA){
  forward(50); delay(800);
  if(turnA == 2){ forward(50); delay(750); turn_to(nearestNinety(get_rot()) + 90); }
  else if(turnA == 1){ turn_to(nearestNinety(get_rot()) - 90); }
  stop_motors(); delay(1000);
  forward(40); delay(500); stop_motors(); delay(500);
  while(!amtOnLine(4)){
    motor1.run(-50);
    motor2.run(50);
  }
  stop_motors(); delay(500);
  motor1.run(-75);
  motor2.run(75);
  delay(200);
  turn_to(nearestNinety(get_rot())); stop_motors(); delay(500);
  lastSquare = 300;
}

bool haveBall = false;
bool cornerSeen(){
  Serial.println("C"); delay(5000);
  inString = Serial.readStringUntil('\n');
  if(inString[0] == 'g'){ return true; }
  return false;
}

int gCorner, gAngle, gSide;
void findCorners(int corner, int t){
  turn_to(t); stop_motors(); delay(500);
  while(readSensor() > 2 && readSensor() < 40000){
    forward(50);
  }
  stop_motors(); delay(1000);
  if(cornerSeen()){ 
    rClaw.attach(9); lClaw.attach(3);
    rMount.attach(49); lMount.attach(6);
    gCorner = corner; gAngle = get_rot();
    if(isObstacle2() < 7){ gSide = -1; }
    else{ gSide = 1; }
    stop_motors(); delay(1000); first = true;
    backward(50); delay(500);
    turn_to(nearestNinety(get_rot() + 180)); stop_motors(); delay(500);
    lowerMount(); releaseC(); raiseMount(); stop_motors(); delay(6000);
    turn_to(nearestNinety(get_rot() - 180)); stop_motors(); delay(500); 
    while(readSensor() > 2 && readSensor() < 40000){
      forward(50);
    }
    stop_motors(); delay(1000);
    findBall();
    return;
  }
  else{
    findCorners(corner+1, nearestNinety(get_rot()) - 90);
  }
}

bool ballFound = false;
int align = 0;
void findBall(){
  turn_to(gAngle - (90 * gSide)); stop_motors(); delay(500);
  resetSerial();
  int mult = 1;
  while(!ballFound || readSensor() > 1 && readSensor() < 40000){
    forward(40);
    inString = Serial.readStringUntil('\n');
    if(inString[0] == 'v' && first){
      stop_motors(); delay(2500);
      first = false;
    }
    int tjg = 0;
    while(inString[0] == 'v' && inString[1] == 'g'){
      if(tjg > 3){
        pickUpBallSequence(); haveBall = true; returnToZone();
      }
      stop_motors(); delay(15);
      tjg++;
      inString = Serial.readStringUntil('\n');
    }
    while(inString[0] == 'v' && inString[1] == 'f'){
      if(readSensor() < 2 || readSensor() > 40000){
        turn_to(nearestNinety(get_rot()) - (90 * gSide * mult)); stop_motors(); delay(500); 
        resetSerial();
        inString = Serial.readStringUntil('\n');
        if(inString[1] != 'f'){ break; }
        forward(60); delay(1250);
        turn_to(nearestNinety(get_rot()) - (90 * gSide * mult)); stop_motors(); delay(500);
        mult *= -1;
      }
      align++;
      if(align == 50){
        turn_to(nearestNinety(get_rot())); stop_motors(); delay(200);
        align = 0;
      }
      motor1.run(40);
      motor2.run(-40);
      delay(5);
      inString = Serial.readStringUntil('\n');
    }
    if(inString[0] == 'v' && inString[1] == 't'){
      backward(60); delay(100);
      motor1.run(-65);
      motor2.run(-65);
      delay(50);
      resetSerial();
    }
    if(inString[0] == 'v' && inString[1] == 'y'){
      backward(60); delay(100);
      motor1.run(65);
      motor2.run(65);
      delay(50);
      resetSerial();
    }
    while(inString[0] == 'v' && inString[1] == 'b'){
      motor1.run(-26);
      motor2.run(26);
      delay(5);
      inString = Serial.readStringUntil('\n');
    }
    while(inString[0] == 'v' && inString[1] == 'l'){
      motor1.run(60);
      motor2.run(60);
      delay(5);
      inString = Serial.readStringUntil('\n');
    }
    while(inString[0] == 'v' && inString[1] == 'r'){
      motor1.run(-60);
      motor2.run(-60);
      delay(5);
      inString = Serial.readStringUntil('\n');
    }
    resetSerial();
  }
}

void returnToZone(){
  turn_to(gAngle); stop_motors(); delay(500);
  while(readSensor() > 2 && readSensor() < 40000){
    forward(50);
  }
  turn_to(gAngle + (90 * gSide)); stop_motors(); delay(500);
  while(readSensor() > 2 && readSensor() < 40000){
    forward(50);
  }
  forward(30); delay(600);
  stop_motors(); delay(500);
  turn_to(gAngle + 315); stop_motors(); delay(500);
  forward(50); delay(500);
  stop_motors(); delay(10);
  lowerMount();
  releaseC();
  raiseMount();
  stop_motors(); delay(1000);
  turn_to(gAngle); stop_motors(); delay(500);
  while(readSensor() > 2 && readSensor() < 40000){
    forward(50);
  }
  stop_motors(); delay(500);
  findBall();
}


boolean victimRoom = true;
int squareCaseTurn=3;
void loop() {
  //findCorners(1, 0);
  //stop_motors(); delay(10000);
  //resetSerial();
  /*qtrSensor.read(qtrSensorValues);
  for(int i=0;i<8;i++){
    //Serial.print(qtrSensorValues[i]); //Serial.print(" ");
  }
  //Serial.println();*/
  /*qtrSensor.read(qtrSensorValues);
  int left = 0, right = 0;
  for(int i = 0; i < 8; i++){
    if(i <= 1){ right+=qtrSensorValues[i]; }
    if(i >= 7){ left+=qtrSensorValues[i]; }
  }*/
  /*if(left >= right - 4000){
    //Serial.println("Left");
  }
  else if(right >= left - 3000){
    //Serial.println("Right");
  }
  else{
    //Serial.println("Moo");
  }*/
  ////Serial.print("L: "); //Serial.print(left/2); //Serial.print(" "); //Serial.print("R: "); //Serial.println(right/2);
  //Serial.println("G");
  get_rotY();
  qtrTrace(60,true);
  avoid();
//  qtrSensor.read(qtrSensorValues);
//  for(int i=0;i<8;i++){
//    //Serial.print(i);
//    //Serial.print(": ");
//    //Serial.println(qtrSensorValues[i]);
//  }

  
if (Serial.available() > 0) {
    num = 0;
    inString = Serial.readStringUntil('\n');
    if(inString[0] == 's' && inString[1] == 'i'){
      turn_to(nearestNinety(get_rot())); stop_motors(); delay(1000);
      forward(50); delay(1000);
      resetSerial();
      findCorners(1, 0);
    }
    //qtrTrace(50,true);
    while(inString[0] == 'g' && inString[1] == 'f'){
      //forward(40);
      qtrTrace(40,false);
      //delay(50);
      //resetSerial();
      inString = Serial.readStringUntil('\n');
    }
//    while(inString[0] == 'g' && inString[1] == 'b'){
//      backward(50);
//      //delay(50);
//      resetSerial();
//      inString = Serial.readStringUntil('\n');
//    }
    while(inString[0] == 'g' && inString[1] == 's'){
          //inString = Serial.readStringUntil('\n');
          //turn_to(nearestNinety(get_rot())); stop_motors(); delay(750);
          int turn = nearestNinety(get_rot());
          //turn_to(turn);
          //stop_motors(); delay(500);
          //if(checkForNoGreen){ break; }
            stop_motors(); delay(1000);
            resetSerial();
            float avg = 0; //Take the average of a few values from the picam to ensure you turn the RIGHT direction wink wink
            int timer=0;
            bool failed=false;
            bool did5=false;
            int doing5=0;
            turn_to(nearestNinety(get_rot()));
            for(int i=0;i<7;i++){
              inString = Serial.readStringUntil('\n');
              while(!validGreen(inString[2])){    
                if(inString[2]=='5'){
                  backward(40); 
                  did5=true;   
                  doing5++;        
                }
                else if(did5){
                  turn_to(nearestNinety(get_rot()));
                  did5=false;
                  doing5=0;
                }
                if(doing5>=300){
                  failed=true;
                  break;
                }
                inString = Serial.readStringUntil('\n');
                if(timer>=80){
                  failed=true; break;
                }
                if(timer>=20){
                  if(timer==20) {
                    forward(50); delay(500);
                    turn_to(nearestNinety(get_rot()));
                    backward(50); delay(500);
                    avg=0;
                    i=0;
                  }
                  backward(50);
                }
                timer++;
              }
              timer=0;
              stop_motors(); delay(500);
              if(failed) break;
              ////Serial.print("Value ");
              ////Serial.print(i);
              ////Serial.print(": ");
              ////Serial.println(inString[2]);
              
              avg+=inString[2]-'0';
              ////Serial.println(avg);
            }
            if(failed) break;
            avg /= 7;
            if(avg-(int)avg>=0.5) avg=(int)avg+1;
            else avg=(int)avg;
           // //Serial.print("Average: ");
            ////Serial.println(avg);
            ////Serial.print("Averages: ");
            ////Serial.println(avg);
            
            //forward(75); delay(2250);
            //stop_motors(); delay(1000);
            if(lastSquare > 0){
              lastSquare = 0;
              turn_to(nearestNinety(get_rot()));
              forward(50); delay(500);
            }
            while(!amtOnLine(5)){
                qtrTrace(50,false);
            }
            //stop_motors(); delay(500);
            forward(60); delay(500);
            stop_motors(); delay(1000);
            turn_to(turn);
            int turnDir=(int)avg;
            switch(turnDir){
              case 3:
                turn_to(turn + 190);
                break;
              case 2: //left
              //forward(75); delay(1250);
              ////Serial.println("TURNING LEFT");
//              turn_to(nearestNinety(get_rot()) - 90); stop_motors(); delay(100);
//              backward(50); delay(1500); stop_motors(); delay(100);
//              while(!amtOnLine(3)){
//                 backward(40);
//              }
              inSquare=false;
              forward(60); delay(350);
              stop_motors(); delay(100);
              ////Serial.println("Turning 2");
              target90=(nearestNinety(get_rot())-90);
              if(target90<0) target90+=360;
               motor1.run(-150);
               motor2.run(-150);
               delay(200);
               
              while(!meetsTarget(get_rot(),target90,1)){
                motor1.run(-150);
                motor2.run(-150);
                delay(5);
              }
              stop_motors(); delay(200);
              forward(60); delay(750);
              stop_motors(); delay(200);
              turn_to(nearestNinety(get_rot()));
              stop_motors(); delay(200);
              while(!amtOnLine(4)){
                motor1.run(-80);
                motor2.run(80);
                delay(5);
              }
              //forward(50); delay(200);
              squareCaseTurn=2;
                break;
              case 1: //right
              ////Serial.println("Turning 1");
              //forward(75); delay(1250);
              ////Serial.println("TURNING RIGHT");
//              turn_to(nearestNinety(get_rot()) + 90); stop_motors(); delay(100);
//              backward(50); delay(1500); stop_motors(); delay(100);
//              while(!amtOnLine(4)){
//                 backward(40);
//              }
              inSquare=false;
              //forward(60); delay(250);
              stop_motors(); delay(100);
              target90=(nearestNinety(get_rot())+90)%360;
               motor1.run(150);
               motor2.run(150);
               delay(200);
              while(!meetsTarget(get_rot(),target90,1)){
                motor1.run(150);//left
                motor2.run(150);
                delay(5);
              }
              stop_motors(); delay(200);
              forward(50); delay(500);
              stop_motors(); delay(200);
              turn_to(nearestNinety(get_rot()));
              stop_motors(); delay(200);
              while(!amtOnLine(4)){
                motor1.run(-80);
                motor2.run(80);
                delay(5);
              }
              //forward(50); delay(200);
              squareCaseTurn=1;
                break;
              default:
                if(inSquare){
                  turnDir = 4;
                  for(int i=0;!amtOnLine(3);i++){
                    qtrTrace(60,true);
                  }
                  turn_to(nearestNinety(get_rot())); stop_motors(); delay(500);
                  forward(50); delay(500);
                }
                else{
                turnDir = 4;
                qtrTrace(60, false); 
                delay(100);
                }
            }
//    motor1.run(-70);
//    motor2.run(150);
//    delay(500);
    Serial.println("D"); stop_motors(); delay(500); resetSerial();
    inString = Serial.readStringUntil('\n');
    if(inString[1] == 'A' && (turnDir != 3||inSquare)){
      inSquare = true;
      squareCase(squareCaseTurn);
    }
    else{
      forward(50); delay(250);
    }
    Serial.println("G");
    stop_motors(); delay(500);
//    while(!amtOnLine(4)){
//      backward(50);
//    }
    //backward(50); delay(500);
    break;
    }
    resetSerial(); 
  }
  lastSquare--;
}
