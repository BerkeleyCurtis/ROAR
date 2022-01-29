#include <PID_v1.h>
#include <ESP32Servo.h>
#include <SerialCommand.h> // *** SERIAL COMUNICATION ***
//#include <movingAvg.h>
#define OVERRIDE_PIN 15
const int Hall = 13; // pin for hall effect sensor
const int Throt = 14; // throttle pin
unsigned int deltaTime = 0;
double distofRotation = 0.079; // distance traveled by Traxxas 3.0 with 1 rotation of the driveshaft
bool newValue = 0;

//movingAvg speedSensor(10);
Servo throttle;

double targetSpeed = 0;
double temp_throttle = 1500;
double speed_mps = 0;
double Kp=50, Ki=30, Kd=.1;
PID speedPID(&speed_mps, &temp_throttle, &targetSpeed, Kp, Ki, Kd, DIRECT);
SerialCommand sCmd;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Serial Initialized");
  sCmd.addCommand("setspeed",setNewSpeed); // argument: brightness 0-255
  pinMode(Hall, INPUT_PULLUP);
  pinMode(OVERRIDE_PIN, OUTPUT);
  digitalWrite(OVERRIDE_PIN, HIGH);
  attachInterrupt(Hall,Rev_Interrupt,FALLING);
//  speedSensor.begin(); // moving average
  throttle.attach(Throt); 
  throttle.writeMicroseconds(1500);
  delay(1000);
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(2);
  speedPID.SetOutputLimits(1500, 1800);
}

void loop() {
  const unsigned int timeout_total = 1000;
  static unsigned long lastTime = 0;
  static unsigned int throttle_output;
  
  sCmd.readSerial();

  if(newValue){
    noInterrupts();
    float tempdeltaTime = deltaTime;
    interrupts();
    speed_mps = 1000 * distofRotation / tempdeltaTime;
    if(speed_mps < 0) speed_mps = 0;
    newValue = 0;
    lastTime = millis(); // for timeout
  }
  else{
    unsigned long timeNow = millis();
    if((timeNow - lastTime) > timeout_total){
      speed_mps = 0;
      lastTime = timeNow;
    }
  }

  speedPID.Compute();
  if(targetSpeed < 0.1) temp_throttle = 1500;
  if(temp_throttle > 1800) temp_throttle = 1800;
  if(temp_throttle < 1500) temp_throttle = 1500;  
  throttle_output = temp_throttle;
  throttle.writeMicroseconds(throttle_output);
  Serial.println(speed_mps);
  Serial.println(throttle_output);

}

void Rev_Interrupt (){
  unsigned long timeNow = millis();
  static unsigned long lastTime = 0;
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  newValue = 1;
}

void setNewSpeed(){
  char *arg;
  arg = sCmd.next();
  if (arg != NULL) {
    int tempSpeed = atoi(arg);
    targetSpeed = (double) tempSpeed / 100;
  }
}
