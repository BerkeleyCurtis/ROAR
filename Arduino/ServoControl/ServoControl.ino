/* Servo Control using serial inputs for ESP32-CAM
 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees from left = left, the middle of the range represents 
 * 90 degrees from left = straight, and the top of the range represents 180 degrees from left = right.
 * So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 90 left, 1500us would equal straight, and 2000us would equal 
 * 90 right. 
 * Circuit:
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. The ESP32-CAM runs on 3.3 volts and servos pull a lot of power at 5v, 
 * so connect the power and ground of the servo directly to the power and ground of your
 * 5v supply - probably your motor controller. There is no built in revers polarity protection
 * so if you do this wrong you will DESTROY YOUR SERVO 
 */

#include <ESP32Servo.h>

//-------------Motor control-------------------
Servo motorControl;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
String input_motor; // this gets the input from serial
int motorPower = 90;    // 90 = stopped, 0 = full reverse, 180 = full forward
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Don't use 4 - connected to LED
int motorOutputPin = 14;
int MAXMOTOR = 1700; //Should be as high as 2000 
int MINMOTOR = 1300; //Should be as low as 1000

//--------------setup servo for steering---------------
Servo steeringServo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
String input_steering; // this gets the input from serial
int steering = 90;    // Steering servo initialize going straight = 90 degrees
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Don't use 4 - connected to LED
int SteeringOutputPin = 2;
int MAX = 1700; //Should be as high as 2000 but my steering is a little broken
int MIN = 1000; //Should be as low as 1000

//--------------- incoming serial commands---------------

// int incoming = 0; // declare later
int servo = 1;
String input_value = "";

//-----------------Manual/Autonomous Mode--------------
const int buttonPin = 14;     // the number of the pushbutton pin
bool autonomous = 1;  // manual when 0, autonomous when 1. 
//This will be implimented later to switch between driving modes

//-------------Define variables 
const int LED = 4; // Built in LED
//int steering = 0; // input to set the value of steering
void setup() {
	// Allow allocation of all timers
  Serial.begin(115200);
  while (!Serial){}// wait for serial connection
  Serial.println("Serial Ready");
  Serial.println("Please enter a or b followed by a value 0 to 180");
  Serial.println("i.e. a150");
  Serial.println("This will set the steering to 150 = 60 degrees right");
  pinMode(buttonPin, INPUT); // Used to switch to autonomous mode
  pinMode(LED, OUTPUT); // Used for trouble-shooting, etc...
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	motorControl.setPeriodHertz(50);
  motorControl.attach(motorOutputPin, MINMOTOR, MAXMOTOR);
	steeringServo.setPeriodHertz(50);    // standard 50 hz servo
	steeringServo.attach(SteeringOutputPin, MIN, MAX); // attaches the servo on pin (whatever you assign)
	// to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
  
}

void loop() {

  //autonomous = digitalRead(buttonPin); // Not yet implimented
  while(Serial.available()>0){
    int incoming = Serial.read();
    if(incoming == 'a')
    {
      servo = 1; //steering
    }else if (incoming =='b'){
      servo = 2;
    }
    if(isDigit(incoming)) input_value += (char)incoming;
    if(incoming == '\n'){
      if(servo == 1) steering = input_value.toInt();
      if(servo == 2) motorPower = input_value.toInt();
   
      Serial.print("You entered ");
      Serial.print(input_value);
      Serial.print(" for servo ");
      Serial.println(servo);
      input_value = ""; // clear for next entry
    }
    
  }
   steeringServo.write(steering);    // tell servo to go to position 'steering'
   delay(10);  // give time 

   motorControl.write(motorPower);    // tell motor to drive with power = motorPower
   delay(10);  // give time 
   
   if (steering > 115 || steering < 65) {
     //checking if steering is reacting by turning LED On
     digitalWrite(LED, HIGH);
   }
   else digitalWrite(LED, LOW);
   

//-----------------------Sweeps servo from side to side----------- 
//	if(0) //  Change 0 to 1 test the servo with a repeating sweep
//	{
//    digitalWrite(LED, LOW);
//	  for (steering = 0; steering <= 180; steering += 1) { // goes from 0 degrees to 180 degrees
//	
//	  	// in steps of 1 degree
//		  steeringServo.write(steering);    // tell servo to go to position in variable 'pos'
//		  delay(10);             // waits 15ms for the servo to reach the position
//	  }
//	  for (steering = 180; steering >=0; steering -= 1) { // goes from 180 degrees to 0 degrees
//		  steeringServo.write(steering);    // tell servo to go to position in variable 'pos'
//		  delay(10);             // waits 15ms for the servo to reach the position
//	  }
//	}
// ----------------------------------------- 
 
}
