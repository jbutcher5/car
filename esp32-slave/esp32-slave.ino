#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <Wire.h>

Servo steeringServo;

#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 12;
const int resolution = 8;

int steeringAngle = 90;    // variable to store the servo position
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32

void receive(int n) {
  int buf[5];

  for (int i = 0; Wire.available() && i < 5; i++)
    buf[i] = Wire.read();

  steeringServo.write(buf[0]);

  //Serial.printf("%d %d %d\n", buf[0], buf[1], buf[2]);

  // Karnaugh map to avoid branching

  digitalWrite(INa, buf[1]);
  digitalWrite(INb, !buf[1]);
  digitalWrite(INc, buf[2]);
  digitalWrite(INd, !buf[2]);

  ledcWrite(enA, buf[3]);
  ledcWrite(enB, buf[4]);
}

void setup() {
  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
  ledcAttachChannel(enA, freq, resolution, ledChannela);
  ledcAttachChannel(enB, freq, resolution, ledChannelb);

  // allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	steeringServo.setPeriodHertz(50);    // standard 50Hz servo
	steeringServo.attach(servoPin, 500, 2400);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  // initialise serial communication
  Serial.begin(115200);
  Serial.println("ESP32 Slave Running");  // sanity check

  Wire.begin(0x08);
  Wire.onReceive(receive);
}


// Rest is all project week 3

void loop() {
  //delay(100);
  // this code rotates the steering between 0 and 180 degrees before driving both wheels forwards and backwards followed by rotating the EEEBot clockwise and anticlockwise for 3 seconds each time

  //set the speed of the motors - minimum speed of 0, maximum speed of 255 i.e. largest value for an 8-bit PWM
/*
  int leftSpeed = 255;
  int rightSpeed = 255;


  moveSteering();
  delay(500);
  goForwards();
  motors(leftSpeed, rightSpeed);
  delay(3000);
  stopMotors();
  delay(500);
  goBackwards();
  delay(3000);
  stopMotors();
  delay(500);
  goClockwise();
  delay(3000);
  stopMotors();
  delay(500);
  goAntiClockwise();
  delay(3000);
  stopMotors();
  delay(500);
*/
}

void moveSteering() {
  steeringServo.write(0);

  for (steeringAngle = 0; steeringAngle <= 180; steeringAngle += 1) {   // goes from 0 degrees to 180 degrees in steps of 1 degree
		steeringServo.write(steeringAngle);                                 // tell servo to go to position in variable 'steeringAngle'
		delay(15);                                                          // waits 15ms for the servo to reach the position
	}
	for (steeringAngle = 180; steeringAngle >= 0; steeringAngle -= 1) {   // goes from 180 degrees to 0 degrees
		steeringServo.write(steeringAngle);                                 // tell servo to go to position in variable 'steeringAngle'
		delay(15);                                                          // waits 15ms for the servo to reach the position
	}
}

void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
  delay(25);
}