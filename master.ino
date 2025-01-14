#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define trig 33
#define echo 32

#define port 12
#define starboard 19

#define RAD_TO_DEG (180/3.141592)

typedef struct {
  float x;
  float y;
  float z;
} Vector3;

Adafruit_MPU6050 mpu;
float distance;
Vector3 offset[2] = {{0, 0, 0}, {0, 0, 0}};

Vector3 gyro;
Vector3 angle;
Vector3 velocity;
Vector3 displacement;
Vector3 acceleration;

int servoStraight = 90;

void calibrate_mpu(unsigned long c) {
  unsigned int count = c;
  
  for (; c; c--) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    offset[0].x += a.acceleration.x;
    offset[0].y += a.acceleration.y;
    offset[0].z += a.acceleration.z;

    offset[1].x += g.gyro.x;
    offset[1].y += g.gyro.y;
    offset[1].z += g.gyro.z;

    delay(2);
  }

  offset[0].x /= count;
  offset[0].y /= count;
  offset[0].z /= count;
  offset[1].x /= count;
  offset[1].y /= count;
  offset[1].z /= count;
}

float integrate(unsigned int dt, float prev, float next) {
  return (prev * dt + 0.5 * (next - prev) * dt) * 0.000001;
}

void integrateV3(Vector3 *acc, unsigned int dt, Vector3 prev, Vector3 next, float conversion) {
  acc->x += integrate(dt, prev.x, next.x) * conversion;
  acc->y += integrate(dt, prev.y, next.y) * conversion;
  acc->z += integrate(dt, prev.z, next.z) * conversion;
}

void get_mpu() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acceleration.x = a.acceleration.x - offset[0].x;
  acceleration.y = a.acceleration.y - offset[0].y;
  acceleration.z = a.acceleration.z - offset[0].z;

  gyro.x = g.gyro.x - offset[1].x;
  gyro.y = g.gyro.y - offset[1].y;
  gyro.z = g.gyro.z - offset[1].z;

  // Integrate gyro to get angle

  static unsigned long previous = micros();
  unsigned long current = micros();

  unsigned long dt = current - previous;
  previous = current;

  static Vector3 prev_gyro;
  static Vector3 prev_accel;
  static Vector3 prev_velocity;

  integrateV3(&angle, dt, prev_gyro, gyro, RAD_TO_DEG);
  integrateV3(&velocity, dt, prev_accel, acceleration, 1);
  integrateV3(&displacement, dt, prev_velocity, velocity, 1);

  prev_gyro = gyro;
  prev_accel = acceleration;
  prev_velocity = velocity;
}

void write_port_led(float d) {
  int a = -0.283 * d + 283.3;

  if (a > 255)
    a = 255;
  else if (a < 0)
    a = 0;

  analogWrite(port, a);
}

void write_starboard_led(float b) {
  b = abs(b);
  
  int a = 2.833 * b;

  if (a > 255)
    a = 255;
  else if (a < 0)
    a = 0;

  analogWrite(starboard, a);
}

float measure_distance() {
  // Initialise pin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  // Trigger
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Wait until a high signal is recieved
  while (!digitalRead(echo)) {}

  // Get start time
  unsigned int time = micros();
  
  // Wait until a low signal is recieved
  while (digitalRead(echo)) {};
  
  // Calculate pulse width
  float time2 = micros() - time;

  // Calculate distance using the speed of sound
  return (time2 * 0.001 * 340)/2;
}

float measure_distance_accurate() {
  float sum = 0;
  
  for (int i = 2; i; i--) {
    sum += measure_distance();
    delay(10);
  }
  
  return sum / 2;
}

void setup() {
  Serial.begin(115200);  
  Serial.println("ESP32 Master Running");

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(port, OUTPUT);
  pinMode(starboard, OUTPUT);

  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("MPU6050 Found!");

  calibrate_mpu(1000);
}


void updateDirection(int servoAngle, int left_motor, int right_motor) {
  uint8_t b[5] = {servoAngle, left_motor > 0, right_motor > 0, abs(left_motor), abs(right_motor)};

  Wire.beginTransmission(0x08); // transmit to slave device address 8
  Wire.write(b, 5); // send five bytes, one for each character
  Wire.endTransmission();
}

void loop() {
  // put your main code here, to run repeatedly:
/*
  uint8_t b[5] = {90, 1, 1, 255, 255};

  Wire.beginTransmission(0x08); // transmit to slave device address 8
  Wire.write(b, 5); // send five bytes, one for each character
  Wire.endTransmission(); 
*/
  //Serial.printf("Pulse in:%f\n", measure_distance());


  //Serial.printf("AO: %f, %f, %f\n", offset[0].x, offset[0].y, offset[0].z);
  //Serial.printf("GO: %f, %f, %f\n", offset[1].x, offset[1].y, offset[1].z);

  //Serial.printf("A: %f %f %f\n", a.acceleration.x - offset[0].x, a.acceleration.y - offset[0].y, a.acceleration.z - offset[0].z);
  //Serial.printf("G: %f %f %f\n", g.gyro.x - offset[1].x, g.gyro.y - offset[1].y, g.gyro.z - offset[1].z);
  //Serial.printf("T: %f\n", temp.temperature);

  //Serial.println(a.acceleration.x - offset[0].x + a.acceleration.y - offset[0].y + a.acceleration.z - offset[0].z + g.gyro.x - offset[1].x + g.gyro.y - offset[1].y + g.gyro.z - offset[1].z);


  get_mpu();

  //Serial.printf("%f %d %f\n", angle.z, (int)angle.z % 90, measure_distance_accurate());

  write_starboard_led(angle.z);
  write_port_led(measure_distance());

  
  int quartile_direction = (int)angle.z % 90;
/*
  Serial.printf("%d %f\n", servoStraight, angle.z);

  if (quartile_direction < 45 && quartile_direction > 4) {
    servoStraight++;
  } else if (quartile_direction > 45 && quartile_direction < 86) {
    servoStraight--;
  if (quartile_direction < 45 && quartile_direction > 4) {
    servoStraight++;
  } else if (quartile_direction > 45 && quartile_direction < 86) {
    servoStraight--;
  }
  } else if (quartile_direction <= 5 || quartile_direction >= 85) {
    servoStraight = 90;
  }
*/

  float x = measure_distance();

  if (x <= 100.f) {
    updateDirection(90, 0, 0);
    delay(500);
    updateDirection(45, 0, 0);
    delay(500);
    updateDirection(0, -255, 255);
    float a = angle.z + 90;
    while (angle.z <= a) get_mpu();
    updateDirection(0, 0, 0);
    /*
    delay(1000);
    if (measure_distance() <= 150.f) {
      updateDirection(0, -100, 100);
      a = angle.z + 100;
      while (angle.z < a) get_mpu();
      updateDirection(0, 0, 0);
      delay(1000);
      if (measure_distance() <= 150.f) {
        updateDirection(0, 200, -200);
        a = angle.z - 90;
        while (angle.z > a) get_mpu();
        updateDirection(0, 0, 0);
        delay(1000);
      }
    }
    */
  } else if (x <= 200.f) {
    updateDirection(90, 255, 255);
  } else {
    updateDirection(servoStraight, 255, 255);
  }

  delay(100);
}
