#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;
int ENA = 5;
int ENB = 6;

// MPU variables
float accAngle, gyroAngle, angle;
float gyroRate;
float dt;

// PID variables
float Kp = 20;
float Ki = 1;
float Kd = 0.15;

float error, previousError;
float integral, derivative;
float output;

// Timing
unsigned long prevTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  prevTime = millis();
}

void loop() {
  // Time calculation
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer angle
  accAngle = atan2(ay, az) * 180 / PI;

  // Gyro rate
  gyroRate = gx / 131.0;

  // Complementary filter
  angle = 0.95 * (angle + gyroRate * dt) + 0.04 * accAngle;

  // PID Control
  float setpoint = 0;  // upright position

  error = angle - setpoint;
  integral += error * dt;
  derivative = (error - previousError) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;

  // Motor control
  int speed = constrain(abs(output), 0, 255);

  if (output > 0) {
    moveForward(speed);
  } else {
    moveBackward(speed);
  }

  // Debug
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" Output: ");
  Serial.println(output);
}

// Motor functions
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}