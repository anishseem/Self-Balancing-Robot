#include <Wire.h>  // Include I2C library for enabling communication with IMU to arduino

// Global Variables
// --------------- IMU Variables -----------------
const byte MPU_ADDR = 0x68;  // MPU6050 I2C address

float angleDeg = 0.0;       // pitch angle from accelerometer (deg)
float gyroDegPerSec = 0.0;  // rotation rate from gyro (deg/s)
float angleOffset = -2.35;   // calibrated accel tilt offset (deg)
float gyroOffset = 0.0;     // gyro zero-rate offset (deg/s units)
float accAngleRaw = 0.0;    // raw accel angle before subtracting offset, used for calibration. Originally -1.2

// --------------- Processing Visualization -----------------
float pitchDeg = 0.0;  //pitch
float rollDeg = 0.0;   //roll
float yawDeg = 0.0;    //yaw

// --------------- PID Gain Settings -----------------
float setAngle = 0.0;  //upright target angle
float kp = 1500.0;     //Proportional angle error reaction
float ki = 0.5;        //Integral long term error accumulation, drift avoidance
float kd = 75.0;       //Derivative reaction to angular velocity from gyro

float iError = 0.0;  // integral of angle error
float iMax = 5000.0;

float quietDeg = 0.6;  // if |angle error| < quietDeg, smeedCmd=0
float fallDeg = 45.0;  // if |angle| > fallDeg, robot has fallen

// --------------- Motor Pin Connections -----------------
const int L_STEP = 9;
const int L_DIR = 8;
const int R_STEP = 10;
const int R_DIR = 7;
const int EN_PIN = 6;  // shared enable, low is ACTIVE

// One common speed for both motors (steps/sec)
float speedCmd = 0.0;   // desired speed from PID output
float speedFilt = 0.0;  // filtered speed after speed/acceleration limiting

float maxSpeed = 7000.0;   // max speed in steps/sec
float maxAccel = 75000.0;  // max change in speed/s (steps/sec^2)

unsigned long lastIMUus = 0;    // last time we updated IMU
unsigned long lastStepLus = 0;  // last step time for left motor
unsigned long lastStepRus = 0;  // last step time for right motor
unsigned long stepIntUs = 0;    // time between steps in microseconds
int stepDirSign = 1;            // +1 forward, -1 backward

unsigned long lastDebugMs = 0;  // for Serial debug timing

// ----------------- Setup & Loop Functions -----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set motor pins state
  pinMode(L_STEP, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Enable drivers
  digitalWrite(EN_PIN, LOW);

  // Default outputs to low on startup
  digitalWrite(L_STEP, LOW);
  digitalWrite(R_STEP, LOW);
  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);

  // IMU init
  Wire.begin();
  Wire.setClock(400000); // Fast mode 400kHz I2C communication
  initIMU(); // Call IMU initialization function

  lastIMUus = micros();
}

void loop() {
  // Compute dt in seconds (time between IMU updates)
  long nowUs = micros();
  if (lastIMUus == 0) {
    lastIMUus = nowUs;
  }
  float dt = (nowUs - lastIMUus) / 1000000.0;
  if (dt <= 0.0) dt = 0.0005;
  lastIMUus = nowUs;

  // Read IMU and update angleDeg and gyroDegPerSec
  updateIMU(dt);

  // PID: convert angle into desired speed
  speedCmd = computeSpeed(dt);

  // Accel + speed limiting, update step interval and direction
  updateSpeedAndAccel(dt);

  // Step both motors to keep robot upright
  updateLeftMotor(nowUs);
  updateRightMotor(nowUs);

  /*
  //Send data to processing every 100ms ~10Hz
  unsigned long nowMs = millis();
  if(nowMs - lastDebugMs >= 100){
    lastDebugMs = nowMs;
    sendOrientationSerial();
  }
  */

  /*
  // Debug serial prints, also use accRaw for calibrating the upright angle position of the bot
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs >= 100) {
    lastDebugMs = nowMs;
    Serial.print("accRaw=");
    Serial.print(accAngleRaw);
    Serial.print("  angle=");
    Serial.print(angleDeg);
    Serial.print("  speedFilt=");
    Serial.println(speedFilt);
  }
  */
}
