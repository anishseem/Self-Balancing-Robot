// --------------- IMU Settings -----------------
// I was able to read/write IMU data for this project using Jeff Rowberg's MPU6050 I2C code & tutorials from I2Cdevlib as a reference
// Register addresses, Full-Scale Range, and LSB sensistivity values taken from the MPU6050 datasheet

// Wake up MPU and measure/calibrate the gyro offset
void initIMU() {
  // Wake device: write 0 to power management register 0x6B
  Wire.beginTransmission(MPU_ADDR); //Initialize IMU communication
  Wire.write(0x6B); //Write to PWR_MGMT_1 Register
  Wire.write(0x00); //Write 0x00 to power mgmt register to wake up IMU
  Wire.endTransmission();
  delay(100);

  Serial.println("Calibrating gyro");

  long gxSum = 0;
  const int N = 1000;

  for (int i = 0; i < N; i++) {
    // Read GYRO_XOUT_H (0x43) and GYRO_XOUT_L (0x44)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // start at gyro X high byte
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (byte)2, true); // Send the 16-bit data values as 2 bytes

    //The IMU sensor writes data to two 8-bit registers, here the data is combined into 16-bit/2-byte values so it can be read.
    int16_t gxRaw = 0;
    if (Wire.available() >= 2) {
      byte hi = Wire.read();
      byte lo = Wire.read();
      gxRaw = (int16_t)((hi << 8) | lo);  // combine high and low bytes (high byte first)
    }

    gxSum += gxRaw;
    delay(2);
  }

  // Average raw gyro reading while still
  float gxAvg = (float)gxSum / (float)N;

  // Convert to FSR(250 deg/s) using the LSB sensitivity (131 counts/deg/s) 
  gyroOffset = gxAvg / 131.0;

  Serial.print("gyroOffset (deg/s) = ");
  Serial.println(gyroOffset);

  Serial.print("accel angle offset (deg) = ");
  Serial.println(angleOffset);
}

// Read accelerometer + gyro, update control angle and visualization angles
void updateIMU(float dt) {
  // Read full 14-byte sensor block starting at ACCEL_XOUT_H (0x3B) (I only actually use 12 bytes since the 2 bytes from the temperature sensor are useless for this application)
  Wire.beginTransmission(MPU_ADDR); //Begin transmission at MPU address
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (byte)14, true); // Send 14 bytes to read starting at register 0x3B

  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;
  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;

  //Reads 
  if (Wire.available() >= 14) {
    byte axHi = Wire.read(); // I read the byte data from each register address in consecutive order and associate a variable to it
    byte axLo = Wire.read(); // The accelerometer and gyro data has to be read in this order to properly associate the address registers to the right axis
    byte ayHi = Wire.read();
    byte ayLo = Wire.read();
    byte azHi = Wire.read();
    byte azLo = Wire.read();
    byte tHi = Wire.read(); // Bytes from the MPU6050 temperature sensor, not used but have to read in the register order for proper variable association
    byte tLo = Wire.read();
    byte gxHi = Wire.read();
    byte gxLo = Wire.read();
    byte gyHi = Wire.read();
    byte gyLo = Wire.read();
    byte gzHi = Wire.read();
    byte gzLo = Wire.read();

    axRaw = (int16_t)((axHi << 8) | axLo); //Here i stitch the high and low bytes together for each axis for both accel/gyro
    ayRaw = (int16_t)((ayHi << 8) | ayLo);
    azRaw = (int16_t)((azHi << 8) | azLo);
    gxRaw = (int16_t)((gxHi << 8) | gxLo);
    gyRaw = (int16_t)((gyHi << 8) | gyLo);
    gzRaw = (int16_t)((gzHi << 8) | gzLo);
  }

  // Convert accel counts to g (Using 2g FSR => 16384 LSB per g)
  float ax_g = axRaw / 16384.0;
  float ay_g = ayRaw / 16384.0;
  float az_g = azRaw / 16384.0;

  // ---- Angle used for control (same as before: YZ plane) ----
  float accAng = atan2(ay_g, az_g) * 180.0 / PI;
  accAngleRaw  = accAng;           // before subtracting offset
  angleDeg     = accAng - angleOffset;

  // Gyro X: convert raw to deg/s and subtract offset (used in PID)
  float gyroXdeg = (gxRaw / 131.0) - gyroOffset;
  gyroDegPerSec  = gyroXdeg;

  // Visualization angles for Processing
  // Side tilt (roll) from XZ plane
  float rollFromAccel = atan2(-ax_g, az_g) * 180.0 / PI;
  rollDeg = rollFromAccel;

  // Yaw (heading) from gyro Z integration (will drift over time)
  float gyroZdegPerSec = gzRaw / 131.0;   // no offset correction here
  yawDeg += gyroZdegPerSec * dt;

}
