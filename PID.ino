// --------------- PID Functions -----------------
float computeSpeed(float dt) {
  // If robot is too tilted, consider it fallen and stop
  if (fabs(angleDeg) > fallDeg) {
    iError = 0.0;
    return 0.0;
  }

  // Angle error (measured - target)
  float err = angleDeg - setAngle;

  // Quiet zone: don't move if very close to upright
  if (fabs(err) < quietDeg) {
    iError = 0.0;
    return 0.0;
  }

  // Proportional term: Correction proportional to angle error
  float p = kp * err;

  // Integral term: Correction is based on accumlated error over time, reduces steady drift in either direction
  iError += err * dt;
  //Prevent integral of error from growing beyond max setting. Results in large overshooting without it,.
  if (iError >  iMax) iError =  iMax;
  if (iError < -iMax) iError = -iMax;
  float i = ki * iError;

  // Derivative Term: Correction based on how fast pitch is changing
  float d = kd * gyroDegPerSec;

  // Final speed command
  float cmd = -(p + i + d);
  return cmd;   // in steps/sec before applying accel/speed limits
}
