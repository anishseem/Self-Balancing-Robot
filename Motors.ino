// --------------- Motor Functions -----------------

// Update speedFilt, stepIntUs, and direction sign based on speedCmd
// I need to implement speed and acceleration control since my stepper motors will initially stalling without limits, 
// slightly hardware constrained based on stepper motors specs and robot being mass heavy at the head (results in greater rotational intertia but more greater torque/speed from motors)
void updateSpeedAndAccel(float dt) {
  // Limit speedCmd to maxSpeed for forward and backward
  if (speedCmd >  maxSpeed) speedCmd =  maxSpeed;
  if (speedCmd < -maxSpeed) speedCmd = -maxSpeed;

  // Limit acceleration to prevent stepper motors stalling
  float diff = speedCmd - speedFilt;
  float maxChange = maxAccel * dt;

  if (diff >  maxChange) diff =  maxChange;
  if (diff < -maxChange) diff = -maxChange;

  speedFilt += diff;

  // Update direction sign and step interval
  if (speedFilt == 0.0) {
    stepIntUs = 0;
  } else {
    // If speed is positive move forward, if its negative move backward
    if (speedFilt > 0.0) {
      stepDirSign = 1;
    } else {
      stepDirSign = -1;
    }
    //Convert speed in steps/sec to time/step in us
    float absSpeed = fabs(speedFilt);
    stepIntUs = (unsigned long)(1000000.0 / absSpeed);  // us per step
  }
}

// Step left motor if enough time has passed
void updateLeftMotor(unsigned long nowUs) {
  if (stepIntUs == 0) return; //Exit function if step inteveral is 0, not moving

  if (lastStepLus == 0) { //Reset lastStepus each time the function runs
    lastStepLus = nowUs;
  }
  //When the enough enough time has passed for at least 1 step, move the last step time forward by one interval
  while ((nowUs - lastStepLus) >= stepIntUs) {
    lastStepLus += stepIntUs;

    // Set Direction
    if (stepDirSign > 0) {
      digitalWrite(L_DIR, HIGH);
    } else {
      digitalWrite(L_DIR, LOW);
    }

    // Send one step pulse
    digitalWrite(L_STEP, HIGH);
    digitalWrite(L_STEP, LOW);
  }
}

// Step right motor if enough time has passed
void updateRightMotor(unsigned long nowUs) {
  if (stepIntUs == 0) return;

  if (lastStepRus == 0) {
    lastStepRus = nowUs;
  }

  while ((nowUs - lastStepRus) >= stepIntUs) {
    lastStepRus += stepIntUs;

    if (stepDirSign > 0) {
      digitalWrite(R_DIR, HIGH);
    } else {
      digitalWrite(R_DIR, LOW);
    }

    digitalWrite(R_STEP, HIGH);
    digitalWrite(R_STEP, LOW);
  }
}
