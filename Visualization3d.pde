float pitchDeg = 0;
float rollDeg = 0;
float yawDeg = 0;

PShape sbr; //Datatype to store the 3d obj file

import processing.serial.*;
Serial myPort;

void setup() {
  size(800, 600, P3D); // Using the P3D rendering engine since im displaying an .obj file
  surface.setLocation(450, 150); //Open processing window in middle of my laptop screen
  // Open first serial port at 115200 baud
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200); //Open the serial port at the set baud rate in arduino
  myPort.bufferUntil('\n');

  // Load 3D .obj model which was converted from CAD .STEP file to MainAssembly.obj in data folder
  sbr = loadShape("MainAssembly.obj");

  // Flip model if it appears upside down
  sbr.rotateX(PI);

  // Scale model 50% larger since it was originally too small in the window  
  sbr.scale(1.5);
}

void draw() {
  background(0); //Black background
  lights();

  pushMatrix();  // Render sbr model in a 3d coordinate system i.e. seperate matrix stack

  // Move origin to middle of window
  translate(width/2, height/2, 0);

  // Apply yaw, roll, pitch (in this order)
  rotateY(radians(yawDeg));    // yaw   (turn left/right)
  rotateX(radians(rollDeg));   // roll  (side tilt)
  rotateZ(radians(pitchDeg));  // pitch (forward/back tilt)

  // Draw the model
  shape(sbr);
  
  popMatrix();   // Return to a 2d coordinate system to render the positional data

  // Display IMU data as text in blue at the top left corner
  fill(255);
  textSize(18);
  fill(0,0,255);
  text("Pitch: " + pitchDeg + " deg", 10, 20);
  text("Roll : " + rollDeg + " deg", 10, 40);
  text("Yaw  : " + yawDeg + " deg", 10, 60);

}

// Called automatically when a full line (\n) arrives on Serial
void serialEvent(Serial p) {
  String inString = p.readStringUntil('\n'); //Read values from serial port up to the new line, store the line as a string
  if (inString != null) {
    inString = trim(inString);
    String[] vals = split(inString, ','); //Splits the serial data string into pieces by comma

    // Checks for pitch, roll, yaw and saves the values to the defined variables
    if (vals.length == 3) {
      pitchDeg = float(vals[0]);
      rollDeg  = float(vals[1]);
      yawDeg   = float(vals[2]);
    }
  }
}
