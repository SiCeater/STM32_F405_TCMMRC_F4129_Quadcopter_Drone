import processing.serial.*;

Serial myPort;
String data = "";

float gx, gy, gz;
float ax, ay, az;
float axF, ayF, azF;
float roll, pitch, yaw;

float alpha = 0.08;
float droneW = 180;
float droneH = 20;
float droneD = 120;

void setup() {
  size(1280, 720, P3D);
  myPort = new Serial(this, "/dev/tty.usbmodem1103", 921600);
  myPort.bufferUntil('\n');
}

void draw() {
  background(255);
  lights();

  pitch = degrees(atan2(-axF, sqrt(ayF * ayF + azF * azF)));
  roll  = degrees(atan2(ayF, azF));
  yaw   = 0;

  camera();
  hint(ENABLE_DEPTH_TEST);

  pushMatrix();
  translate(width * 0.5, height * 0.58, 0);

  drawWorldAxes(220);
  drawAccelVector(axF, ayF, azF, 18);

  pushMatrix();
  rotateY(HALF_PI);
  rotateZ(radians(roll));
  rotateX(radians(pitch));
  drawDrone();
  drawBodyAxes(140);
  popMatrix();

  popMatrix();

  hint(DISABLE_DEPTH_TEST);
  camera();
  fill(0);
  textSize(22);
  text("A: " + nf(axF, 0, 2) + "  " + nf(ayF, 0, 2) + "  " + nf(azF, 0, 2), 40, 40);
  text("G: " + nf(gx, 0, 3) + "  " + nf(gy, 0, 3) + "  " + nf(gz, 0, 3), 40, 70);
  text("Roll: " + nf(roll, 0, 1) + "   Pitch: " + nf(pitch, 0, 1) + "   Yaw: " + nf(yaw, 0, 1), 40, 100);
}

void drawDrone() {
  stroke(0);
  fill(160);
  box(droneW, droneH, droneD);

  strokeWeight(4);
  line(-droneW * 0.5, 0, 0, droneW * 0.5, 0, 0);
  line(0, 0, -droneD * 0.5, 0, 0, droneD * 0.5);
  strokeWeight(1);

  float m = 14;
  pushMatrix(); translate(-droneW * 0.5, 0, -droneD * 0.5); sphere(m); popMatrix();
  pushMatrix(); translate( droneW * 0.5, 0, -droneD * 0.5); sphere(m); popMatrix();
  pushMatrix(); translate(-droneW * 0.5, 0,  droneD * 0.5); sphere(m); popMatrix();
  pushMatrix(); translate( droneW * 0.5, 0,  droneD * 0.5); sphere(m); popMatrix();
}

void drawWorldAxes(float L) {
  strokeWeight(3);

  stroke(255, 0, 0);
  line(0, 0, 0, L, 0, 0);

  stroke(0, 180, 0);
  line(0, 0, 0, 0, -L, 0);

  stroke(0, 90, 255);
  line(0, 0, 0, 0, 0, L);

  strokeWeight(1);
}

void drawBodyAxes(float L) {
  strokeWeight(4);

  stroke(255, 0, 0);
  line(0, 0, 0, L, 0, 0);

  stroke(0, 180, 0);
  line(0, 0, 0, 0, -L, 0);

  stroke(0, 90, 255);
  line(0, 0, 0, 0, 0, L);

  strokeWeight(1);
}

void drawAccelVector(float x, float y, float z, float scaleF) {
  stroke(20);
  strokeWeight(5);
  line(0, 0, 0, x * scaleF, -y * scaleF, z * scaleF);
  strokeWeight(1);
}

void serialEvent(Serial myPort) {
  data = myPort.readStringUntil('\n');

  if (data != null) {
    data = trim(data);
    data = data.replace("G:", "");
    data = data.replace("A:", "");
    String[] items = splitTokens(data, " ");

    if (items.length >= 6) {
      gx = float(items[0]);
      gy = float(items[1]);
      gz = float(items[2]);

      ax = float(items[3]);
      ay = float(items[4]);
      az = float(items[5]);

      axF = lerp(axF, ax, alpha);
      ayF = lerp(ayF, ay, alpha);
      azF = lerp(azF, az, alpha);
    }
  }
}
