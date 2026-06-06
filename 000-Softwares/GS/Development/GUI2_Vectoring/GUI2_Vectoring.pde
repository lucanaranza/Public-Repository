/*GRAPHIC USER INTERFACE - VECTORING*/
import java.time.*;
import java.time.format.*;
import java.util.*;
import processing.serial.*;
import grafica.*;
import processing.video.*;

PFont font, titleFont, nameFont, mediumFont, smallFont;
PShape rocketModel;
PImage vectoring_text, lnspace_logo, MSRO_logo;
PGraphics pg3D;
GPlot alPlot, vzPlot, PPlot, s1_posPlot;

Serial myPort;
Capture cam;

/*data set scheme and names from VScode*/
float ts=0.0, ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0, p=0.0, y=0.0, r=0.0, vx=0.0, vy=0.0, vz=0.0, al=0.0, tp=0.0, bv=0.0, s1_pos=0.0, s2_pos=0.0, s3_pos=0.0, s4_pos=0.0, s5_pos=0.0, s6_pos=0.0, mx=0.0, my=0.0, mz=0.0, lat=0.0, lng=0.0, al_gps=0.0, ve_gps=0.0;
float om=0;
float ts_on=0.0;
float s1_set=0.0;
float s2_set=0.0;
float s3_set=0.0;
float s4_set=0.0;
float w_spd=0.0;
float w_dir=0.0;
float hum=0.0;
float prs=0.0;
float r_set=0.0;
float gz_set=0.0;
float s1P_pos=0.0;
float s2P_pos=0.0;
float hedng=0.0;
float hedng_set=0.0;
float vz_set=0.0;
float minLat = Float.MAX_VALUE;
float maxLat = -Float.MAX_VALUE;
float minLng = Float.MAX_VALUE;
float maxLng = -Float.MAX_VALUE;
float minAlt = Float.MAX_VALUE;
float maxAlt = -Float.MAX_VALUE;
float tp_ground=0.0;
float z=0.0;
int gpssat=2;
int rssi=98;
boolean s1_status = false;
boolean s2_status = false;
/*-------------------*/


/*----------auxiliary variables----------*/
float ac=0.0, ve=0.0;
float box_color = 50;
float text_color1 = 0; // black
float text_color2 = 250; //white
float bg_color = 250;

String title = "VICTORY III";
String currentTimeStr;
String rawData;
float PO_time = 0.0;
float LO_time = 0.0;

int graphX = 50, graphY = 50, graphW = 400, graphH = 400;
float maxX = 100;
float maxY = 100;

float px = 0.0;
float py = 0.0;

/*----------VEICHLE STATUS----------*/
int arm_status = 0;
int launch_status = 0;
int recovery_status = 0;

final int WIDTH = 1920;
final int HEIGHT = 1080;

/*----------DATA SETTINGS----------*/
String[] labels = {
  "om ", "ts ", "ax ", "ay ", "az ",
  "gx ", "gy ", "gz ", "p ", "y", "r ",
  "vx ", "vy ", "vz ", "al ", "tp ",
  "bv ", "s1_pos ", "mx ", "my ", "mz ",
  "lat ", "lng ", "al_gps", "ve_gps"
};
String[] data = new String[labels.length];

int graphLength = 300;
ArrayList<Float> omData = new ArrayList<Float>();
ArrayList<Float> tsData = new ArrayList<Float>();
ArrayList<Float> axData = new ArrayList<Float>();
ArrayList<Float> ayData = new ArrayList<Float>();
ArrayList<Float> azData = new ArrayList<Float>();
ArrayList<Float> gxData = new ArrayList<Float>();
ArrayList<Float> gyData = new ArrayList<Float>();
ArrayList<Float> gzData = new ArrayList<Float>();
ArrayList<Float> pData = new ArrayList<Float>();
ArrayList<Float> yData = new ArrayList<Float>();
ArrayList<Float> rData = new ArrayList<Float>();
ArrayList<Float> vxData = new ArrayList<Float>();
ArrayList<Float> vyData = new ArrayList<Float>();
ArrayList<Float> vzData = new ArrayList<Float>();
ArrayList<Float> alData = new ArrayList<Float>();
ArrayList<Float> tpData = new ArrayList<Float>();
ArrayList<Float> bvData = new ArrayList<Float>();
ArrayList<Float> s1_posData = new ArrayList<Float>();
ArrayList<Float> mxData = new ArrayList<Float>();
ArrayList<Float> myData = new ArrayList<Float>();
ArrayList<Float> mzData = new ArrayList<Float>();
ArrayList<Float> latData = new ArrayList<Float>();
ArrayList<Float> lngData = new ArrayList<Float>();
ArrayList<Float> al_gpsData = new ArrayList<Float>();
ArrayList<Float> ve_gpsData = new ArrayList<Float>();
ArrayList<PVector> gpsPoints = new ArrayList<PVector>();

float[] alHistory = new float[100];
float[] vzHistory = new float[100];
float[] PHistory = new float[100];
float[] s1_posHistory = new float[100];


void settings() {
  size(WIDTH, HEIGHT, P3D);
}

void setup() {
  surface.setLocation(0, 0);
  pg3D = createGraphics(300, 400, P3D); // size of the 3D panel
  
  titleFont = createFont("Arial Bold", 28);
  nameFont = createFont("marlin", 24);
  font = createFont("marlin", 18);
  mediumFont = createFont("marlin", 14);
  smallFont = createFont("marlin", 11);

  vectoring_text = loadImage("vectoring_logo.png");
  MSRO_logo = loadImage("MSRO_logo.png");
  lnspace_logo = loadImage("lnspace_logo.jpg");
  
  rocketModel = loadShape("rocket.obj");
  rocketModel.disableStyle();
  
  printArray(Serial.list());
  String serialPortName = Serial.list()[0];
  myPort = new Serial(this, serialPortName, 115200);
  myPort.bufferUntil('\n');
  
  // Setup altitude plot
  alPlot = new GPlot(this);
  alPlot.setPos(320, 308);
  alPlot.setDim(370, 150);
  alPlot.setTitleText("Altitude");
  alPlot.getXAxis().setAxisLabelText("Time");
  alPlot.getYAxis().setAxisLabelText("Altitude [m]");
  
  // Setup velocity plot
  vzPlot = new GPlot(this);
  vzPlot.setPos(320, 570);
  vzPlot.setDim(370, 150);
  vzPlot.setTitleText("Velocity");
  vzPlot.getXAxis().setAxisLabelText("Time");
  vzPlot.getYAxis().setAxisLabelText("Velocity [m/s]");
  
  // Setup PYR plot
  PPlot = new GPlot(this);
  PPlot.setPos(750, 308);
  PPlot.setDim(370, 150);
  PPlot.setTitleText("P Y R");
  PPlot.getXAxis().setAxisLabelText("Time");
  PPlot.getYAxis().setAxisLabelText("Angles [°]");
  
  // Setup Servo plot
  s1_posPlot = new GPlot(this);
  s1_posPlot.setPos(750, 570);
  s1_posPlot.setDim(370, 150);
  s1_posPlot.setTitleText("Servo");
  s1_posPlot.getXAxis().setAxisLabelText("Time");
  s1_posPlot.getYAxis().setAxisLabelText("Servo [°]");
  
  // Initialize data arrays
  for (int i = 0; i < alHistory.length; i++) {
    alHistory[i] = 0;
    vzHistory[i] = 0;
    PHistory[i] = 0;
    s1_posHistory[i] = 0;
  }

  String[] cameras = Capture.list();
  if (cameras == null || cameras.length == 0) {
    println("No cameras found!");
    exit();
  } else {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++) {
      println(i + ": " + cameras[i]);
    }

    cam = new Capture(this, cameras[0]); // use first camera
    cam.start();
  }
}

void draw() {
  background(bg_color);
  lights();
  
  textAlign(CENTER, TOP);
  textFont(titleFont);
  stroke(100);
  textFont(font);

  drawLogosAndHeader();
  drawAttitudeVisualizer();
  drawAltitudeVelocityAccBox();
  drawPYRBox();
  drawFinOrientationFigure();
  drawRollRateIndicator();
  drawFlightInfo();
  drawRollControlBox();
  drawParafoilControlBox();
  drawRawDataSection();
  drawVideoFeedBox();
  drawTelemetryBox();
  drawWeatherBox();
  drawTrajectory3DPlot();
  addNewGPSPoint();
  drawGroundTrackPlot();
  drawStatusBox();
  drawActionButtons();
  
  updateData();
  drawalGraph();
  drawvzGraph();
  drawPGraph();
  draws1_posGraph();
}

/*----------data INFO----------*/

// ----------------- GUI COMPONENTS ----------------------
float vectoring_text_proportions = 280/40;
float MSRO_text_proportions = 240/40;
float lnspace_logo_proportions = 1;

float vectoring_text_size = 35;
float MSRO_text_size = 40;
float lnspace_logo_size = 50;

String formatTimestamp(float t) {
  int minutes = int(t) / 60;
  int seconds = int(t) % 60;
  int hundredths = int((t - int(t)) * 100);
  
  return nf(minutes, 2) + ":" + nf(seconds, 2) + ":" + nf(hundredths, 2);
}

String sign(float n) {
  if (n > 0) 
    return("+");
  else if (n < 0)
    return("-");
  else 
    return(" ");
}

void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line != null) {
    line = trim(line);
    rawData = line;
    String[] fields = split(line, '\t');
    if (fields.length == labels.length) {
      data = fields;
      trimBuffers();
    }
    addDataPoint(al, vz, p, y, r, s1_pos);
  }
  
  om = parseFloatSafe(data[0]);
  ts = parseFloatSafe(data[1]);
  ax = parseFloatSafe(data[2]);
  ay = parseFloatSafe(data[3]);
  az = parseFloatSafe(data[4]);
  gx = parseFloatSafe(data[5]);
  gy = parseFloatSafe(data[6]);
  gz = parseFloatSafe(data[7]);
  p  = parseFloatSafe(data[8]);
  y  = parseFloatSafe(data[9]);
  r  = parseFloatSafe(data[10]);
  vx = parseFloatSafe(data[11]);
  vy = parseFloatSafe(data[12]);
  vz = parseFloatSafe(data[13]);
  al = parseFloatSafe(data[14]);
  tp = parseFloatSafe(data[15]);
  bv = parseFloatSafe(data[16]);
  s1_pos = parseFloatSafe(data[17]);
  mx = parseFloatSafe(data[18]);
  my = parseFloatSafe(data[19]);
  mz = parseFloatSafe(data[20]);
  lat = parseFloatSafe(data[21]);
  lng = parseFloatSafe(data[22]);
  al_gps = parseFloatSafe(data[23]);
  ve_gps = parseFloatSafe(data[24]);
  
  ac = sqrt(ax*ax + ay*ay + az*az);
  hedng = atan2(lat,lng);
  
  if(om == 1){
    arm_status = 1;
  }
  if(om == 3){
    recovery_status = 1;
  }
  if(ac > 15){
    launch_status = 1;
  }
}

void addDataPoint(float al, float vz, float p, float y, float r, float s1_pos) {
  tsData.add(ts);
  alData.add(al);
  vzData.add(vz);
  pData.add(p);
  yData.add(y);
  rData.add(r);
  s1_posData.add(s1_pos);

  // Keep arrays limited to graphLength
  if (tsData.size() > graphLength-1) {
    tsData.remove(0);
    alData.remove(0);
    vzData.remove(0);
    pData.remove(0);
    yData.remove(0);
    rData.remove(0);
    s1_posData.remove(0);
  }
}


void trimBuffers() {
  while (alData.size() > graphLength) alData.remove(0);
  while (vzData.size() > graphLength) vzData.remove(0);
  while (pData.size() > graphLength) pData.remove(0);
  while (yData.size() > graphLength) yData.remove(0);
  while (rData.size() > graphLength) rData.remove(0);
  while (s1_posData.size() > graphLength) s1_posData.remove(0);
  while (gzData.size() > graphLength) gzData.remove(0);
}

float parseFloatSafe(String s) {
  try {
    return Float.parseFloat(s);
  } catch (Exception e) {
    return 0;
  }
}

boolean mouseOverRestartButton() {
  return mouseX > 1294 && mouseX < 1294+180 && mouseY > 224 && mouseY < 224+60;
}
boolean mouseOverArmButton() {
  return mouseX > 320 && mouseX < 320+180 && mouseY > 224 && mouseY < 224+60;
}
boolean mouseOverRecoveryButton() {
  return mouseX > 963 && mouseX < 1294+180 && mouseY > 224 && mouseY < 224+60;
}


void mousePressed() {
  if (mouseOverRestartButton()) {
    myPort.write("REBOOT\n");  // or any custom keyword you choose
    gpsPoints.clear();
  }
  if (mouseOverArmButton()) {
    myPort.write("ARM\n");  // or any custom keyword you choose
  }
  if (mouseOverRecoveryButton()) {
    myPort.write("PARACHUTE\n");  // or any custom keyword you choose
  }
}


void drawLogosAndHeader() {
  fill(250);
  image(vectoring_text, 20, 50, vectoring_text_size*vectoring_text_proportions, vectoring_text_size);  
  image(MSRO_logo, 20, 110, MSRO_text_size*MSRO_text_proportions, MSRO_text_size);
  textAlign(LEFT);
  
  textFont(titleFont);
  image(lnspace_logo, 1650, 40, lnspace_logo_size*lnspace_logo_proportions, lnspace_logo_size);
  fill(0);
  text(title, 1700, 30);
  
  textFont(nameFont);
  text("LN_Space", 1710, 80);
  textAlign(RIGHT);
  
  textFont(font);
}

void updateBounds(float lat, float lng, float alt) {
  if (lat < minLat) minLat = lat;
  if (lat > maxLat) maxLat = lat;
  if (lng < minLng) minLng = lng;
  if (lng > maxLng) maxLng = lng;
  if (alt < minAlt) minAlt = alt;
  if (alt > maxAlt) maxAlt = alt;
}



/*----------BOXES FOR PLOTS AND GRAPHS ----------*/
void drawAttitudeVisualizer() {
  strokeWeight(1);
  pushMatrix();
  translate(width - 490, height - 330, 0);
  scale(2);
  lights();

  float pitch = parseFloatSafe(data[8]);
  float yaw   = parseFloatSafe(data[9]);
  float roll  = parseFloatSafe(data[10]);

  rotateX(HALF_PI);        // Orient rocket upward
  rotateZ(radians(-roll));
  rotateY(radians(-yaw));
  rotateX(radians(-pitch));

  // --- First draw transparent axes ---
  strokeWeight(2);
  stroke(255, 0, 0, 150);   // X-axis
  line(0, 0, 0, 30, 0, 0);
  stroke(0, 255, 0, 150);   // Y-axis
  line(0, 0, 0, 0, 30, 0);
  stroke(0, 0, 255, 150);   // Z-axis
  line(0, 0, 0, 0, 0, 150);

  // --- Then draw the rocket fully opaque on top ---
  fill(255, 0, 0);          // opaque red
  noStroke();
  shape(rocketModel);

  popMatrix();
}

/*Altitude Velocity Acceleration values*/
void drawAltitudeVelocityAccBox() {
  textFont(nameFont);
  fill(box_color);
  rect(300, 0, 326.3, 200);
  fill(text_color2);
  text("ALT: " + sign(al) + nf(abs(al),3,2) + "m      " + "\n\n" + "VEL: " + sign(vz) + nf(abs(vz),3,2) + "m/s   " + "\n\n" + "ACC: " + sign(ac) + nf(abs(ac),3,2) + "m/s^2", 580, 50);
}

void drawPYRBox() {
  textFont(nameFont);
  fill(box_color);
  rect(631.3, 0, 326.3, 200);
  fill(text_color2);
  text("PITCH: " + sign(p) + nf(abs(p),2,2) + "°" + "\n\n" + "YAW: " + sign(y) + nf(abs(y),2,2) + "°" + "\n\n" + "ROLL: " + sign(r) + nf(abs(r),2,2) + "°", 860, 50);
}

/*Fin Orientation Scheme*/
void drawFinOrientationFigure() {
  textFont(font);
  fill(box_color);
  rect(962.5, 0, 326.3, 200);
  fill(text_color2);
  text("Roll Figure", 1175, 30);
   pushMatrix();
  translate(1130, 110);  // center of roll view (adjust as needed)

  // Rocket body
  stroke(255);
  fill(50);
  ellipse(0, 0, 100, 100);

  // Apply roll rotation
  rotate(r*PI/180);

  // Fins
  strokeWeight(4);
  for (int i = 0; i < 4; i++) {
    float angle = radians(i * 90);
    float x1 = cos(angle) * 10;
    float y1 = sin(angle) * 10;
    float x2 = cos(angle) * 45;
    float y2 = sin(angle) * 45;

    // Color the leading fin (0° direction)
    if (i == 0) {
      stroke(255, 0, 0);  // red
    } else {
      stroke(0, 200, 255);  // blue/cyan
    }

    line(x1, y1, x2, y2);
  }

  // Optional center dot
  stroke(255);
  strokeWeight(2);
  point(0, 0);

  popMatrix();
}

/*Roll rate indicator needle dial*/
void drawRollRateIndicator() {
  textFont(font);
  fill(box_color);
  rect(1293.8, 0, 326.3, 200);
  fill(text_color2);
  text("Roll Rate", 1495, 30);

  // Dial center coordinates
  float cx = 1455;
  float cy = 100;

  // Draw outer circle
  stroke(0);
  fill(255);
  ellipse(cx, cy, 120, 120);  // dial face

  // Map roll rate to angle (clamp for visual limit)
  float maxRate = 500;
  gz = constrain(gz, -maxRate, maxRate);
  float angle = map(gz, -maxRate, maxRate, -PI/2, PI/2);

  // Draw needle
  stroke(255, 0, 0);
  strokeWeight(4);
  float len = 60;
  float needleX = cx + len * cos(angle);
  float needleY = cy + len * sin(angle);
  line(cx, cy, needleX, needleY);

  // Center point
  fill(0);
  ellipse(cx, cy, 8, 8);

  // Text (below dial)
  fill(255);
  textSize(14);
  textAlign(CENTER);
  text("Roll Rate", cx, cy + 75);
  text(nf(gz, 1, 1) + " °/s", cx, cy + 90);

  strokeWeight(1);
}

/*Flight Info*/
void drawFlightInfo() {
  strokeWeight(1);
  textFont(font);
  fill(text_color1);
  ZoneId zone = ZoneId.of("Europe/Rome");
  ZonedDateTime now = ZonedDateTime.now(zone);
  DateTimeFormatter formatter = DateTimeFormatter.ofPattern("dd/MM/yyyy HH:mm:ss");
  currentTimeStr = now.format(formatter);
  
  PO_time = ts;
  if(ts>ts_on){
    ts_on = ts;
  }
  if(om == 2){
    LO_time = ts-ts_on;
  }
  text("UTC+1: " + currentTimeStr, 1775, 130);
  text("PO Time: " + formatTimestamp(PO_time) , 1816, 150);
  text("LO Time: " + formatTimestamp(LO_time) , 1816, 170);
}

/*Roll Control Values*/
void drawRollControlBox() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(1620, 199.6, 300, 200);
  fill(text_color2);
  text("Roll Control", 1780, 220);
  text("s1_pos: " + s1_pos, 1690, 270);
  text("s2_pos: " + s2_pos, 1690, 300);
  text("s3_pos: " + s3_pos, 1690, 330);
  text("s4_pos: " + s4_pos, 1690, 360);
  text("SERVO STATUS 1: " + s1_status, 1735, 245);
  strokeWeight(2);
  stroke(255);
  line(1620+260/2, 255, 1620+260/2, 199.6+200);
  text("roll_set:      " + r_set, 1830, 270);
  text("roll_actual:   " + r, 1830, 300);
  text("r_rate_set:    " + gz_set, 1830, 330);
  text("r_rate_actual: " + gz, 1830, 360);
}

/*Parafoil Control Values*/
void drawParafoilControlBox() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(1620, 405, 300, 200);
  fill(text_color2);
  text("Parafoil Control", 1780, 425);
  text("SERVO STATUS 2: " + s2_status, 1735, 450);
  strokeWeight(2);
  stroke(255);
  line(1620+260/2, 460, 1620+260/2, 405+200);
  text("s1P_pos: " + s1P_pos, 1690, 475);
  text("s2P_pos: " + s2P_pos, 1690, 505);
  text("hedng_set: " + s2P_pos, 1830, 475);
  text("hedng_actual: " + hedng, 1830, 505);
  text("drate_set: " + vz_set, 1830, 535);
  text("drate_actual: " + abs(vz), 1830, 565);
}

/*Raw Data Values*/
void drawRawDataSection() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(1620, 610, 300, 470);
  fill(text_color2);
  text("Data Values", 1620+300/2, 630);
  
textFont(font);
int xLeft = 1690;
int xRight = 1830;  // Second column starts to the right
int yStart = 670;
int lineSpacing = 30;

int half = ceil(labels.length / 2.0);  // divide labels in half

for (int i = 0; i < labels.length; i++) {
  String valStr = "N/A";
  if (i < data.length) {
    float val = parseFloatSafe(data[i]);
    String sign = val >= 0 ? "+" : "-";
    float absVal = abs(val);
    valStr = sign + nf(absVal, 3, 2);
  }

  // Decide which column and row
  int x = (i < half) ? xLeft : xRight;
  int y = yStart + ((i < half) ? i : i - half) * lineSpacing;

  text(labels[i] + ": " + valStr, x, y);
  }
}

/*Live Video Feed*/
void drawVideoFeedBox() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(1365, 830, 250, 250);
  fill(text_color2);
  text("Live Feed", 1365+250/2, 850);
  if (cam.available()) {
    cam.read();
  }

  image(cam, 1365, 830, 250, 250); // Display webcam feed
}

/*Raw Telemetry values*/
void drawTelemetryBox() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(560, 830, 800, 250);
  fill(text_color2);
  text("Raw Telemetry", 560+800/2, 850);
  textFont(mediumFont);
  text(rawData, 560+800/2, 830+220/2);
}

/*Weather values*/
void drawWeatherBox() {
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(305, 830, 250, 250);
  fill(text_color2);
  text("Weather Station", 305+250/2, 850);
  text("Wind SPD: \n" + w_spd +" m/s", 305+60, 880);
  text("Wind DIR: \n" + w_dir +" ° N", 305+60, 930);
  
  text("Temp: \n" + tp_ground +" °C", 305+180, 880);
  text("Hum: \n" + hum +" %", 305+180, 930);
  text("Prs: \n" + prs +" hPa", 305+180, 980);
  
  strokeWeight(2);
  stroke(255);
  line(305+250/2, 860, 305+250/2 ,850+250);
}

/* Trajectory Plot 3D */
void drawTrajectory3DPlot() {
  pg3D.beginDraw();
  pg3D.background(255);
  pg3D.perspective();
  pg3D.ambientLight(255, 255, 255); // brighter scene
  pg3D.noLights();                 // optional: remove dimming

  pg3D.textFont(font);
  pg3D.textAlign(CENTER, CENTER);

  pg3D.camera();
  pg3D.translate(pg3D.width / 2, pg3D.height / 2 + 30, 0);

  // Rotations for real-world orientation
  pg3D.rotateX(HALF_PI);
  pg3D.rotateZ(PI / 4 + PI / 12);

  int axisLen = 80;
  int gridSpacing = 10;

  // Grid on XY plane (Z=0)
  pg3D.stroke(230);
  pg3D.strokeWeight(1);
  for (int i = -axisLen; i <= axisLen; i += gridSpacing) {
    pg3D.line(i, -axisLen, 0, i, axisLen, 0);
    pg3D.line(-axisLen, i, 0, axisLen, i, 0);
  }

  // Axes
  pg3D.strokeWeight(2);
  pg3D.stroke(255, 0, 0);    // Longitude
  pg3D.line(0, 0, 0, axisLen, 0, 0);

  pg3D.stroke(0, 255, 0);    // Latitude
  pg3D.line(0, 0, 0, 0, -axisLen, 0);

  pg3D.stroke(0, 0, 255);    // Altitude
  pg3D.line(0, 0, 0, 0, 0, axisLen);

  // Trajectory
  pg3D.stroke(0, 120, 255);
  pg3D.strokeWeight(2);
  pg3D.noFill();
  pg3D.beginShape();
  for (PVector p : gpsPoints) {
    float x = map(p.x, 0, 100, 0, axisLen);
    float y = map(p.y, 0, 50, 0, axisLen);
    float z = map(p.z, 0, 200, 0, axisLen);
    pg3D.vertex(x, y, z);
  }
  pg3D.endShape();

  // Current position dot
  if (gpsPoints.size() > 0) {
    PVector last = gpsPoints.get(gpsPoints.size() - 1);
    float x = map(last.x, 0, 100, 0, axisLen);
    float y = map(last.y, 0, 50, 0, axisLen);
    float z = map(last.z, 0, 200, 0, axisLen);
    pg3D.pushMatrix();
    pg3D.translate(x, y, z);
    pg3D.fill(255, 0, 0);
    pg3D.noStroke();
    pg3D.sphereDetail(4);
    pg3D.sphere(4);
    pg3D.popMatrix();
  }
  pg3D.endDraw();
  
  // Draw white background container in GUI
  fill(box_color);
  stroke(0);
  rect(0, 680, 300, 400); // exact container

  // Draw label
  fill(text_color2);
  textAlign(CENTER);
  textFont(font);
  text("3D Trajectory", 150, 700);

  // Draw pg3D image (clipped)
  image(pg3D, 0, 680);
}



/*Ground Track plot 2D*/
void addNewGPSPoint() {
  float x = r;
  float y = -r/2;
   z = r*10;

  // Keep within bounds
  if (x >= -10 && x <= 10 && y >= -10 && y <= 10) {
    gpsPoints.add(new PVector(x, y, z));
  }
}
void drawGroundTrackPlot() {
  // Plot box
  fill(255);
  stroke(0);
  rect(0, 405, 300, 270);
  
  // Title
  fill(0);
  textAlign(CENTER, CENTER);
  textFont(font);
  text("Ground Track", 150, 425);

  // Define plotting area
  int left = 40;
  int right = 280;
  int top = 405 + 20;
  int bottom = 405 + 270 - 30;

  // Draw axes lines
  stroke(0);
  strokeWeight(1);
  textFont(smallFont);
  // X axis (bottom)
  line(left, bottom, right, bottom);
  // Y axis (left)
  line(left, bottom, left, top);

  // Draw ticks and labels
  fill(0);
  textAlign(CENTER, CENTER);
  int numTicks = 5;

  // Longitude (X-axis) ticks and labels
  for (int i = 0; i <= numTicks; i++) {
    float x = map(i, 0, numTicks, left, right);
    line(x, bottom, x, bottom + 5);
    
    // Map to longitude values (-10 to 10 from your example)
    float lngVal = map(i, 0, numTicks, -10, 10);
    String label = nf(lngVal, 1, 1); // one decimal place
    text(label, x, bottom + 15);
  }

  // Latitude (Y-axis) ticks and labels
  textAlign(RIGHT, CENTER);
  for (int i = 0; i <= numTicks; i++) {
    float y = map(i, 0, numTicks, bottom, top); // y flipped
    line(left - 5, y, left, y);

    // Map to latitude values (-10 to 10)
    float latVal = map(i, 0, numTicks, -10, 10);
    String label = nf(latVal, 1, 1);
    text(label, left - 7, y);
  }

  // Plot trail
  noFill();
  stroke(0, 150, 255);
  strokeWeight(2);
  beginShape();
  for (PVector point : gpsPoints) {
    float x = map(point.x, -10, 10, left, right);
    float y = map(point.y, -10, 10, bottom, top); // flip Y
    vertex(x, y);
  }
  endShape();

  // Current position marker
  if (gpsPoints.size() > 0) {
    PVector last = gpsPoints.get(gpsPoints.size() - 1);
    float x = map(last.x, -10, 10, left, right);
    float y = map(last.y, -10, 10, bottom, top);
    fill(255, 0, 0);
    noStroke();
    ellipse(x, y, 6, 6);
  }
}



/*Flight Status Values*/
void drawStatusBox() {
  stroke(0, 0, 0); // Y
  strokeWeight(1);
  textFont(font);
  fill(box_color);
  rect(0, 200, 300, 200);
  fill(text_color2);
  text("Status",0+300/2, 220);
  
  text("Op_Mode: " + om, 0+300/2, 250);
  text("Vbat: " + bv, 0+300/2, 280);
  text("Temp: " + tp, 0+300/2, 310);
  text("RSSI: " + rssi, 0+300/2, 340);
  text("GPS_sat: " + gpssat, 0+300/2, 370);
}

/*Action Buttons and Indicators*/
void drawActionButtons() {
  strokeWeight(1);
  textFont(font);
  fill(50, 200, 50); 
  rect(320, 224, 180, 60); 
  fill(255); 
  textAlign(CENTER, CENTER);
  text("ARM: " + "[" + arm_status + "]", 320+180/2, 224+60/2); 
  
  fill(200, 50, 50); 
  rect(632, 224, 180, 60); 
  fill(255); 
  textAlign(CENTER, CENTER);
  text("LAUNCH: " + "[" + launch_status + "]", 632+180/2, 224+60/2);
  
  fill(100, 100, 255); 
  rect(963, 224, 180, 60); 
  fill(255); 
  textAlign(CENTER, CENTER);
  text("RECOVERY: " + "[" + recovery_status + "]", 963+180/2, 224+60/2);
 
  fill(#25E5CD);
  rect(1294, 224, 180, 60);
  fill(255);
  text("RESTART", 1294+180/2, 224+60/2);
}


/*4 main plots*/
void updateData() {
  // Shift data left
  for (int i = 0; i < alHistory.length - 1; i++) {
    alHistory[i] = alHistory[i+1];
    vzHistory[i] = vzHistory[i+1];
    PHistory[i] = PHistory[i+1];
    s1_posHistory[i] = s1_posHistory[i+1];
  }
  
  alHistory[alHistory.length - 1] = al;
  vzHistory[vzHistory.length - 1] = vz;
  PHistory[PHistory.length - 1] = p;
  s1_posHistory[s1_posHistory.length - 1] = s1_pos;
}

/*----------Altitude Graph----------*/
void drawalGraph() {
  alPlot.setPoints(getGraphPointsFromAl(alHistory));
  alPlot.defaultDraw();
}
/*----------Altitude Graph----------*/



/*----------vz Graph----------*/
void drawvzGraph() {
  vzPlot.setPoints(getGraphPointsFromVz(vzHistory));
  vzPlot.defaultDraw();
}
/*----------vz Graph----------*/



/*----------P Graph----------*/
void drawPGraph() {
  PPlot.setPoints(getGraphPointsFromP(PHistory));
  PPlot.defaultDraw();
}
/*----------P Graph----------*/



/*----------s1_pos Graph----------*/
void draws1_posGraph() {
  s1_posPlot.setPoints(getGraphPointsFromS1(s1_posHistory));
  s1_posPlot.defaultDraw();
}
/*----------s1_pos Graph----------*/

GPointsArray getGraphPointsFromAl(float[] al) {
  GPointsArray points = new GPointsArray(al.length);
  for (int i = 0; i < al.length; i++) {
    points.add(i, al[i]);
  }
  return points;
}

GPointsArray getGraphPointsFromVz(float[] vz) {
  GPointsArray points = new GPointsArray(vz.length);
  for (int i = 0; i < vz.length; i++) {
    points.add(i, vz[i]);
  }
  return points;
}

GPointsArray getGraphPointsFromP(float[] p) {
  GPointsArray points = new GPointsArray(p.length);
  for (int i = 0; i < p.length; i++) {
    points.add(i, p[i]);
  }
  return points;
}

GPointsArray getGraphPointsFromS1(float[] s1_pos) {
  GPointsArray points = new GPointsArray(s1_pos.length);
  for (int i = 0; i < s1_pos.length; i++) {
    points.add(i, s1_pos[i]);
  }
  return points;
}
