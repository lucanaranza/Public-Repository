/*GROUND STATION GUI - telemetry downlink*/

/*----------IMPORT LIBRARIES----------*/
import processing.serial.*;
import java.io.File;
import java.io.PrintWriter;
import processing.video.*;

/*----------TELEMETRY SIGNALS - in order from telemetry stream----------*/
float azimuth = 0;       // ground station data
float elevation = 0;
float distance_gs = 0;
int mode_gs_int = 0;
float batt_gs = 0;
float rssi = 0;
float heading = 0;

int packet_id = 0;      // telemetry stream incoming
int om_int = 0;
float fq = 0;
int sd_flag = 0;
float ts = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float pitch = 0; float yaw = 0; float roll = 0;

float alt = 0;
float vz = 0;
float s_pos = 0;
float temp = 0;
float batt_rocket = 0;
int gps_sats = 0;
float X_pos = 0;
float Y_pos = 0;

float[] trackX = new float[200];
float[] trackY = new float[200];
float lastTs = 0;
float currentX = 0;
float currentY = 0;
int trackSize = 0;

/*----------PARSING AND DATALOGGING AND INPUT AND DATA CONVERSION----------*/
String[] ports;              // ports for USB communication
String line = "";
Serial ser;
boolean connected = false;
    
PrintWriter logFile;         // file for datalogging

String inputBuffer = "";     // Input string for keyboard commands
boolean typing = false;

String mode_rocket = "";     // Text value of operative mode 
String mode_gs = "";
String sd_flag_text = "";

float mach = 0;              // Calculations for extra data
float speed_sound = 0;
float velocity_ms = 0;

int warmupPackets = 10;
float defaultMaxX = 800;
float defaultMaxY = 800;

/*---------- RECORDING VARIABLES ----------*/
boolean VIDEO = false;   // change this to make the video feed active or not 
boolean recording = false;
int frameCounter = 0;
Capture cam;
boolean cam_flag = false;
PImage logo;
/*----------SETUP ----------*/

void setup() {
  size(1920, 1080);
  fullScreen();
  pixelDensity(displayDensity());
  logo = loadImage("logo.jpg");
  
  if (VIDEO==true) {
    String[] cameras = Capture.list();
    println("Available cameras:");
    printArray(cameras);
    cam = new Capture(this, cameras[0]);
    cam.start();
  }
  
  ports = Serial.list();


  File folder = new File(sketchPath("logs"));
  if (!folder.exists()) {
    folder.mkdir();
  }

 smooth(8);
 textFont(createFont("Consolas", 25, true));
 textMode(MODEL);

  println("Available ports:");
  for (int i = 0; i < ports.length; i++) {
    println(i + ": " + ports[i]);
  }
  
  File recFolder = new File(sketchPath("recordings"));
  if (!recFolder.exists()) recFolder.mkdir();
}

/*----------DRAW ----------*/
void draw() {
  background(236, 224, 200); // clear parchment tone (stronger sand feel)
  if (VIDEO==true) {
      if (cam.available()) {
        cam.read();
        cam_flag = true;
      }
  }
  
  image(logo, 1755, 100, 100, 100);
  
  if (!connected) {
    drawPortSelection();
    return;
  }
  drawDashboard();
  drawTerminal();
  
  if (recording) {
    saveFrame("recordings/frame-######.jpg");
    frameCounter++;
  }
}

/*----------SERIAL ----------*/
void serialEvent(Serial s) {
  line = trim(s.readStringUntil('\n'));
  
  if (line == null) return;
  String[] d = split(line, ',');
  
  if (d.length < 5) return;
  
  try {
    azimuth     = float(d[0]);   //used to display gs data
    elevation   = float(d[1]);   //used to display gs data
    distance_gs = float(d[2]);   //used to display gs data
    mode_gs_int   = int(d[3]);   //used to display gs data
    packet_id     = int(d[4]);   //not used for now
    om_int        = int(d[5]);   //used for state machine display
    fq          = float(d[6]);   //used in system data
    sd_flag       = int(d[7]);   //used in system data
    ts          = float(d[8]);   //used in date time 
    ax          = float(d[9]);   //used in acceleration box
    ay          = float(d[10]);  //used in acceleration box
    az          = float(d[11]);  //used in acceleration box
    gx          = float(d[12]);  //used in gyro box
    gy          = float(d[13]);  //used in gyro box
    gz          = float(d[14]);  //used in gyro box and roll rate dial
    pitch       = float(d[15]);  //used in important box2
    yaw         = float(d[16]);  //used in important box2
    roll        = float(d[17]);  //used in important box2 and roll dial
    alt         = float(d[18]);  //used in important box1
    vz          = float(d[19]);  //used in important box1
    s_pos       = float(d[20]);  //used in active control box
    temp        = float(d[21]);  //used in system data
    X_pos       = float(d[22]);  //used in ground track
    Y_pos       = float(d[23]);  //used in ground track
    gps_sats    = int(d[24]);    //used in system data      
    batt_rocket = float(d[25]);  //used in system data
    heading     = float(d[26]);
    
    if (ts < lastTs) {
      resetTrack();
    }
    lastTs = ts;
    
    if (warmupPackets > 0) {
      warmupPackets--;
      return;
    }
    
    addTrackPoint(X_pos, Y_pos);
    
    speed_sound = speedOfSound(temp);   // extra data calculations 
    mach = vz / speed_sound;            //used in important box1
        
    switch  (sd_flag) {                 // SD flag
      case 0:
        sd_flag_text = "SD OFF";
        break;
      case 1:
        sd_flag_text = "SD ON";
        break;
    }
    
    switch (om_int) {                   // Rocket modes
      case 0:
        mode_rocket = "Low Power";
        break;
      case 1:
        mode_rocket = "Idle";
        break;
      case 2:
        mode_rocket = "Armed";
        break;
      case 3:
        mode_rocket = "Powered Ascent";
        break;
      case 4:
        mode_rocket = "Coast";
        break;
      case 5:
        mode_rocket = "Parachute Event";
        break;
      case 6:
        mode_rocket = "Stable Descent";
        break;
      case 7:
        mode_rocket = "Landed";
        break;
    }

    switch (mode_gs_int) {               // ground station modes
      case 0:
        mode_gs = "Manual Tracking";
        break;
      case 1:
        mode_gs = "Automatic Tracking";
        break;
    }
  
    if (logFile != null) {
      logFile.println(line);
      logFile.flush();
    }

  } catch(Exception e) {
    println("Parse Error!!!");
  }
}

/*----------SPEED OF SOUND / MACH CALCULATIONS ----------*/
float speedOfSound(float T) {
  return sqrt(1.4 * 287.0 * (T+273.15));
}
/*-------------------------------------------------------*/

/*----------COLOR MODE / STATE MACHINE DISPLAY ----------*/
color getModeColor(int om) {
  switch(om) {
  case 0: return color(80);          // Low power
  case 1: return color(120);         // Idle
  case 2: return color(255, 180, 0); // Armed
  case 3: return color(255, 80, 0);  // Ascent
  case 4: return color(0, 120, 255); // Coast
  case 5: return color(255, 0, 0);   // Parachute
  case 6: return color(0, 180, 0);   // Descent
  case 7: return color(0);           // Landed
  default: return color(150);        // DEFAULT
  }
}
/*-------------------------------------------------------*/

/*----------PORT SELECTION ----------*/
void drawPortSelection() {
  fill(0);
  textSize(22);
  text("Select Serial Port:", 30, 40);
  textSize(16);
  
  for (int i = 0; i < ports.length; i++) {
    fill(240);
    stroke(0);
    rect(30, 70 + i * 40, 500, 35, 6);

    fill(0);
    text(ports[i], 40, 95 + i * 40);
  }
}
/*------------------------------------*/

/*----------CONNECT ----------*/
void mousePressed() {

  // -----------------------------
  // SERIAL PORT SELECTION ONLY IF NOT CONNECTED
  // -----------------------------
  if (!connected) {

    for (int i = 0; i < ports.length; i++) {
      int yPos = 70 + i * 40;

      if (mouseX > 30 && mouseX < 530 &&
          mouseY > yPos && mouseY < yPos + 35) {

        ser = new Serial(this, ports[i], 500000);
        ser.bufferUntil('\n');

        String filename =
          "logs/telemetry_" +
          year() + "_" + month() + "_" + day() + "_" +
          hour() + "_" + minute() + "_" + second() + ".txt";

        logFile = createWriter(filename);
        println("Logging to: " + filename);

        connected = true;
      }
    }

    return; // important: prevents clicking record area during selection
  }

  // -----------------------------
  // RECORD BUTTON (ONLY AFTER CONNECTED)
  // -----------------------------
  if (mouseX > width - 160 && mouseX < width - 40 &&
      mouseY > height - 140 && mouseY < height - 100) {

    recording = !recording;

    if (recording) {
      println("Recording started...");
      frameCounter = 0;
    } else {
      println("Recording stopped...");
    }
  }
}
/*-----------------------------*/

/*----------DASHBOARD ----------*/
void drawDashboard() {
  /*GS data displayed on the left*/
  drawBox(30, 240, "Azimuth [°]", nf(azimuth, 0, 2));
  drawBox(30, 320, "Elevation [°]", nf(elevation, 0, 2));
  drawBox(30, 400, "Distance [m]", nf(distance_gs, 0, 2));
  drawBox(30, 480, "Mode", mode_gs);
  
  /*Main ascent data displayed in the center up*/
  drawImportantData(width/2 - 150, 30); 
  drawCommandHelp1();
  drawCommandHelp2();
  
  /*Acceleration Block*/
  drawBox(width - 230, 240, "Acc XYZ [m/s^2]", nf(ax, 0, 2) + " " + nf(ay, 0, 2) + " " + nf(az, 0, 2));

  /*Gyro Block*/
  drawBox(width - 230, 320, "Gyro XYZ [°/s]", nf(gx, 0, 2) + " " + nf(gy, 0, 2) + " " + nf(gz, 0, 2));

  /*Attitude Block*/
  drawAttitude(width/2 - 120, height - 180);
  
  /*System Data Block*/
  drawSystemPanel(width/2 - 250, 30, 500, 220);
  
  /*Roll Position Block*/
  drawRoll(width/2 + 200, height - 180);
  
  /*Roll Rate Block*/
  drawRollRateDial(width/2 + 300, height - 180, gz);
  
  /*Draw time panel*/
  drawDateTimePanel(25, 60);
  
  /*Draw X Y Plot for X Y position*/
  drawXYPlot(width - 600, 250);
  
  /*Draw Active Control BOX*/
  drawControlBox(width - 550, 530);
  
  /*Draw Title*/
  textSize(32);
  text("Explorer 1", 1720, 90);
  
  float camW = 1100;
  float camH = 618;

  if (VIDEO==true) {
    image(cam, width/2 - camW/2 - 150, height/2 - camH/2 + 20, 1100, 618);
  }
  
}

/*----------TERMINAL ----------*/
void drawTerminal() {
  fill(0);
  textSize(14);
  text("Command Input: ", 30, height - 110);

  fill(255);
  stroke(0);
  rect(30, height - 90, width - 60, 40, 8);

  fill(0);
  text(inputBuffer, 40, height - 65);
  
  fill(recording ? color(255, 80, 80) : color(200));  // draws a toggle button whose appearance depends on the boolean variable
  stroke(0);
  rect(width - 160, height - 140, 120, 40, 8);

  fill(0);
  textSize(16);
  text(recording ? "RECORDING" : "RECORD", width - 145, height - 115);
}
/*------------------------------*/

/*----------INPUT HANDLING ----------*/
void keyPressed() {

  if (!connected) return;

  if (key == BACKSPACE) {
    if (inputBuffer.length() > 0) {
      inputBuffer = inputBuffer.substring(0, inputBuffer.length() - 1);
    }
  }

  else if (key == ENTER || key == RETURN) {
    if (inputBuffer.length() > 0) {
      ser.write(inputBuffer + "\n");
      println("SENT: " + inputBuffer);
      inputBuffer = "";
    }
  }

  else if (Character.isLetterOrDigit(key) || key == '.' || key == '-') {
    inputBuffer += Character.toUpperCase(key);
  }
}
/*---------------------------------*/

/*----------UI BOX ----------*/
void drawBox(int x, int y, String label, String value) {

  fill(255);
  stroke(0);
  rect(x, y, 200, 70, 10);

  fill(0);
  textSize(16);
  text(label, x + 10, y + 25);

  textSize(18);
  text(value, x + 10, y + 55);
}
/*----------------------------*/

/*----------ATTITUDE ----------*/
void drawAttitude(int x, int y) {
  x = x - 610;
  y = y - 870;
  
  fill(255);
  stroke(0);
  rect(x, y, 230, 200, 10);

  fill(0);
  
  textSize(20);
  text("Pitch: ", x + 20, y + 50);
  textSize(25);
  text(String.format("%+7.2f", pitch), x + 100, y + 50);
  textSize(20);
  text("°", x + 200, y + 50);
  
  text("Yaw: ", x + 20, y + 110);
  textSize(25);
  text(String.format("%+7.2f", yaw), x + 100, y + 110);
  textSize(20);
  text("°", x + 200, y + 110);
  
  text("Roll: ", x + 20, y + 170);
  textSize(25);
  text(String.format("%+7.2f", roll), x + 100, y + 170);
  textSize(20);
  text("°", x + 200, y + 170);
}
/*----------------------------*/

/*----------IMPORTANT FLIGHT DATA----------*/
void drawImportantData(int x, int y) {
  x = x - 340;
  y = y + 20;
  
  fill(255);
  stroke(0);
  rect(x, y-20, 230, 200, 10);
  fill(0);

  textSize(20);
  text("Alt: ", x + 15, y + 30);
  textSize(25);
  text(String.format("%+7.2f", alt), x + 80, y + 30);
  textSize(20);
  text(" m", x + 175, y + 30);
  
  text("Vel:   ", x + 15, y + 90);
  textSize(25);
  text(String.format("%+7.2f", vz), x + 80, y + 90);
  textSize(20);
  text(" m/s", x + 175, y + 90);
    
  text("Mach: ", x + 15, y + 150);
  textSize(25);
  text(String.format("%+7.2f", mach), x + 80, y + 150);
  textSize(20);
}

void drawCommandHelp1() {

  fill(255);
  stroke(0);
  rect(1490, height - 180, 400, 30, 10);

  fill(0);
  textSize(16);
  text(" RCKT CMDS: ", 1490, height - 160);
  textSize(14);
  text("LPO | IDL | ARM | DRM | PAR | REB", 1620, height - 160);
}
/*-----------------------------------------*/

void drawCommandHelp2() {

  fill(255);
  stroke(0);
  rect(30, height - 180, 500, 30, 10);

  fill(0);
  textSize(16);
  text("GS CMDS:", 40, height - 160);
  textSize(14);
  text("MAN | AUTO | AZx (0-180°) | ELx (0-90°) | DISTx (m)", 130, height - 160);
}
/*-----------------------------------------*/

/*----------ROLL POSITION ----------*/
void drawRoll(int x, int y) {
  x = x + 60;
  y = y - 870;
  
  fill(255);
  stroke(0);
  rect(x, y, 230, 200, 10);

  fill(0);
  textSize(20);
  text("Roll Position", x + 45, y + 25);
  text(nf(roll,0,2), x + 85, y + 50);
  
  pushMatrix();

  translate(x + 110, y + 120);

  // rotate by roll (convert to radians)
  rotate(radians(-roll));
  
  // heading marker
  stroke(0);
  line(0, 0, 0, -60);

  // rocket body
  stroke(0);
  fill(245);
  ellipse(0, 0, 100, 100);

  // fins (4 sides)
  for (int i = 0; i < 4; i++) {

    pushMatrix();
    rotate(HALF_PI * i);

    if (i == 0) {
      fill(255, 0, 0); // reference fin
    } else {
      fill(0);
    }

    rect(35, -8, 25, 16, 3);

    popMatrix();
  }

  popMatrix();
  
}
/*----------------------------*/

/*----------Roll Rate Mapping----------*/
float mapRollRate(float w) {

  float maxRate = 1000.0;

  float absW = constrain(abs(w), 0, maxRate);
  float sign = (w < 0) ? -1 : 1;

  float norm;

  // high resolution zone ±150°/s
  if (absW < 150) {
    norm = (absW / 150.0) * 0.6;
  }
  // mid + high range
  else {
    norm = 0.6 + ((absW - 150.0) / (maxRate - 150.0)) * 0.4;
  }

  norm = constrain(norm, 0, 1);

  return sign * norm * HALF_PI; // ±90° sweep
}


/*----------Dial----------*/
void drawRollRateDial(int x, int y, float rollRate) {

  float angle = mapRollRate(rollRate);
  x = x + 200;
  y = y - 870;

  // widget box
  fill(255);
  stroke(0);
  rect(x, y, 230, 200, 10);

  // title (left-aligned default)
  fill(0);
  textSize(20);
  text("Roll Rate", x + 60, y + 25);

  // center of dial
  float cx = x + 115;
  float cy = y + 155;

  // semicircle background
  noFill();
  stroke(180);
  arc(cx, cy, 160, 160, PI, TWO_PI);

  // -------- TICKS --------
  float[] ticks = {-1E3, -500, -150, -50, 0, 50, 150, 500, 1000};

  stroke(0);
  textSize(10);

  for (int i = 0; i < ticks.length; i++) {

    float w = ticks[i];
    float a = mapRollRate(w);

    float r1 = 70;
    float r2 = 82;

    float x1 = cx + cos(a - HALF_PI) * r1;
    float y1 = cy + sin(a - HALF_PI) * r1;
    float x2 = cx + cos(a - HALF_PI) * r2;
    float y2 = cy + sin(a - HALF_PI) * r2;

    line(x1, y1, x2, y2);

    // labels (manual offset instead of textAlign)
    if (w == 0) {
      float tx = cx -5 + cos(a - HALF_PI) * 100;
      float ty = cy + sin(a - HALF_PI) * 100 + 5;
      textSize(12);
      text(nf(w, 0, 0), tx+2, ty + 4);
    }
    if (abs(w) == 500 || abs(w) == 150) {
      float tx = cx + cos(a - HALF_PI) * 100;
      float ty = cy + sin(a - HALF_PI) * 100;
      textSize(12);
      text(nf(w, 0, 0), tx - 12, ty + 4);
    }
    if (w == 1000) {
      float tx = cx + cos(a - HALF_PI) * 100;
      float ty = cy + sin(a - HALF_PI) * 100;
      textSize(12);
      text("1E3", tx - 10, ty);
    }
     if (w == -1000) {
      float tx = cx + cos(a - HALF_PI) * 100;
      float ty = cy + sin(a - HALF_PI) * 100;
      textSize(12);
      text("-1E3", tx - 12, ty);
    }
  }

  // center reference line
  stroke(0);
  line(cx, cy, cx, cy - 80);

  // needle
  stroke(255, 0, 0);
  strokeWeight(3);
  float nx = cx + sin(angle) * 75;
  float ny = cy - cos(angle) * 75;
  line(cx, cy, nx, ny);
  strokeWeight(1);
  
  // numeric value (manual positioning)
  fill(0);
  textSize(18);
  text(nf(rollRate, 0, 1), cx - 15, cy + 30);
}
/*-------------------------------*/

/*----------TIME PANEL----------*/
void drawDateTimePanel(int x, int y) {

  fill(255);
  stroke(0);

  fill(0);
  textSize(20);

  text(
    "Date: " +
    year() + "-" +
    nf(month(), 2) + "-" +
    nf(day(), 2),
    x + 10, y + 30
  );

  text(
    "Time: " +
    nf(hour(), 2) + ":" +
    nf(minute(), 2) + ":" +
    nf(second(), 2),
    x + 10, y + 60
  );

  text(
    "TimeRKT: " + nf(ts, 0, 2) + " s",
    x + 10, y + 90
  );
}

/*----------XY Plot----------*/
void resetTrack() {
  trackSize = 0;
  currentX = 0;
  currentY = 0;
  warmupPackets = 10;

  for (int i = 0; i < trackX.length; i++) {
    trackX[i] = 0;
    trackY[i] = 0;
  }
}

void addTrackPoint(float X_pos, float Y_pos) {

if (abs(X_pos) > 800 || abs(Y_pos) > 800) return;

  if (trackSize >= trackX.length) {
    trackSize = 0; // or shift buffer (better)
  }

  if (trackSize < trackX.length) {
    trackX[trackSize] = X_pos;
    trackY[trackSize] = Y_pos;
    trackSize++;
  }
  currentX = X_pos;
  currentY = Y_pos;
}

void drawXYPlot(int x, int y) {
  x = x + 220;
  y = y + 350;
  
  fill(255);
  stroke(0);
  rect(x, y, 350, 290, 10);

  fill(0);
  textSize(18);
  text("GROUND TRACK", x + 10, y + 25);
  text("X pos \n" + nf(X_pos, 0, 7) + "°E", x + 10, y + 260);
  text("Y pos \n" + nf(Y_pos, 0, 7) + "°N", x + 230, y + 260);
  text("HDG \n" + nf(heading, 0, 2) + "°", x + 145, y + 260);

  stroke(200);
  line(-150+175+x, 0+130+y, 150+175+x, 0+130+y);
  line(0+175+x, -100+130+y, 0+175+x, 100+130+y);

  stroke(0);
  noFill();
  
  float halfW = 150;
  float halfH = 125;
  
  float maxX = 11.059000;
  float maxY = 45.413000;
 
  float scaleX = halfW / maxX;
  float scaleY = halfH / maxY;
  float scale = min(scaleX, scaleY); 
  
  if (trackSize < 2) return;
  
  if ((abs(currentX)<maxX) && abs(currentY)<maxY){
    for (int i = 1; i < trackSize; i++) {         //trackX[i-1] trackX[i]
      float x1 = x + 175 + trackX[i-1] * scale;
      float y1 = y + 130 - trackY[i-1] * scale;
      float x2 = x + 175 + trackX[i] * scale;
      float y2 = y + 130 - trackY[i] * scale;
      
      strokeWeight(3);
      line(x1, y1, x2, y2);
    
      stroke(0);
      textSize(14);
    }
  }
  strokeWeight(1);
 
}
/*-----------------------------------*/

/*----------CONTROL BOX PANEL----------*/
void drawControlBox(int x, int y) {
  x = x + 320;
  y = y - 130;
  
  fill(255);
  stroke(0);
  rect(x, y, 200, 100, 10);

  fill(0);

  textSize(20);
  text("ACTIVE CONTROL", x + 20, y + 25);

  textSize(20);
  text("S_pos", x + 20, y + 70);
  textSize(25);
  text(nf(s_pos*180.0/PI, 0, 2) + "°", x + 100, y + 70);
  textSize(20);
}

/*-----------------------------------*/

/*---------- SYSTEM PANEL ----------*/
void drawSystemPanel(int x, int y, int w, int h) {
  
  x = x + 0;

  // outer box
  fill(255);
  stroke(0);
  rect(x, y, w, h-20, 10);

  // ---------------- TITLE BAR ----------------
  color c = getModeColor(om_int);

  fill(c);
  rect(x, y, w, 35, 10);

  fill(255);
  textSize(16);
  text("ROCKET SYSTEM STATUS", x + 10, y + 24);
  text(mode_rocket, x + 300, y + 24);

  // ---------------- LEFT SIDE ----------------
  fill(0);
  textSize(14);

  text("BATT ROCKET: " + batt_rocket + " V", x + 10, y + 65);
  text("BATT GS:     " + nf(batt_gs, 0, 2) + " V", x + 10, y + 95);

  text("SYSTEM MODE: " + om_int, x + 10, y + 125);
  text("TEMP:        " + nf(temp, 0, 2) + " C", x + 10, y + 155);
  text("SD MODE:     " + sd_flag_text, x + 10, y + 185);

  // ---------------- DIVIDER ----------------
  line(x + w/2, y + 45, x + w/2, y + h - 30);

  // ---------------- RIGHT SIDE ----------------
  text("CAMERA: " + cam_flag, x + w/2 + 10, y + 65);
  text("SOUNDSPEED: " + nf(speed_sound, 0, 1) + " m/s", x + w/2 + 10, y + 95);
  text("RSSI:       " + nf(rssi, 0, 1), x + w/2 + 10, y + 125);
  text("TELEM Hz:   " + nf(fq, 0, 1), x + w/2 + 10, y + 155);
  text("GPS SATS:   " + gps_sats, x + w/2 + 10, y + 185);
}
/*-----------------------------------------*/
