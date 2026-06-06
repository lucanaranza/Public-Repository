import processing.serial.*;
import java.util.*;

// --- Serial ---
Serial myPort;

// --- Telemetry variables --- AZ ; EL ; DIST ; MODE OF ANTENNA POINTER ; PACKETID ; OPERATIVE MODE, FREQUENCY, SD FLAG, TIMESTAMP, AX AY AZ, GX GY GZ, PITCH YAW ROLL, ALTITUDE, VZ, SERVO POS
float azimuth=0, elevation=0, distancegs=0;
int packet_id=0;
float fq=0;
int sd_flag=0;
float ts=0, ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
float p=0, yaw=0, r=0, al=0, vz=0; s_pos=0;
int gs_om=0;int om=0;

// --- GUI parameters ---
int panelMargin = 20;
int panelSpacing = 20;
int panelRadius = 15;
int panelWidth, panelHeight;
int textOffsetX = 180; // move all panel text right

// --- Command buttons ---
String[] commands = {"ARM", "PAR", "REB"};
Rectangle[] buttons;

// --- Graph history ---
ArrayList<Float> altHistory = new ArrayList<Float>();
ArrayList<Float> vzHistory = new ArrayList<Float>();
ArrayList<Float> tsHistory = new ArrayList<Float>();  // Store timestamps
int graphMaxPoints = 200; // number of points in graph

// --- 3D Rocket Model ---
PShape rocket;
PGraphics pg3d;  // offscreen graphics for 3D view

void setup() {
  size(1600, 900, P3D);  // Enable P3D for 3D rendering
  smooth();
  
  // shrink panels to give more graph space
  panelWidth = 400;
  panelHeight = 200;  // smaller height
  
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 500000);
  myPort.bufferUntil('\n');
  
  textAlign(LEFT, CENTER);
  textSize(20);
  
  // --- Setup buttons ---
  buttons = new Rectangle[commands.length];
  int btnWidth = 200;
  int btnHeight = 50;
  int startX = width - panelMargin - btnWidth - 100; // moved 100px to left
  int startY = height - panelMargin - (btnHeight + 10) * commands.length;
  for(int i=0; i<commands.length; i++){
    buttons[i] = new Rectangle(startX, startY + i*(btnHeight + 10), btnWidth, btnHeight);
  }
  
  // --- Load 3D rocket model ---
  rocket = loadShape("rocket.obj");
  
  // --- Create offscreen graphics for 3D view ---
  pg3d = createGraphics(380, 380, P3D);
}

void draw() {
  background(30);
  
  // Reset text rendering for 2D
  hint(DISABLE_DEPTH_TEST);
  
  // --- Draw telemetry panels ---
  int x = panelMargin;
  int y = panelMargin;
  
  drawPanel(x, y, panelWidth, panelHeight, "Acceleration", "ax", ax, "ay", ay, "az", az);
  drawPanel(x + panelWidth + panelSpacing, y, panelWidth, panelHeight, "Gyroscope", "gx", gx, "gy", gy, "gz", gz);
  
  y += panelHeight + panelSpacing;
  drawPanel(x, y, panelWidth, panelHeight, "Orientation", "Pitch", p, "Yaw", yaw, "Roll", r);
  drawPanel(x + panelWidth + panelSpacing, y, panelWidth, panelHeight, "Altitude/VZ", "AL", al, "VZ", vz);
  
  y += panelHeight + panelSpacing;
  drawPanel(x, y, panelWidth, panelHeight, "Other", "OM", om);
  
  // --- Draw 3D Attitude Visualizer ---
  draw3DRocket(x + panelWidth + panelSpacing, y, panelWidth, panelHeight);
  
  // --- Update graph data ---
  altHistory.add(al);
  vzHistory.add(vz);
  tsHistory.add(ts);  // Store timestamp
  if(altHistory.size() > graphMaxPoints) altHistory.remove(0);
  if(vzHistory.size() > graphMaxPoints) vzHistory.remove(0);
  if(tsHistory.size() > graphMaxPoints) tsHistory.remove(0);
  
  // --- Draw graphs ---
  int graphW = width - panelWidth*2 - panelSpacing*3 - 90; // wide graph
  int graphH = 180;
  int graphX = panelWidth*2 + panelSpacing*3 + 50;
  int graphY = panelMargin;
  drawGraph(graphX, graphY, graphW, graphH, altHistory, tsHistory, "Altitude", color(0,200,200));
  
  drawGraph(graphX, graphY + graphH + 30, graphW, graphH, vzHistory, tsHistory, "VZ", color(200,100,0));
  
  // --- Draw command buttons ---
  for(int i=0; i<commands.length; i++){
    Rectangle btn = buttons[i];
    fill(100, 200, 150, 200);
    stroke(255);
    strokeWeight(2);
    rect(btn.x, btn.y, btn.w, btn.h, 8);
    
    fill(0);
    textAlign(CENTER, CENTER);
    textSize(20);
    text(commands[i], btn.x + btn.w/2, btn.y + btn.h/2);
  }
}

void mousePressed(){
  for(int i=0; i<buttons.length; i++){
    Rectangle btn = buttons[i];
    if(mouseX > btn.x && mouseX < btn.x + btn.w && mouseY > btn.y && mouseY < btn.y + btn.h){
      myPort.write(commands[i] + "\n");
      println("Sent command: " + commands[i]);
    }
  }
}

// --- Draw a simple scrolling graph with axis and scale values ---
void drawGraph(int x, int y, int w, int h, ArrayList<Float> data, ArrayList<Float> timestamps, String title, int col){
  fill(50,50,50,200);
  stroke(255);
  strokeWeight(2);
  rect(x, y, w, h, 12);
  
  fill(255);
  textSize(20);
  textAlign(LEFT, TOP);
  text(title, x + 10, y + 5);
  
  if(data.size() < 2) return;
  
  // Find min/max for scaling
  float minVal = Collections.min(data);
  float maxVal = Collections.max(data);
  if(minVal == maxVal){ maxVal += 1; minVal -= 1; }
  
  float minTime = Collections.min(timestamps);
  float maxTime = Collections.max(timestamps);
  if(minTime == maxTime){ maxTime += 1; }
  
  // Draw y-axis labels
  fill(255);
  textSize(12);
  textAlign(RIGHT, CENTER);
  for(int i=0; i<=4; i++){
    float val = lerp(minVal, maxVal, i/4.0);
    float py = map(val, minVal, maxVal, y + h - 10, y + 25);
    text(nf(val,0,2), x - 5, py);
    stroke(100);
    line(x, py, x + w, py); // optional grid
  }
  
  // Draw x-axis labels with fixed timestamps
  textAlign(CENTER, TOP);
  float timeRange = maxTime - minTime;
  float timeStep = timeRange / 5.0;
  for(int i=0; i<=5; i++){
    float timeVal = minTime + i * timeStep;
    float px = map(timeVal, minTime, maxTime, x, x + w);
    text(nf(timeVal,0,2), px, y + h + 5);
  }
  
  // Draw data line
  noFill();
  stroke(col);
  strokeWeight(2);
  beginShape();
  for(int i=0; i<data.size(); i++){
    float gx = map(timestamps.get(i), minTime, maxTime, x, x + w);
    float gy = map(data.get(i), minVal, maxVal, y + h - 10, y + 25);
    vertex(gx, gy);
  }
  endShape();
}

// --- Draw 3D Rocket Attitude Visualizer ---
void draw3DRocket(int x, int y, int w, int h){
  // Render 3D scene to offscreen graphics
  pg3d.beginDraw();
  pg3d.background(30, 30, 50);
  pg3d.lights();
  
  // Setup camera: slightly from front, slightly from side, slightly from above
  pg3d.camera(pg3d.width/2 + 50, pg3d.height/2 - 50, 300, pg3d.width/2, pg3d.height/2, 0, 0, 1, 0);
  
  pg3d.translate(pg3d.width/2, pg3d.height/2 + 100, 0);
  
  // Rotate the view so rocket stands vertical (rotate 90 degrees around X)
  pg3d.rotateX(HALF_PI);
  
  // Apply telemetry rotations (convert degrees to radians)
  pg3d.rotateZ(radians(-r));      // Roll
  pg3d.rotateY(radians(p));       // Pitch (swapped with yaw)
  pg3d.rotateX(radians(yaw));     // Yaw (swapped with pitch)
  
  // Draw coordinate axes for reference
  pg3d.strokeWeight(3);
  // X axis - Red
  pg3d.stroke(255, 0, 0);
  pg3d.line(0, 0, 0, 50, 0, 0);
  // Y axis - Green (reversed direction)
  pg3d.stroke(0, 255, 0);
  pg3d.line(0, 0, 0, 0, -50, 0);
  // Z axis - Blue
  pg3d.stroke(0, 0, 255);
  pg3d.line(0, 0, 0, 0, 0, 50);
  
  // Draw rocket model
  pg3d.scale(2);  // Scale rocket to appropriate size
  pg3d.shape(rocket);
  
  pg3d.endDraw();
  
  // Draw panel background
  noStroke();
  fill(50, 100, 150, 180);
  rect(x, y, w, h, panelRadius);
  stroke(255);
  strokeWeight(2);
  noFill();
  rect(x, y, w, h, panelRadius);
  
  // Draw title
  fill(255);
  textSize(25);
  textAlign(LEFT, CENTER);
  text("Attitude", x + 15 + textOffsetX, y + 25);
  
  // Draw the 3D graphics
  image(pg3d, x + 10, y + 50, w - 20, h - 60);
}

// --- Draw panels ---
void drawPanel(int x, int y, int w, int h, String title, String var1Name, float var1, String var2Name, float var2, String var3Name, float var3) {
  fill(50, 100, 200, 180);
  stroke(255);
  strokeWeight(2);
  rect(x, y, w, h, panelRadius);
  
  fill(255);
  textSize(25);
  text(title, x + 15 + textOffsetX, y + 25);
  
  textSize(16);
  text(var1Name + ": " + nf(var1,0,2), x + 15 + textOffsetX, y + 60);
  text(var2Name + ": " + nf(var2,0,2), x + 15 + textOffsetX, y + 90);
  text(var3Name + ": " + nf(var3,0,2), x + 15 + textOffsetX, y + 120);
}

void drawPanel(int x, int y, int w, int h, String title, String var1Name, float var1, String var2Name, float var2) {
  fill(50, 200, 100, 180);
  stroke(255);
  strokeWeight(2);
  rect(x, y, w, h, panelRadius);
  
  fill(255);
  textSize(25);
  text(title, x + 15 + textOffsetX, y + 25);
  
  textSize(16);
  text(var1Name + ": " + nf(var1,0,2), x + 15 + textOffsetX, y + 60);
  text(var2Name + ": " + nf(var2,0,2), x + 15 + textOffsetX, y + 90);
}

void drawPanel(int x, int y, int w, int h, String title, String var1Name, int var1) {
  fill(200, 100, 50, 180);
  stroke(255);
  strokeWeight(2);
  rect(x, y, w, h, panelRadius);
  
  fill(255);
  textSize(25);
  text(title, x + 15 + textOffsetX, y + 25);
  
  textSize(16);
  text(var1Name + ": " + var1, x + 15 + textOffsetX, y + 60);
}

// --- Serial parsing ---
void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if(line != null){
    line = trim(line);
    
    String[] fields = split(line, '\t');
    if(fields.length >= 16){
      om = parseInt(fields[1]);
      ts = parseFloat(fields[4]);
      ax = parseFloat(fields[5]);
      ay = parseFloat(fields[6]);
      az = parseFloat(fields[7]);
      gx = parseFloat(fields[8]);
      gy = parseFloat(fields[9]);
      gz = parseFloat(fields[10]);
      p  = parseFloat(fields[11]);
      yaw = parseFloat(fields[12]);
      r  = parseFloat(fields[13]);
      al = parseFloat(fields[14]);
      vz = parseFloat(fields[15]);
    }
  }
}

// --- Simple rectangle class for buttons ---
class Rectangle{
  int x, y, w, h;
  Rectangle(int x, int y, int w, int h){
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }
}
