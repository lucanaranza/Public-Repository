import processing.serial.*;

Serial myPort;

// --- SETTINGS ---
int plotWidth = 1600;   // ⬅ Increased window width (was 1600)
int plotHeight = 900;
int bufferSize = 500;
int numChannels = 24;

// Channel names
String[] channelNames = {
  "Op Mode","Frequency Hz","Timestamp",
  "ax","ay","az",
  "gx","gy","gz",
  "mx","my","mz",
  "Pitch","Yaw","Roll",
  "vx","vy","vz",
  "Altitude","Temperature",
  "Lat","Lng","Alt GPS","Vel GPS"
};

// --- DATA ---
float[][] dataBuffers = new float[numChannels][bufferSize];
int index = 0;
float[] minValues = new float[numChannels];
float[] maxValues = new float[numChannels];
boolean[] activeChannels = new boolean[numChannels];
color[] channelColors;

// --- BUTTONS ---
float buttonX, buttonY;
float buttonWidth = 90, buttonHeight = 20, buttonSpacing = 35;

void settings() {
  size(plotWidth, plotHeight);
}

void setup() {
  textSize(14);

  // init buffers and channels
  for (int i=0; i<numChannels; i++) {
    for (int j=0; j<bufferSize; j++) dataBuffers[i][j] = 0;
    minValues[i] = Float.MAX_VALUE;
    maxValues[i] = -Float.MAX_VALUE;
    activeChannels[i] = false; // all off by default
  }

  // assign colors
  channelColors = new color[numChannels];
  for (int i=0; i<numChannels; i++) {
    channelColors[i] = lerpColor(color(0,0,255), color(255,0,0), i/(float)numChannels);
  }

  // Serial setup
  println(Serial.list());
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');

  // Move buttons further to the right to use new space
  buttonX = plotWidth - 300; // ⬅ Previously -80
  buttonY = 80;
}

void draw() {
  background(255); // white
  drawGrid();
  drawAxes();

  for (int ch=0; ch<numChannels; ch++) {
    if (activeChannels[ch]) drawChannel(ch);
  }

  drawLegendAndButtons();
}

// --- GRID ---
void drawGrid() {
  stroke(200);
  for (int i=0; i<=10; i++) {
    float y = map(i,0,10,plotHeight-60,60);
    line(60,y,plotWidth-450,y);  // ⬅ Graph ends earlier (more space for legend)
  }
  for (int i=0; i<=10; i++) {
    float x = map(i,0,10,60,plotWidth-450);
    line(x,60,x,plotHeight-60);
  }
}

// --- AXES ---
void drawAxes() {
  stroke(0);
  strokeWeight(2);
  line(60,60,60,plotHeight-60); // Y-axis
  line(60,plotHeight-60,plotWidth-450,plotHeight-60); // X-axis ⬅ updated limit

  // arrows
  line(60,60,55,70);
  line(60,60,65,70);
  line(plotWidth-450,plotHeight-60,plotWidth-460,plotHeight-65);
  line(plotWidth-450,plotHeight-60,plotWidth-460,plotHeight-55);

  strokeWeight(1);

  // Y axis labels for each horizontal grid line
  fill(0);
  textAlign(RIGHT,CENTER);

  // get current active channel range
  float minV = 0, maxV = 1;
  for (int ch=0; ch<numChannels; ch++) {
    if (activeChannels[ch]) {
      minV = minValues[ch];
      maxV = maxValues[ch];
      break;
    }
  }
  float yStepVal = (maxV - minV) / 10.0;
  for (int i=0; i<=10; i++) {
    float y = map(i,0,10,plotHeight-60,60);
    float val = minV + i*yStepVal;
    text(nfc(val,2), 50, y);
  }

  // X axis labels for each vertical grid line
  textAlign(CENTER,TOP);
  for (int i=0; i<=10; i++) {
    float x = map(i,0,10,60,plotWidth-450);
    int val = (int) map(i, 0, 10, 0, bufferSize-1);
    text(val, x, plotHeight-50);
  }

  // X axis title
  text("Iteration Index", (plotWidth-450+60)/2, plotHeight-30);
}

// --- DRAW CHANNEL ---
void drawChannel(int ch) {
  stroke(channelColors[ch]);
  noFill();
  beginShape();

  float minV = Float.MAX_VALUE;
  float maxV = -Float.MAX_VALUE;
  for (int i=0;i<bufferSize;i++){
    int bufIdx=(i+index)%bufferSize;
    float val = dataBuffers[ch][bufIdx];
    if (val<minV) minV=val;
    if (val>maxV) maxV=val;
  }
  if (minV==maxV){ minV-=1; maxV+=1; }
  minValues[ch]=minV;
  maxValues[ch]=maxV;

  for (int i=0; i<bufferSize; i++) {
    int bufIdx = (i + index) % bufferSize;
    float yVal = map(dataBuffers[ch][bufIdx], minV, maxV, plotHeight-60,60);
    float xVal = map(i,0,bufferSize-1,60,plotWidth-450);
    vertex(xVal, yVal);
  }
  endShape();
}

// --- OFFSETS ---
int legendOffsetX = 50;  // right shift
int legendOffsetY = -30; // upward shift

// --- LEGEND & BUTTONS ---
void drawLegendAndButtons() {
  textSize(12);
  int ly = (int) buttonY + legendOffsetY;
  for (int i=0; i<numChannels; i++) {
    // button rectangle
    stroke(0);
    if (activeChannels[i]) fill(100,200,100);
    else fill(220);
    rect(buttonX-110 + legendOffsetX, ly-10, buttonWidth, buttonHeight, 3);

    // legend line
    stroke(channelColors[i]);
    line(buttonX + legendOffsetX, ly, buttonX+20 + legendOffsetX, ly);
    
    // legend text aligned with button
    noStroke();
    fill(0);
    textAlign(LEFT, CENTER);
    text(channelNames[i], buttonX+25 + legendOffsetX, ly);

    // button text
    fill(0);
    textAlign(CENTER, CENTER);
    text("Toggle", buttonX-60 + legendOffsetX, ly);

    ly += buttonSpacing;
  }
}

// --- MOUSE CLICK ---
void mousePressed() {
  int ly = (int) buttonY + legendOffsetY;
  for (int i=0; i<numChannels; i++) {
    if (mouseX >= buttonX-110 + legendOffsetX && mouseX <= buttonX-110 + legendOffsetX + buttonWidth &&
        mouseY >= ly-10 && mouseY <= ly-10 + buttonHeight) {
      activeChannels[i] = !activeChannels[i];
    }
    ly += buttonSpacing;
  }
}



// --- SERIAL EVENT ---
void serialEvent(Serial p) {
  String inData = p.readStringUntil('\n');
  if (inData == null) return;
  inData = trim(inData);
  if (inData.length()==0) return;

  String[] tokens = split(inData,'\t');
  if (tokens.length<numChannels) return;

  for (int i=0;i<numChannels;i++){
    float val=float(tokens[i]);
    dataBuffers[i][index]=val;
  }
  index=(index+1)%bufferSize;
}
