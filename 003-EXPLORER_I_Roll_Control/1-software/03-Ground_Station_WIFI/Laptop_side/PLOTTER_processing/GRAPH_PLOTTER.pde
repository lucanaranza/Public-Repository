import processing.serial.*;

// --- SETTINGS ---
int plotWidth = 1920;
int plotHeight = 1080;
int bufferSize = 500;
int numChannels = 14;

// Channel names
String[] channelNames = {
  "Op Mode","Frequency","Timestamp",
  "ax","ay","az",
  "gx","gy","gz",
  "Pitch","Yaw","Roll",
  "Altitude","Vz"
};

// --- DATA ---
float[][] dataBuffers = new float[numChannels][bufferSize];
int index = 0;
float[] minValues = new float[numChannels];
float[] maxValues = new float[numChannels];
boolean[] activeChannels = new boolean[numChannels];
int[] channelColors;

// --- BUTTONS ---
float buttonX, buttonY;
float buttonWidth = 90, buttonHeight = 20, buttonSpacing = 35;

Serial myPort;

// --- RAW SERIAL MONITOR ---
ArrayList<String> rawLines = new ArrayList<String>();
int rawMaxLines = 15;

// --- RAW WINDOW POSITION ---
int rawWinX = 60;
int rawWinY = plotHeight - 225;
int rawWinW = 500;
int rawWinH = 200;

// --- SERIAL TX WINDOW ---
String txBuffer = "";
int txX = plotWidth - 500;
int txY = plotHeight - 180;
int txW = 360;
int txH = 30;
int txSendW = 70;

void settings() {
  size(plotWidth, plotHeight);
}

void setup() {
  textSize(14);

  // init buffers and channels
  for (int i = 0; i < numChannels; i++) {
    for (int j = 0; j < bufferSize; j++) dataBuffers[i][j] = 0;
    minValues[i] = Float.MAX_VALUE;
    maxValues[i] = -Float.MAX_VALUE;
    activeChannels[i] = false;
  }

  // HIGH-CONTRAST COLORS
  color[] distinctColors = {
    color(0, 100, 255),
    color(255, 140, 0),
    color(0, 200, 0),
    color(255, 0, 100),
    color(100, 0, 255),
    color(255, 255, 0),
    color(0, 255, 200),
    color(255, 80, 0),
    color(0, 255, 0),
    color(255, 0, 0),
    color(0, 0, 255),
    color(255, 200, 0),
    color(0, 150, 100),
    color(255, 0, 200)
  };

  channelColors = new int[numChannels];
  for (int i = 0; i < numChannels; i++) {
    channelColors[i] = distinctColors[i % distinctColors.length];
  }

  // Serial setup
  println(Serial.list());
  if (Serial.list().length > 0) {
    String portName = Serial.list()[0];
    myPort = new Serial(this, portName, 500000);
    myPort.bufferUntil('\n');
  }

  buttonX = plotWidth - 300;
  buttonY = 80;
}

void draw() {
  background(255);
  drawGrid();
  drawAxes();

  for (int ch = 0; ch < numChannels; ch++) {
    if (activeChannels[ch]) drawChannel(ch);
  }

  drawLegendAndButtons();
  drawRawSerialWindow();
  drawSerialTXWindow();
}

// --- GRID ---
void drawGrid() {
  stroke(200);
  for (int i = 0; i <= 10; i++) {
    float y = map(i, 0, 10, plotHeight-60-rawWinH, 60);
    line(60, y, plotWidth-450, y);
  }
  for (int i = 0; i <= 10; i++) {
    float x = map(i, 0, 10, 60, plotWidth-450);
    line(x, 60, x, plotHeight-60-rawWinH);
  }
}

// --- AXES ---
void drawAxes() {
  stroke(0);
  strokeWeight(2);
  line(60,60,60,plotHeight-60-rawWinH);
  line(60,plotHeight-60-rawWinH,plotWidth-450,plotHeight-60-rawWinH);
  strokeWeight(1);

  fill(0);
  textAlign(RIGHT, CENTER);

  float minV = 0, maxV = 1;
  for (int ch = 0; ch < numChannels; ch++) {
    if (activeChannels[ch]) {
      minV = minValues[ch];
      maxV = maxValues[ch];
      break;
    }
  }

  float yStepVal = (maxV - minV) / 10.0f;
  for (int i = 0; i <= 10; i++) {
    float y = map(i, 0, 10, plotHeight-60-rawWinH, 60);
    float val = minV + i*yStepVal;
    text(nfc(val, 2), 50, y);
  }

  textAlign(CENTER, TOP);
  for (int i = 0; i <= 10; i++) {
    float x = map(i, 0, 10, 60, plotWidth-450);
    int val = (int) map(i, 0, 10, 0, bufferSize-1);
    text(val, x, plotHeight-50-rawWinH);
  }

  text("Iteration Index", (plotWidth-450+60)/2, plotHeight-30-rawWinH);
}

// --- DRAW CHANNEL ---
void drawChannel(int ch) {
  stroke(channelColors[ch]);
  strokeWeight(2.8);   // <<< THICKER LINE
  noFill();
  beginShape();

  float minV = Float.MAX_VALUE;
  float maxV = -Float.MAX_VALUE;
  for (int i = 0; i < bufferSize; i++) {
    int bufIdx = (i + index) % bufferSize;
    float val = dataBuffers[ch][bufIdx];
    if (!Float.isNaN(val)) {
      if (val < minV) minV = val;
      if (val > maxV) maxV = val;
    }
  }
  if (minV == maxV) { minV -= 1; maxV += 1; }
  minValues[ch] = minV;
  maxValues[ch] = maxV;

  for (int i = 0; i < bufferSize; i++) {
    int bufIdx = (i + index) % bufferSize;
    float val = dataBuffers[ch][bufIdx];
    if (!Float.isNaN(val)) {
      float yVal = map(val, minV, maxV, plotHeight-60-rawWinH, 60);
      float xVal = map(i, 0, bufferSize-1, 60, plotWidth-450);
      vertex(xVal, yVal);
    }
  }
  endShape();
}

int legendOffsetX = 50;
int legendOffsetY = -30;

// --- LEGEND & BUTTONS ---
void drawLegendAndButtons() {
  textSize(12);
  int ly = (int) buttonY + legendOffsetY;
  for (int i = 0; i < numChannels; i++) {
    stroke(0);
    if (activeChannels[i]) fill(100,200,100);
    else fill(220);
    rect(buttonX-110 + legendOffsetX, ly-10, buttonWidth, buttonHeight, 3);

    stroke(channelColors[i]);
    line(buttonX + legendOffsetX, ly, buttonX+20 + legendOffsetX, ly);

    noStroke();
    fill(0);
    textAlign(LEFT, CENTER);
    text(channelNames[i], buttonX+25 + legendOffsetX, ly);

    fill(0);
    textAlign(CENTER, CENTER);
    text("Toggle", buttonX-60 + legendOffsetX, ly);

    ly += buttonSpacing;
  }
}

// --- RAW SERIAL WINDOW ---
void drawRawSerialWindow() {
  fill(30);
  stroke(0);
  rect(rawWinX, rawWinY, rawWinW, rawWinH);

  fill(0, 255, 0);
  textAlign(LEFT, TOP);
  textFont(createFont("Monospaced", 12));
  int lineHeight = 14;

  int maxLines = rawWinH / lineHeight;
  int start = max(0, rawLines.size() - maxLines);

  for (int i = start; i < rawLines.size(); i++) {
    String line = rawLines.get(i);
    while (textWidth(line) > rawWinW - 10 && line.length() > 0) {
      line = line.substring(0, line.length()-1);
    }
    text(line, rawWinX + 5, rawWinY + 5 + (i - start) * lineHeight);
  }
}

// --- SERIAL TX WINDOW ---
void drawSerialTXWindow() {
  fill(40);
  stroke(255);
  rect(txX, txY, txW, txH);

  fill(255);
  textAlign(LEFT, CENTER);
  text(txBuffer, txX + 5, txY + txH/2);

  fill(80,200,80);
  rect(txX + txW + 5, txY, txSendW, txH);

  fill(0);
  textAlign(CENTER, CENTER);
  text("Send", txX + txW + 5 + txSendW/2, txY + txH/2);
}

// --- MOUSE CLICK ---
void mousePressed() {

  // TX SEND BUTTON
  if (mouseX >= txX + txW + 5 && mouseX <= txX + txW + 5 + txSendW &&
      mouseY >= txY && mouseY <= txY + txH) {

    if (myPort != null) {
      myPort.write(txBuffer + "\n");
    }
    txBuffer = "";
  }

  int ly = (int) buttonY + legendOffsetY;
  for (int i = 0; i < numChannels; i++) {
    if (mouseX >= buttonX-110 + legendOffsetX && mouseX <= buttonX-110 + legendOffsetX + buttonWidth &&
        mouseY >= ly-10 && mouseY <= ly-10 + buttonHeight) {
      activeChannels[i] = !activeChannels[i];
    }
    ly += buttonSpacing;
  }
}

// --- TYPING INTO TX BOX ---
void keyTyped() {
  if (key == BACKSPACE) {
    if (txBuffer.length() > 0) txBuffer = txBuffer.substring(0, txBuffer.length()-1);
  } else if (key == ENTER || key == RETURN) {
    if (myPort != null) myPort.write(txBuffer + "\n");
    txBuffer = "";
  } else if (key != CODED) {
    txBuffer += key;
  }
}

// --- SERIAL EVENT ---
void serialEvent(Serial p) {
  String inData = p.readStringUntil('\n');
  if (inData == null) return;
  inData = trim(inData);
  if (inData.length() == 0) return;

  rawLines.add(inData);
  if (rawLines.size() > rawMaxLines) rawLines.remove(0);

  String[] tokens = split(inData, '\t');
  if (tokens.length < numChannels) return;

  for (int i = 0; i < numChannels; i++) {
    float val = parseFloat(tokens[i]);
    if (Float.isNaN(val)) val = 0;
    dataBuffers[i][index] = val;
  }
  index = (index + 1) % bufferSize;
}
