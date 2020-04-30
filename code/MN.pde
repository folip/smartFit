import processing.serial.*; //<>//
Serial ser;
long startTime = 0;
int times = 0;
// parameters for drawing three views
final int H = 320;
final int L = 240;
final int D = 480;
final int l = 100;
final float tr = 57.2958;
//angels read from MPU9250, vertexs to draw a matchstick men. 
float[][] angels;
int[][] vertexs;
//pressure val and curvature val read with BLE
//for osc
final int startX = 900;
int maxPreValue = 1024; // max value of pressure
boolean v = true;
final int maxCurValue = 1024; // max value of curvature
final int oscH = 240;
final int preLh = 280;
final int preLh2 = 600;
final int curLh = 920;
final int maxPoints = 400;
int pulse = 0;
boolean channel2 = false;

int preVal;
int preVal2;
int curVal;
int[] preVals;
int[] preVals2;
int[] curVals;
int preValIndex = 0; 
int preValIndex2 = 0;
int curValIndex = 0;

String mes = "";
boolean g = false;

PImage arm;
PImage chest;
PImage right;
PImage wrong;
PImage perf;

boolean armB = true;
boolean armA = true;
boolean chestA = true;
boolean chestS = true;
boolean chestL =  true;

int force = 1024;
int percent = 100;
int motionT = 0;
int test = 0;
int PX = 900;

int page = 0;

int X = 0;
int Y = 0;

void setup(){
 // size(2560,1440);
 // softBodyIntl();
  size(1480,1040);
  //setTV();
  //imageIntl();
  oscInit();
  //serialConnect();
}


void draw(){
  background(255);  
  //String message = readSerial();
 // parsing(message);
  //softBody();
  //drawTV();
  oscUpdate();
  //displayTime();
  //if(page == 0)  armPage();
  //else perfPage();
  //updateMA();
  //displayMessage();
}

void displayTime(){
  times+= 1;
  println("times: " + times);
  print((System.nanoTime() - startTime)/1000000 + "ms\n");
}
/*---------------------------------------BLE---------------------------------*/
void serialConnect(){
  print("avaliable serial ports:\n");
  printArray(Serial.list());
  print("connecting.....\n");
  ser = new Serial(this, "COM3", 9600); //BLE is on COM4 in my computer. 
  print("connected\n");
}

String readSerial(){
  int newLine = 13; // new line character in ASCII
  String message;
  do {
      message = ser.readStringUntil(newLine); // read from port until new line
      //print(message);
  } while (message == null);
  //print(message);
  //notice that there is a huge buffer for BLE!!! what you are reading is not what happens NOW!
  return message;
}

void parsing(String message){
  if(message != null){ //NULL null
    String[] list = split(trim(message),' ');
    //printArray(list);
    
    if(1 == list.length){
      mes = list[0]; 
      g = true;
      println(mes);
      if(mes.equals("channel_2_open")){ // == is different from equals!!
        channel2 = true;
        preValIndex2 = 0;
        mes = "channel 2 did open";  
        g = true;
      }
      else if(mes.equals("channel_2_closed")){
        channel2 = false;
        preValIndex2 = 0;
        mes = "channel 2 did close";
        g = true;
      }

      return;
    }
    
    if(6 <= list.length){
      angels[1][1] = -float(list[0]) / tr;
      angels[1][0] = -float(list[1]) / tr;
      angels[2][1] = -float(list[2]) / tr;
      angels[2][0] = -float(list[3]) / tr;
      
      curVal = int(list[4]);
      preVal = int(list[5]);
      /*
      curVal = int(list[4]);
      if(v){
        preVal = int(89724 / float(list[5]) - 107);
        if(float(list[5])>990) preVal = 0;
      }
      else{
        preVal = int(list[5]);
      }
      */
    }
    
    if(channel2 && list.length == 7) {
      if(v){
         preVal2 = int(89724 / float(list[6]) - 107);
        if(float(list[6])>990) preVal2 = 0;  
      }else{
        preVal2 = int(list[6]);
      }
    
    }
  }
}

void controlChar(char c){
  ser.write(c);
  mes = "control char sent: " + c;
  g = false;
}

void keyPressed() {
  delay(100);
  controlChar(key);
  pulse = 4;
  if('v' == key){v = true; maxPreValue = 300;}
  if('r' == key){v = false; maxPreValue = 1024;}
  /*
  switch(key){
    case 'o': println("o pressed");controlChar('o');break;
    case 'o': println("o pressed");controlChar('o');break;
    default: return;
  }
  */
  if('p' == key){printCur();}
  if('n' == key){page = 1;}
  if('f' == key){motionT = 4;}
}

void displayMessage(){
  textSize(20);
  textAlign(CENTER);
  if(g) fill(0,255,0);
  else fill(255,0,0);
  text(mes,450,900);
}

int[] motionSs;
long[] peakTs;
long[] motionTs;
int[] maxFs;
int maxF = 0;
long peakT = 0;
long preT;
int motionS = 0;
void updateMA(){
  if(angels[1][0] > (45/tr) || angels[1][0] < (-45/tr)) armA  = false;
  else armA = true;
  armB = true;
  println("arm angel "+angels[2][1]*57);
  maxF = max(maxF,force);
  percent = int(1.3 * (angels[2][1] * 57 + 180));
  println("percent: " + percent);
  if(percent <= 100) force = percent * 10;
  if(percent <= 100)motionS = max(percent,motionS);
  if(percent > 100) percent = 100;
  AT();
  peakT();
}

void peakT(){
  if(peakT != 0){
    if(angels[2][1] < 0  && (angels[2][1] * 57) < - 140){
      peakT = (System.nanoTime() - peakT) / 1000000;
      peakTs[motionT] = peakT;
      peakT = 0;
      mes = "";
    } 
  }
  else{
    if(angels[2][1] < 0 && (angels[2][1] * 57) > -140 ){
      peakT = System.nanoTime();
      mes = "peak";
      println("Im In");
      return;
    }
    println("peak still zero");
  }
}

int stateAT = 0;
// state: start from 0, 1 for l to h, 2 for h to l. update T whenever state transfers from 2 to 1.
void AT(){
  if(stateAT == 0){
    if(angels[2][1] < 0  && angels[2][1] < -170/tr) stateAT = 1;
    preT = System.nanoTime();
  }
  else if(stateAT == 1){
    if(angels[2][1] < 0  && angels[2][1] > -140/tr) stateAT = 2;
  }
  else if(stateAT == 2){
    if(angels[2][1] < 0  && angels[2][1] < -170/tr) {
      motionTs[motionT] = (System.nanoTime() - preT)/1000000;
      preT = System.nanoTime();
      motionSs[motionT] = motionS;
      motionS = 0;// record max movement percent 
      maxFs[motionT] = maxF;
      maxF = 0;
      motionT++;
      peakTs[motionT] = 0;
      stateAT = 1;
    }
  }
}


void pushValue(){
  
  curValIndex += 1; // move to the next position
  if(curValIndex >= maxPoints){
    curValIndex= 0;
  }
  curVals[curValIndex] = curVal; // and store the value;
  
  preValIndex += 1; // move to the next position
  if(preValIndex >= maxPoints){
    preValIndex= 0;
  }
  preVals[preValIndex] = preVal; // and store the value;
  
  if(channel2){
    preValIndex2 += 1; // move to the next position
    if(preValIndex2 >= maxPoints){
      preValIndex2= 0;
    }
    preVals2[preValIndex2] = preVal2; // and store the value;
  }  
}

void printCur(){
  printArray(maxFs);
  printArray(motionSs);
  printArray(motionTs);
  printArray(peakTs);
}

/*------------------------matchstick men model three views--------------------------------*/
void drawTV(){
  setVertexs();
  drawFrontView();
  drawSideView();
  drawTopView();
}

void setTV(){
  angels = new float[5][2]; // 0 for row and 1 for pitch
  vertexs = new int[7][3]; 
  
  angels[0][0] = 0 / tr;
  angels[0][1] = 20 / tr;
  
  angels[1][0] = 90 / tr;
  angels[1][1] = 0 / tr;
  
  angels[2][0] = 0 / tr;
  angels[2][1] = 0 / tr;
  
  angels[3][0] = -90 / tr;
  angels[3][1] = 0 / tr;
 
  angels[4][0] = 0 / tr;
  angels[4][1] = 0 / tr;
 
  vertexs[0][0] = 0;
  vertexs[0][1] = 0;
  vertexs[0][2] = 0; // 0 as end point of your upper body
}


void setVertexs(){
  for(int i = 1;i<6;i++){
    vertexs[i][0] = int(l * cos(angels[i-1][0]) * sin(angels[i-1][1])); 
    vertexs[i][1] = int(l * sin(angels[i-1][0]));
    vertexs[i][2] = int(l * cos(angels[i-1][0]) * cos(angels[i-1][1]));
  }
  //generate 5 vectors from angels representing your body and arm.
  
  vertexs[2][0] += vertexs[1][0];
  vertexs[2][1] += vertexs[1][1];
  vertexs[2][2] += vertexs[1][2];
  
  vertexs[3][0] += vertexs[2][0];
  vertexs[3][1] += vertexs[2][1];
  vertexs[3][2] += vertexs[2][2];
  //left arm
  vertexs[4][0] += vertexs[1][0];
  vertexs[4][1] += vertexs[1][1];
  vertexs[4][2] += vertexs[1][2];
  
  vertexs[5][0] += vertexs[4][0];
  vertexs[5][1] += vertexs[4][1];
  vertexs[5][2] += vertexs[4][2];
  //right arm
  vertexs[6][0] = int(1.4 * vertexs[1][0]);
  vertexs[6][1] = int(1.4 * vertexs[1][1]);
  vertexs[6][2] = int(1.4 * vertexs[1][2]);
}

void drawFrontView(){
  stroke(255,0,123);
  strokeWeight(5);
  line(L + vertexs[0][1], H - vertexs[0][2], L + vertexs[1][1],H - vertexs[1][2]);
  stroke(0);
  strokeWeight(8);
  line(L + 1.15 * vertexs[1][1], H - 1.15 * vertexs[1][2], L + vertexs[6][1],H - vertexs[6][2]);
  strokeWeight(5);
  stroke(123,0,255);
  line(L + vertexs[1][1], H - vertexs[1][2], L + vertexs[2][1],H - vertexs[2][2]);//left upper
  line(L + vertexs[1][1], H - vertexs[1][2], L + vertexs[4][1],H - vertexs[4][2]);//right upper
  stroke(123,233,255);
  line(L + vertexs[2][1], H - vertexs[2][2], L + vertexs[3][1],H - vertexs[3][2]);//left lower
  line(L + vertexs[4][1], H - vertexs[4][2], L + vertexs[5][1],H - vertexs[5][2]);//right lower
}

void drawSideView(){
  stroke(255,0,123);
  strokeWeight(5);
  line(L + D + vertexs[0][0], H - vertexs[0][2], L + D + vertexs[1][0],H - vertexs[1][2]);
  strokeWeight(7);
  stroke(50);
  line(L + D +  1.15 * vertexs[1][0], H - 1.15 * vertexs[1][2], L + D +  vertexs[6][0],H - vertexs[6][2]);//left upper
  strokeWeight(5);
  stroke(123,0,255);
  line(L + D +  vertexs[1][0], H - vertexs[1][2], L + D +  vertexs[2][0],H - vertexs[2][2]);//left upper
  line(L + D +  vertexs[1][0], H - vertexs[1][2], L + D +  vertexs[4][0],H - vertexs[4][2]);//right upper
  stroke(123,233,255);
  line(L + D + vertexs[2][0], H - vertexs[2][2], L + D +  vertexs[3][0],H - vertexs[3][2]);//left lower
  line(L + D + vertexs[4][0], H - vertexs[4][2], L + D +  vertexs[5][0],H - vertexs[5][2]);//right lower
}

void drawTopView(){
  stroke(255,0,123);
  strokeWeight(5);
  line(L + vertexs[0][1], L + H - vertexs[0][0], L + vertexs[1][1],L + H  - vertexs[1][0]);
  strokeWeight(7);
  stroke(50);
  line(L + 1.15 * vertexs[1][1], L + H - 1.15 * vertexs[1][0], L + vertexs[6][1],L + H - vertexs[6][0]);//left upper
  strokeWeight(5);
  stroke(123,0,255);
  line(L + vertexs[1][1], L + H - vertexs[1][0], L + vertexs[2][1],L + H - vertexs[2][0]);//left upper
  line(L + vertexs[1][1], L + H - vertexs[1][0], L + vertexs[4][1],L + H - vertexs[4][0]);//right upper
  stroke(123,233,255);
  line(L + vertexs[2][1], L + H - vertexs[2][0], L + vertexs[3][1],L + H - vertexs[3][0]);//left lower
  line(L + vertexs[4][1], L + H - vertexs[4][0], L + vertexs[5][1],L + H - vertexs[5][0]);//right lower
}

/*--------------------------------------------------------------------------------------osc------------------------------------------------------------------------------------*/
int getYP(int val){
  return (int)(preLh - oscH * val / maxPreValue);
}

int getYP2(int val){
  return (int)(preLh2 - oscH * val / maxPreValue);
}

int getYC(int val){
  return (int)(curLh - oscH * val / maxCurValue);
}

int t = 0;
void getValueT(){
  preVal = (int) (500 + 500 * sin(t/tr));
  preVal2 = (int) (500 + 500 * sin(2 * t/tr));
  curVal = (int) (500 + 500 * cos(t/tr));
  t++;
}

// todo: get value from BLE and store that in preVal


void drawPre(){
  stroke(112,0,225);
  strokeWeight(2);
  int k = 0;
  int x0 = startX;
  int y0 = getYP(preVals[k]);
  k++;
  for (;k<=preValIndex; k++) {
    int x1 = (int) (startX + k); // we may modify how x1 come from
    int y1 = getYP(preVals[k]);
    line(x0, y0, x1, y1); // the core of this program x0, y0 is the previous position.
    x0 = x1;
    y0 = y1; // store the damn thing each time
  }
  textSize(20);
  textAlign(CENTER);
  fill(255,0,0);
  text("pressure: " + preVal,startX + maxPoints / 2, preLh + 24);
  fill(255,0,0);
}

void drawPre2(){
  stroke(112,0,225);
  strokeWeight(2);
  int k = 0;
  int x0 = startX;
  int y0 = getYP2(preVals2[k]);
  k++;
  for (;k<=preValIndex2; k++) {
    int x1 = (int) (startX + k); // we may modify how x1 come from
    int y1 = getYP2(preVals2[k]);
    line(x0, y0, x1, y1); // the core of this program x0, y0 is the previous position.
    x0 = x1;
    y0 = y1; // store the damn thing each time
  }
  textSize(20);
  textAlign(CENTER);
  fill(255,0,0);
  text("pressure: " + preVal2,startX + maxPoints / 2, preLh2 + 24);
  fill(255,0,0);
}

void drawCur(){
  stroke(112,0,225);
  strokeWeight(2);
  int k = 0;
  int x0 = startX;
  int y0 = getYC(curVals[k]);
  k++;
  for (;k<=curValIndex; k++) {
    int x1 = (int) (startX + k); // we may modify how x1 come from
    int y1 = getYC(curVals[k]);
    line(x0, y0, x1, y1); // the core of this program x0, y0 is the previous position.
    x0 = x1;
    y0 = y1; // store the damn thing each time
  }
  textSize(20);
  textAlign(CENTER);
  text("curvature: " + curVal,startX + maxPoints / 2, curLh + 24);
  fill(255,0,0);
}

void drawGrid(){
  stroke(50);
  strokeWeight(2);
  line(startX, preLh, startX , preLh - oscH - 10);
  line(startX, preLh, startX + maxPoints + 2, preLh);
  line(startX, preLh2, startX , preLh2 - oscH - 10);
  line(startX, preLh2, startX + maxPoints + 2, preLh2);
  line(startX, curLh, startX + maxPoints + 2, curLh);
  line(startX, curLh, startX , curLh - oscH - 10);
  triangle(startX, preLh - oscH - 20, startX - 3, preLh - oscH - 10, startX + 3, preLh - oscH -10);
  triangle(startX, preLh2 - oscH - 20, startX - 3, preLh2 - oscH - 10, startX + 3, preLh2 - oscH -10);
  triangle(startX, curLh - oscH - 20, startX - 3, curLh - oscH - 10, startX + 3, curLh - oscH -10);
  triangle(startX + maxPoints + 12, preLh, startX + maxPoints + 2, preLh - 3, startX + maxPoints + 2, preLh + 3);
  triangle(startX + maxPoints + 12, preLh2, startX + maxPoints + 2, preLh2 - 3, startX + maxPoints + 2, preLh2 + 3);
  triangle(startX + maxPoints + 12, curLh, startX + maxPoints + 2, curLh - 3, startX + maxPoints + 2, curLh + 3);
  fill(50);
  
  stroke(220,220,220);
  strokeWeight(1);
  for(int i = 1; i < 4; i++){
    line(startX, preLh - oscH * i / 4, startX + maxPoints + 2, preLh - oscH * i / 4);
    line(startX, preLh2 - oscH * i / 4, startX + maxPoints + 2, preLh2 - oscH * i / 4);
    line(startX, curLh - oscH * i / 4, startX + maxPoints + 2, curLh - oscH * i / 4);
  }
  
  for(int i = 1; i < 5; i++){
     line(startX + maxPoints * i /5, preLh, startX + maxPoints * i /5 , preLh - oscH - 10);
     line(startX + maxPoints * i /5, preLh2, startX + maxPoints * i /5 , preLh2 - oscH - 10);
     line(startX + maxPoints * i /5, curLh, startX + maxPoints * i /5 , curLh - oscH - 10);
  }
  
  textSize(12);
  textAlign(RIGHT);
  for(int i = 0;i<=4;i++){
    text(maxPreValue * i / 4,startX - 6, preLh - oscH * i /4);
    text(maxPreValue * i / 4,startX - 6, preLh2 - oscH * i /4);
    text(maxCurValue * i / 4,startX - 6, curLh - oscH * i /4);
  }
  fill(50);
}

void oscInit(){
  preVals = new int[maxPoints];
  preVals2 = new int[maxPoints];
  curVals = new int[maxPoints];
  drawGrid();
  smooth();
}

void oscUpdate(){
  getValueT();
  pushValue();
  drawGrid();
  drawPre();
  if(channel2) drawPre2();
  drawCur();
}
/*--------------------------------------------------------------*/

void imageIntl(){
  arm = loadImage("arm.png");
  chest = loadImage("chest.png");
  right = loadImage("right.png");
  wrong = loadImage("wrong.png");
  perf = loadImage("perf.png");
  motionSs = new int[30];
  peakTs = new long[30];
  motionTs = new long[30];
  maxFs = new int[30];
}

void armPage(){
  image(arm,PX + 0,0,500,960);
  if(armB) right(PX + 82,312); else wrong(PX + 78,308);
  if(armA) right(PX + 392,234);else wrong(PX + 390,232);
  forceBar(PX + 250,640,force);
  percent(718,percent);
  Num(PX + 257,850,motionT);
}

void chestPage(){
  image(chest,PX + 0,0,500,960);
  if(chestA) right(PX + 72,366); else wrong(PX +72,366);
  if(chestS) right(PX + 386,188);else wrong(PX + 386,188);
  if(chestL) right(PX + 424,434);else wrong(PX + 424,434);
  forceBar(PX + 250,645,force);
  percent(748,percent);
  Num(PX + 257,880,motionT);
}

void perfPage(){
  image(perf,PX + 0,0);
}

void right(int x, int y){
  image(right,x,y,20,20);
}

void wrong(int x,int y){
  image(wrong, x,y,30,30);
}

void forceBar(int x,int y, int f){
  rectMode(CENTER);
  stroke(255);
  int colorRS = 0xE1;
  int colorGS = 0xEE;
  int colorBS = 0xC3;
  int colorRE = 0xF0;
  int colorGE = 0x50;
  int colorBE = 0x53;
  int dR = colorRE - colorRS; 
  int dG = colorGE - colorGS; 
  int dB = colorBE - colorBS; 
  fill(colorRS + dR * f / 1024,colorGS + dG * f / 1024,colorBS + dB * f / 1024);
  rect(x,y,200 * f / 1024, 20);
}

//60 + 400 * p/100
void percent(int y, int p){
  stroke(0xF6,0x5,0x53);
  fill(0xF6,0x5,0x53);
  triangle(60 + PX + 400 * p / 100 - 3, y - 10,60 + 400 * p / 100 + 3 + PX, y - 10,60 + 400 * p / 100 + PX, y + 10);
}

void Num(int x,int y, int num){
  stroke(0x06,0xF3,0X8C);
  fill(0x06,0xF3,0X8C);
  textSize(60);
  textAlign(CENTER);
  text(num,x,y);
}

/*-----------------------------softBody---------------------------------*/
float centerX = 0, centerY = 0;
int cor = 255;
float radius = 45, rotAngle = -90;
float accelX, accelY;
float springing = .0009, damping = .98;

//corner nodes
int nodes = 5;
float nodeStartX[] = new float[nodes];
float nodeStartY[] = new float[nodes];
float[]nodeX = new float[nodes];
float[]nodeY = new float[nodes];
float[]angle = new float[nodes];
float[]frequency = new float[nodes];

// soft-body dynamics
float organicConstant = 1;

void softBody(){
  fill(0, 100);
  rect(0,0,width, height);
  updateXY();
  drawShape();
  moveShape();
  softMes();
}

void softBodyIntl(){
  centerX = width/2;
  centerY = height/2;
  // iniitalize frequencies for corner nodes
  for (int i=0; i<nodes; i++){
    frequency[i] = random(5, 12);
  }
  noStroke();
  frameRate(30);
}

void drawShape() {
  //  calculate node  starting positions
  for (int i=0; i<nodes; i++){
    nodeStartX[i] = centerX+cos(radians(rotAngle))*radius;
    nodeStartY[i] = centerY+sin(radians(rotAngle))*radius;
    rotAngle += 360.0/nodes;
  }

  // draw polygon
  curveTightness(organicConstant);
  fill(255,cor,cor);
  beginShape();
  for (int i=0; i<nodes; i++){
    curveVertex(nodeX[i], nodeY[i]);
  }
  for (int i=0; i<nodes-1; i++){
    curveVertex(nodeX[i], nodeY[i]);
  }
  endShape(CLOSE);
}

void moveShape() {
  //move center point
  float deltaX = X-centerX;
  float deltaY = Y-centerY;

  // create springing effect
  deltaX *= springing;
  deltaY *= springing;
  accelX += deltaX;
  accelY += deltaY;

  // move predator's center
  centerX += accelX;
  centerY += accelY;

  // slow down springing
  accelX *= damping;
  accelY *= damping;

  // change curve tightness
  organicConstant = 1-((abs(accelX)+abs(accelY))*.1);

  //move nodes
  for (int i=0; i<nodes; i++){
    nodeX[i] = nodeStartX[i]+sin(radians(angle[i]))*(accelX*2);
    nodeY[i] = nodeStartY[i]+sin(radians(angle[i]))*(accelY*2);
    angle[i]+=frequency[i];
  }
}

void updateXY(){
  X = int(width/2 - width/2.5 * sin(angels[2][0]));
  Y = int(height/2 - height/2.5 * cos(angels[2][0]) * cos(angels[2][1]));
  mes = "pre: "+ (1024 - preVal) + "angels: "+ 57 * angels[2][0] + " "+ 57 * angels[2][1] + " X: " + X + " Y: " + Y ;
  cor = int(255 - (1024 - float(preVal)) / 1024 * 255);
}

void softMes(){
  textSize(20);
  textAlign(CENTER);
  fill(255);
  text(mes,1280,1000);
}
