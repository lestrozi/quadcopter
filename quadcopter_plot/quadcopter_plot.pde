import processing.serial.*;

Serial myPort;        // The serial port
int curPos = 0;
float roll;
float pitch;
float yaw;

float rSize;  // rectangle size

void setup() {
  size(640, 360, P3D);
  rSize = width / 6;  
  noStroke();
  fill(204, 204);
  
  
  // List all the available serial ports
  println(Serial.list());
 
  myPort = new Serial(this, Serial.list()[4], 57600);
  
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');
}

String getNextValue(String s) {
  int start = s.indexOf("=", curPos);
  int end = s.indexOf(",", curPos);
  
  if (start == -1)
    return null;
  
  if (end == -1)
    end = s.length()-1;
    
  curPos = end+1;
    
  return s.substring(start+1, end);
}

void serialEvent (Serial port) {
  try {
    String s = port.readStringUntil('\n');
  
    if (s == null)
      return;
      
    if (s.charAt(0) != '#')
      return;
  
    curPos = 0;
    
    String v;
    v = getNextValue(s);  //yaw
    yaw = Float.parseFloat(v);
    
    v = getNextValue(s);  //pitch
    pitch = Float.parseFloat(v);
    
    v = getNextValue(s);  //roll
    roll = Float.parseFloat(v);
  
    //print("r=", roll, "p=", pitch, "r=", yaw, "\n");
    //print(".");
  }
  catch(Exception e) {
    e.printStackTrace();
  }
}

void draw() {
    background(126);
    
    translate(width/2, height/2);
    
    rotateX(3.1415926535897/2.0);
    
    rotateX(-roll);
    rotateY(-pitch);
    rotateZ(-yaw);
    
    fill(0);
    rect(-rSize, -rSize, rSize*2, rSize*2);
}

