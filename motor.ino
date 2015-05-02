void initMotors() {
  pinMode(ESC1, OUTPUT);
  pinMode(ESC2, OUTPUT);
  pinMode(ESC3, OUTPUT);
  pinMode(ESC4, OUTPUT);
  
  escs[0].attach(ESC1);
  escs[1].attach(ESC2);
  escs[2].attach(ESC3);
  escs[3].attach(ESC4);
}

void setSpeed(Servo esc, int val) {
  val = map(val, 0, 1000, 1000, 2000);
  esc.writeMicroseconds(val);
}

void setSpeedAll(int val) {
  for(int i=0; i<4; i++)
    setSpeed(escs[i], val);
}

void calibrateAll() {
  setSpeedAll(1000);
  delay(5000);

  setSpeedAll(0);    delay(2000);
/*
  setSpeedAll(200);  delay(2000);
  setSpeedAll(0);    delay(2000);
  setSpeedAll(500);  delay(2000);
  setSpeedAll(0);    delay(2000);
  setSpeedAll(1000); delay(2000);
  setSpeedAll(0);
*/
  delay(10000000);
}

void armAll() {
  for(int i=0; i<4; i++)
    setSpeed(escs[i], 1000);

  delay(2000);
 
  for(int i=0; i<4; i++)
    setSpeed(escs[i], 0);

  delay(4000);
}

void safetyStart() {
  for(int i=0; i<4; i++)
    setSpeed(escs[i], 70);
    
  delay(180);
  
  for(int i=0; i<4; i++)
    setSpeed(escs[i], 0);

  delay(1000);
}

