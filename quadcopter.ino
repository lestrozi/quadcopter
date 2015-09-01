#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#include <Servo.h>
#include <PID_v1.h>


//#define ARM_MOTORS

#define DEBUG
////#define DEBUG_DONT_INIT_IMU
////#define SAFETY_START
////#define CONSTANT_CALIBRATING
//#define WAIT_RADIO
#define WAIT_STABLE_YAW
#define DEBUG_NO_YAW_CONTROL


#ifdef DEBUG
#define D(X) X
#else
#define D(X)
#endif


#define RUNLOOPEACH_MS 20

// pins
#define INT 2    // sendo usado por interrupcao do IMU
#define LED 13

// esc
#define ESC1 3  // green
#define ESC2 5  // yellow
#define ESC3 6  // white
#define ESC4 9  // gray

// channels
#define NUM_CHANNELS 6
#define CH1 4    // amarelo    2,LR
#define CH2 8    // verde      1,UD
#define CH3 10   // azul       2,UD
#define CH4 11   // roxo       1,LR
#define CH5 12   // marrom     button
#define CH6 A0   // branco     button
//#define CH7 A1

#define YAWDELTA 0.0001


// imu
bool dmpReady = false; // set true if DMP init was successful
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
float lastYaw;

// motor
Servo escs[4];
int escs_speed[4];
extern void setSpeed(Servo, int);


// PID
double PID_roll_input, PID_roll_output, PID_roll_setpoint=0;
double PID_pitch_input, PID_pitch_output, PID_pitch_setpoint=0;
double PID_yaw_input, PID_yaw_output, PID_yaw_setpoint=0;

double PID_roll_Kp=300, PID_roll_Ki=20, PID_roll_Kd=10;
double PID_pitch_Kp=300, PID_pitch_Ki=20, PID_pitch_Kd=10;
double PID_yaw_Kp=150, PID_yaw_Ki=10, PID_yaw_Kd=5;
//double PID_roll_Kp=600, PID_roll_Ki=0, PID_roll_Kd=0;
//double PID_pitch_Kp=600, PID_pitch_Ki=0, PID_pitch_Kd=0;
//double PID_yaw_Kp=600, PID_yaw_Ki=0, PID_yaw_Kd=0;

PID PID_roll(&PID_roll_input, &PID_roll_output, &PID_roll_setpoint, PID_roll_Kp, PID_roll_Ki, PID_roll_Kd, DIRECT);
PID PID_pitch(&PID_pitch_input, &PID_pitch_output, &PID_pitch_setpoint, PID_pitch_Kp, PID_pitch_Ki, PID_pitch_Kd, DIRECT);
PID PID_yaw(&PID_yaw_input, &PID_yaw_output, &PID_yaw_setpoint, PID_yaw_Kp, PID_yaw_Ki, PID_yaw_Kd, DIRECT);


// radio
volatile boolean lostSignal = true;
int throttle=0;
int roll=0;
int pitch=0;
int yaw=0;
int ch5int=0;
int ch6int=0;
boolean ch5=false;
boolean ch6=false;


boolean mpuCalled = false;
boolean pidStarted = false;
boolean radioMinSeem = false;
boolean yawStable = false;

void setup() {
  D(Serial.begin(57600);)
  D(while(!Serial);)
  pinMode(LED, OUTPUT);

  initRadio();

  #ifdef WAIT_RADIO  
  waitRadio();
  #endif

  D(Serial.println("initializing motors");)
  initMotors();

  #ifdef CALIBRATE_MOTORS
  calibrateAll();
  #endif
}


void waitRadio() {
  delay(350);    //calibrating

  while(lostSignal) {
    setSpeedAll(0);
    
    digitalWrite(LED, digitalRead(LED));

    D(Serial.println("WAITING FOR INITIAL RADIO SIGNAL");)
    delay(100);
  }
  
  readRadio();
  
  if (throttle > 900) {    // be careful because chmax starts on 1800!
    D(Serial.println("SKIPPING RADIO MIN!");)
    radioMinSeem = true;
  }
}

void readRadio() {
  //noInterrupts();
  
//  if (isUpdatedValue(1))
    throttle = getNormalizedRadioValue(1);
    
//  if (isUpdatedValue(0))
    roll = getNormalizedRadioValue(0);

//  if (isUpdatedValue(2))
    pitch = getNormalizedRadioValue(2);

//  if (isUpdatedValue(3))
    yaw = getNormalizedRadioValue(3);

//  if (isUpdatedValue(4))
    ch5int = getNormalizedRadioValue(4);

//  if (isUpdatedValue(5))
    ch6int = getNormalizedRadioValue(5);
  
//  clearUpdates();
  
  //interrupts();


  ch5 = (ch5int > 500);
  ch6 = (ch6int > 500);
}

void printMpu() {
  Serial.print("#yaw=");
  Serial.print(ypr[0]/* * 180/M_PI*/, 6);
  Serial.print(", pitch=");
  Serial.print(ypr[1]/* * 180/M_PI*/, 6);
  Serial.print(", roll=");
  Serial.print(ypr[2]/* * 180/M_PI*/, 6);
  Serial.println();
}

void printRadio() {
  Serial.print("throttle=");
  Serial.print(throttle);
  Serial.print(",\troll=");
  Serial.print(roll);
  Serial.print(",\tpitch=");
  Serial.print(pitch);
  Serial.print(",\tyaw=");
  Serial.print(yaw);
  Serial.print(",\t5=");
  Serial.print(ch5);
  Serial.print(",\t6=");
  Serial.print(ch6);
  Serial.println();
}

void printESCs() {
  D(Serial.print("\tesc_y1=");)
  D(Serial.print(escs_speed[0]);)
  D(Serial.print("\tesc_x2=");)
  D(Serial.print(escs_speed[1]);)
  D(Serial.print("\tesc_y2=");)
  D(Serial.print(escs_speed[2]);)
  D(Serial.print("\tesc_x1=");)
  D(Serial.print(escs_speed[3]);)
  D(Serial.println();)
}

void failSafe() {
  static long lastRun = millis();
  
  if (millis() - lastRun < 100)
    return;
  
  D(Serial.println("FAILSAFE!!");)

  //FIXME: a linha abaixo eh a correta, mas:
  //1) ainda nao precisa disso
  //2) ele ta pegando o ULTIMO throttle (ruido), que vem ANTES de detectar o failsafe
  //throttle *= 0.97;

  throttle = 0;
  roll = 500;
  pitch = 500;
  yaw = 500;
    
  lastRun = millis();
}

void suicide() {
  D(Serial.println("suicide");)
  
  for(int i=0; i<4; i++)
    setSpeed(escs[i], 0);

  delay(1000000000);
}

int yawCount = 0;
void loop() {
  long timeRun = millis();
  
  if (timeRun > 45000)
    suicide();

  if (!pidStarted) {
    initPID();
    pidStarted = true;
    return;
  }
  
  #ifndef DEBUG_DONT_INIT_IMU
  if (!mpuCalled) {
    D(Serial.println("initializing MPU");)
    initMpu();
    mpuCalled = true;
    return;
  }
  #endif

  // read radio
  if (lostSignal) {
    failSafe();
  }
  else {
    readRadio();
    //D(printRadio();)
    
    if (ch5) {    // STABLE MODE
      roll = 500;
      pitch = 500;
      yaw = 500;
    }

    if (!radioMinSeem) {
      if (throttle < 150) {
        radioMinSeem = true;
        return;
      }
  
      throttle = 0;
    }
  }

  if (throttle < 150)
    throttle = 0;
  else if (throttle < 250)
    throttle = (throttle - 150) * 2.5;  // slow rampup

  readMpu();

  #ifdef WAIT_STABLE_YAW  
  if (!yawStable) {
    if (abs(ypr[0] - lastYaw) < YAWDELTA)
      yawCount++;
    else
      yawCount = 0;

    if (yawCount > 10) {
      yawStable = true;
    }
    else {
      lastYaw = ypr[0];
      throttle = 0;
    }
  }
  #endif

  //D(printMpu();)

  PID_roll_input = ypr[2];
  PID_roll_setpoint = (roll - 500)/500.0 * 0.60;
  PID_roll.Compute();
  PID_pitch_input = -ypr[1];
  PID_pitch_setpoint = (pitch - 500)/500.0 * 0.60;
  PID_pitch.Compute();
  PID_yaw_input = ypr[0];
  PID_yaw_setpoint += (yaw - 500)/500.0 * 0.01;    // depends on how fast the loop runs
  while(PID_yaw_setpoint > PI)
    PID_yaw_setpoint -= 2*PI;
  while(PID_yaw_setpoint < -PI)
    PID_yaw_setpoint += 2*PI;
  PID_yaw.Compute();

  PIDoutputToSpeed(escs_speed, throttle, PID_roll_output, PID_pitch_output, PID_yaw_output);
  
  for(int i=0; i<4; i++)
    setSpeed(escs[i], escs_speed[i]);

  D(printESCs();)

  delay(max(0, RUNLOOPEACH_MS - (millis() - timeRun)));
}

