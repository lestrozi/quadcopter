//TODO: possible bugs: overflow
//TODO: feature: auto-calibrate IMU?

MPU6050 mpu;
bool blinkState = false;

uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector


void dmpDataReady() {
  mpuInterrupt = true;
}

void initMpu() {
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  mpu.initialize();
  
  D(Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));)
  D(Serial.println(F("Initializing DMP..."));)
  devStatus = mpu.dmpInitialize();  

  mpu.setXGyroOffset(129);
  mpu.setYGyroOffset(-23);
  mpu.setZGyroOffset(24);
  mpu.setXAccelOffset(-1174);
  mpu.setYAccelOffset(575);
  mpu.setZAccelOffset(1718);

/*
  mpu.setXGyroOffset(127);
  mpu.setYGyroOffset(-15);
  mpu.setZGyroOffset(-60);
  mpu.setXAccelOffset(-1250);
  mpu.setYAccelOffset(589);
  mpu.setZAccelOffset(1674);
*/  

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    D(Serial.println("OK");)
  } else {
    D(Serial.print(F("DMP Initialization failed (code "));)
    D(Serial.print(devStatus);)
    D(Serial.println(F(")"));)
  }
}

void readMpu() {
  fifoCount = 0;
  mpu.resetFIFO();

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
//    mpu.dmpGetEuler(ypr, &q);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

