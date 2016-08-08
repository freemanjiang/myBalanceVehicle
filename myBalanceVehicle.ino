#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <PID_v1.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

double Setpoint, Input, Output;
double Setpoint_offset;
PID myPID(&Input, &Output, &Setpoint, 50, 200, 0.22, DIRECT);
double countR = 0;
double countL = 0;

#define MPU_6050_INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards// On Pro Micro, move it to pin 7.
#define WHEEL_R_PCINT_PIN 8
#define WHEEL_L_PCINT_PIN 15
#define LED_PIN 14 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void wheelR(void)
{
  if (Output < 0)
  {
    countR--;
  }
  else
  {
    countR++;
  }
}

void wheelL(void)
{
  if (Output < 0)
  {
    countL--;
  }
  else
  {
    countL++;
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(5, OUTPUT); //WHEEL R
  pinMode(6, OUTPUT); //WHEEL R
  pinMode(10, OUTPUT);//WHEEL L
  pinMode(9, OUTPUT); //WHEEL L

  pinMode(WHEEL_R_PCINT_PIN, INPUT_PULLUP);//one wheel speed detect , Pin Change Interrupt
  pinMode(WHEEL_L_PCINT_PIN, INPUT_PULLUP);//another wheel speed detect , Pin Change Interrupt

  attachPCINT(digitalPinToPCINT(WHEEL_R_PCINT_PIN), wheelR, RISING);
  attachPCINT(digitalPinToPCINT(WHEEL_L_PCINT_PIN), wheelL, RISING);

  Setpoint = 0;
  Setpoint_offset = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);//PID 采样周期 10ms
  myPID.SetOutputLimits(-255, 255);


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  delay(200);
  mpu.initialize();
  pinMode(MPU_6050_INTERRUPT_PIN, INPUT);

  delay(3000);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
#if 0
  mpu.setXGyroOffset(-138);
  mpu.setYGyroOffset(-81);
  mpu.setZGyroOffset(-34);
  mpu.setXAccelOffset(1608);
  mpu.setYAccelOffset(-2089);
  mpu.setZAccelOffset(200 + 2048);
  //mpu.setZAccelOffset(200);
#else
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
#endif

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_6050_INTERRUPT_PIN), dmpDataReady, RISING);//pin 2, dmp detect Angle
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

unsigned long lastt1 = 0;
unsigned long lastRpmMeasure = 0;
double lastCount10ms = 0;
double lastCountR = 0;
double lastCountL = 0;
int in = 0;
double rpmR = 0;
double rpmL = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //    Serial.print("!");
    //    Serial.print(ypr[0] * 180 / M_PI);
    //    Serial.print("#");
    //    Serial.print(ypr[1] * 180 / M_PI);
    //    Serial.print("#");
    //    Serial.println(ypr[2] * 180 / M_PI);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  //motors control
  if (millis() - lastt1 >= 10)//10ms 获取一次Input,并更新Setpoint
  {
    lastt1 = millis();
    Input = countR - lastCount10ms;
    lastCount10ms = countR;
    Serial.print("I=");
    Serial.print(Input);
    Serial.print("\tS=");
    in = analogRead(A0);//电位器输入 Setpoint
    //in = map(in, 0, 1023, -13, 13);//10ms内，电机最大转速，码盘最多能产生13个脉冲
    in = map(ypr[1] * 180 / M_PI, -20, 20, -20, 20);
    Setpoint = in + Setpoint_offset;
    Serial.print(Setpoint);
    Serial.print("\tO=");
    Serial.print(Output);
    Serial.print("\trpmR=");
    Serial.println(rpmR);
  }
  if (millis() - lastRpmMeasure >= 500)//500ms 做一次RPM计算
  {
    lastRpmMeasure = millis();
    rpmR = (countR - lastCountR) * 24;
    rpmL = (countL - lastCountL) * 24;
    lastCountR = countR;
    lastCountL = countL;
  }

  myPID.Compute();

  if (Output < 0)
  {
    analogWrite(5, LOW);
    analogWrite(6, -Output);

    analogWrite(10, LOW);
    analogWrite(9, -Output);
  }
  else
  {
    analogWrite(5, Output);
    analogWrite(6, LOW);

    analogWrite(10, Output);
    analogWrite(9, LOW);
  }
}
