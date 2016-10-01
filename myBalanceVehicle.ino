//L298N 模块供电电源12V， L298N的5V输出作为其他电路逻辑电源。
#include <EEPROM.h>

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

struct ControlParams {
  double kp;
  double ki;
  double kd;
  double sp;
  double si;
  double sd;
  double angle_calibration;
  int speedPidOn;//1:on   0:off
  unsigned int dead_zone;
};

double Setpoint, Input, Output;
double Setpoints, Inputs, Outputs;
int velocity = 0;
unsigned int dead_zone = 20;
int finalOutput;
unsigned int show1 = 1;
unsigned int show2 = 1;

double kp = 10, ki = 0, kd = 0.15;
double sp = 0.55, si = 0.25, sd = 0.0;
double angle_calibration = -0.4;
ControlParams ctlparams = {
  kp, ki, kd,
  sp, si, sd,
  angle_calibration,
  1, dead_zone
};
float value = 0.0;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
PID myPID(&Input, &Output,   &Setpoint, ctlparams.kp, ctlparams.ki, ctlparams.kd, DIRECT);
PID sPID(&Inputs, &Outputs, &Setpoints, ctlparams.sp, ctlparams.si, ctlparams.sd, REVERSE);
int countL = 0;
int countR = 0;

#define MPU_6050_INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards// On Pro Micro, move it to pin 7.
#define WHEEL_L_PCINT_PIN_A 8
#define WHEEL_L_PCINT_PIN_B 14
#define WHEEL_R_PCINT_PIN_A 15
#define WHEEL_R_PCINT_PIN_B 16

#define WHEEL_L_IN1 A1
#define WHEEL_L_IN2 A0
#define WHEEL_L_PWM 5
#define WHEEL_R_IN1 9
#define WHEEL_R_IN2 10
#define WHEEL_R_PWM 6

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

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

/////////////////begin Encoder/////////////////////
typedef struct {
  volatile uint8_t       pinA;
  volatile uint8_t       pinB;
  uint8_t                state;
  int32_t                position;
} Encoder_internal_state_t;

#define ENCODER_ARGLIST_SIZE 4
class Encoder
{
  public:
    Encoder(uint8_t pin1, uint8_t pin2) {
      pinMode(pin1, INPUT_PULLUP);
      pinMode(pin2, INPUT_PULLUP);
      encoder.pinA = pin1;
      encoder.pinB = pin2;
      encoder.position = 0;
      // allow time for a passive R-C filter to charge
      // through the pullup resistors, before reading
      // the initial state
      delayMicroseconds(2000);
      uint8_t s = 0;
      if (digitalRead(encoder.pinA) & 0x1) s |= 1;
      if (digitalRead(encoder.pinB) & 0x2) s |= 2;
      encoder.state = s;
      attach_interrupt(pin1, &encoder);
      attach_interrupt(pin2, &encoder);
    }

    inline int32_t read() {
      noInterrupts();
      int32_t ret = encoder.position;
      interrupts();
      return ret;
    }
    inline void write(int32_t p) {
      noInterrupts();
      encoder.position = p;
      interrupts();
    }

  private:
    Encoder_internal_state_t encoder;
  public:
    static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];
    //                           _______         _______
    //               Pin1 ______|       |_______|       |______ Pin1
    // negative <---         _______         _______         __      --> positive
    //               Pin2 __|       |_______|       |_______|   Pin2

    //  new   new   old   old
    //  pin2  pin1  pin2  pin1  Result
    //  ----  ----  ----  ----  ------
    //  0   0   0   0   no movement
    //  0   0   0   1   +1
    //  0   0   1   0   -1
    //  0   0   1   1   +2  (assume pin1 edges only)
    //  0   1   0   0   -1
    //  0   1   0   1   no movement
    //  0   1   1   0   -2  (assume pin1 edges only)
    //  0   1   1   1   +1
    //  1   0   0   0   +1
    //  1   0   0   1   -2  (assume pin1 edges only)
    //  1   0   1   0   no movement
    //  1   0   1   1   -1
    //  1   1   0   0   +2  (assume pin1 edges only)
    //  1   1   0   1   -1
    //  1   1   1   0   +1
    //  1   1   1   1   no movement

    // update() is not meant to be called from outside Encoder,
    // but it is public to allow static interrupt routines.
    // DO NOT call update() directly from sketches.
    static void update(Encoder_internal_state_t *encoder) {
      uint8_t s = encoder->state & 3;
      if (digitalRead(encoder->pinA)) s |= 4;
      if (digitalRead(encoder->pinB)) s |= 8;
      switch (s) {
        case 0: case 5: case 10: case 15:
          break;
        case 1: case 7: case 8: case 14:
          encoder->position++; break;
        case 2: case 4: case 11: case 13:
          encoder->position--; break;
        case 3: case 12:
          encoder->position += 2; break;
        default:
          encoder->position -= 2; break;
      }
      encoder->state = (s >> 2);
    }

  private:
    static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state)
    {
      switch (pin) {
        case WHEEL_L_PCINT_PIN_A:
          interruptArgs[0] = state;
          attachPCINT(digitalPinToPCINT(pin), isr0, CHANGE);//here using Pin Change Interrupt
          break;
        case WHEEL_L_PCINT_PIN_B:
          interruptArgs[1] = state;
          attachPCINT(digitalPinToPCINT(pin), isr1, CHANGE);
          break;
        case WHEEL_R_PCINT_PIN_A:
          interruptArgs[2] = state;
          attachPCINT(digitalPinToPCINT(pin), isr2, CHANGE);
          break;
        case WHEEL_R_PCINT_PIN_B:
          interruptArgs[3] = state;
          attachPCINT(digitalPinToPCINT(pin), isr3, CHANGE);
          break;
        default:
          return 0;
      }
      return 1;
    }
    static void isr0(void)
    {
      update(interruptArgs[0]);
    }
    static void isr1(void)
    {
      update(interruptArgs[1]);
    }
    static void isr2(void)
    {
      update(interruptArgs[2]);
    }
    static void isr3(void)
    {
      update(interruptArgs[3]);
    }
};
Encoder_internal_state_t* Encoder::interruptArgs[] = {NULL, NULL, NULL, NULL};
Encoder myEncL(WHEEL_L_PCINT_PIN_A, WHEEL_L_PCINT_PIN_B);
Encoder myEncR(WHEEL_R_PCINT_PIN_A, WHEEL_R_PCINT_PIN_B);
/////////////////end of Encoder////////////////////


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  EEPROM.get(0, ctlparams);

  pinMode(WHEEL_L_IN1, OUTPUT); //WHEEL L
  pinMode(WHEEL_L_IN2, OUTPUT); //WHEEL L
  pinMode(WHEEL_L_PWM, OUTPUT); //WHEEL L enable, pwm

  pinMode(WHEEL_R_IN1, OUTPUT);//WHEEL R
  pinMode(WHEEL_R_IN2, OUTPUT); //WHEEL R
  pinMode(WHEEL_R_PWM, OUTPUT); //WHEEL R enable, pwm

  //  pinMode(WHEEL_L_PCINT_PIN_A, INPUT_PULLUP);// wheel speed detect , Pin Change Interrupt
  //  pinMode(WHEEL_L_PCINT_PIN_B, INPUT_PULLUP);// wheel speed detect , Pin Change Interrupt
  //  pinMode(WHEEL_R_PCINT_PIN_A, INPUT_PULLUP);// wheel speed detect , Pin Change Interrupt
  //  pinMode(WHEEL_R_PCINT_PIN_B, INPUT_PULLUP);// wheel speed detect , Pin Change Interrupt

  //  attachPCINT(digitalPinToPCINT(WHEEL_L_PCINT_PIN_A), wheelL, CHANGE);
  //  attachPCINT(digitalPinToPCINT(WHEEL_L_PCINT_PIN_B), wheelL, CHANGE);
  //  attachPCINT(digitalPinToPCINT(WHEEL_R_PCINT_PIN_A), wheelR, CHANGE);
  //  attachPCINT(digitalPinToPCINT(WHEEL_R_PCINT_PIN_B), wheelR, CHANGE);

  Setpoint = 0;
  myPID.SetTunings(ctlparams.kp, ctlparams.ki, ctlparams.kd);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(5);//PID 采样周期 5ms
  myPID.SetMode(AUTOMATIC);

  sPID.SetTunings(ctlparams.sp, ctlparams.si, ctlparams.sd);
  sPID.SetOutputLimits(-190, 190);//范围小于Setpoint的范围，否则，速度环积分饱和后，无法通过Setpoint来消除积分。
  sPID.SetSampleTime(50);
  sPID.SetMode(AUTOMATIC);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial1.begin(115200);
  inputString.reserve(200);
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

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_6050_INTERRUPT_PIN), dmpDataReady, RISING);//pin 2, dmp detect Angle
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
  }
}
unsigned long lastVelocityMesure = 0;
unsigned long lastShowState = 0;
unsigned long lastParamShow = 0;
int lastCountR = 0;
int lastCountL = 0;
double angle = 0;
int CountDeltaR = 0;
int CountDeltaL = 0;

void loop() {
  //get yaw pitch roll
  if (!dmpReady)
  {
    Serial1.println("dmp Not Ready");
    return;
  }
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial1.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle = -(ypr[1] * 180 / M_PI - ctlparams.angle_calibration);
  }

  //motors control
  if (millis() - lastVelocityMesure >= 50)
  {
    lastVelocityMesure = millis();
    countR = myEncR.read();
    countL = myEncL.read();
    CountDeltaR = countR - lastCountR;
    lastCountR = countR;
    CountDeltaL = countL - lastCountL;
    lastCountL = countL;
  }
  if (millis() - lastShowState >= 500)
  {
    lastShowState = millis();
    if (show1)
    {
      Serial1.print(Setpoints);
      Serial1.print("\tO=");
      Serial1.print(Output);
      Serial1.print("\tI=");
      Serial1.print(Input);//angle
      Serial1.print("\tis=");
      Serial1.print(Inputs);
      Serial1.print("\tos=");
      Serial1.print(Outputs);
      Serial1.print("\tfo=");
      Serial1.println(finalOutput);
    }
  }
  if (millis() - lastParamShow >= 3000)//3000ms
  {
    lastParamShow = millis();
    if (show2)
    {
      EEPROM.get(0, ctlparams);
      Serial1.print(ctlparams.kp, 5);
      Serial1.print(',');
      Serial1.print(ctlparams.ki, 5);
      Serial1.print(',');
      Serial1.print(ctlparams.kd, 5);
      Serial1.print('\t');
      Serial1.print(ctlparams.sp, 5);
      Serial1.print(',');
      Serial1.print(ctlparams.si, 5);
      Serial1.print(',');
      Serial1.print(ctlparams.sd, 5);
      Serial1.print('\t');
      Serial1.print(ctlparams.angle_calibration);
      Serial1.print('\t');
      Serial1.println(ctlparams.speedPidOn);
      Serial1.print('\t');
      Serial1.println(ctlparams.dead_zone);
    }
  }

  Inputs = CountDeltaR + CountDeltaL;
  Input = angle;
  Setpoints = velocity;
  Setpoint = 0;

  if (angle > -60 && angle < 60)
  {
    if (ctlparams.speedPidOn)
    {
      sPID.Compute();
    }
    else
    {
      Outputs = 0;
    }

    myPID.Compute();
    //  Serial.print(Output);
    //  Serial.print("+");
    //  Serial.print(Output);

    finalOutput = Output + Outputs;

    //Motor's PWM dead zone
    if (finalOutput > 0)
    {
      finalOutput += ctlparams.dead_zone;
    }
    else if (Output < 0)
    {
      finalOutput -= ctlparams.dead_zone;
    }

    finalOutput = constrain(finalOutput, -255, 255);
  }
  else// (angle > 60 || angle < -60)
  { //倒下后电机停转，并且不计算PID（此状态下PID不积分）
    finalOutput = 0;
  }

  //  Serial.print("=");
  //  Serial.println(finalOutput);
  if (finalOutput < 0)
  { //backward
    digitalWrite(WHEEL_L_IN1, LOW);
    digitalWrite(WHEEL_L_IN2, HIGH);
    analogWrite(WHEEL_L_PWM, -finalOutput);

    digitalWrite(WHEEL_R_IN1, HIGH);
    digitalWrite(WHEEL_R_IN2, LOW);
    analogWrite(WHEEL_R_PWM, -finalOutput);
  }
  else
  { //forwared
    digitalWrite(WHEEL_L_IN1, HIGH);
    digitalWrite(WHEEL_L_IN2, LOW);
    analogWrite(WHEEL_L_PWM, finalOutput);

    digitalWrite(WHEEL_R_IN1, LOW);
    digitalWrite(WHEEL_R_IN2, HIGH);
    analogWrite(WHEEL_R_PWM, finalOutput);
  }

  if (stringComplete) {
    Serial1.println(inputString);
    value = inputString.substring(inputString.indexOf('=') + 1).toFloat();
    if (inputString.startsWith("kp="))
    {
      ctlparams.kp = value;
    }
    else if (inputString.startsWith("ki="))
    {
      ctlparams.ki = value;
    }
    else if (inputString.startsWith("kd="))
    {
      ctlparams.kd = value;
    }
    else if (inputString.startsWith("sp="))
    {
      ctlparams.sp = value;
    }
    else if (inputString.startsWith("si="))
    {
      ctlparams.si = value;
    }
    else if (inputString.startsWith("sd="))
    {
      ctlparams.sd = value;
    }
    else if (inputString.startsWith("cal="))//setpoint offset
    {
      ctlparams.angle_calibration = value;
    }
    else if (inputString.startsWith("spid="))//setpoint offset
    {
      ctlparams.speedPidOn = (int)value;
    }
    else if (inputString.startsWith("dz="))//setpoint offset
    {
      ctlparams.dead_zone = (unsigned int)value;
    }
    else if (inputString.startsWith("show1="))//setpoint offset
    {
      if ((unsigned int)value == 1)
      {
        show1 = 1;
      }
      else
      {
        show1 = 0;
      }
    }
    else if (inputString.startsWith("show2="))//setpoint offset
    {
      if ((unsigned int)value == 1)
      {
        show2 = 1;
      }
      else
      {
        show2 = 0;
      }
    }
    else if (inputString.startsWith("v+"))//setpoint offset
    {
      velocity += 5;
    }
    else if (inputString.startsWith("v-"))//setpoint offset
    {
      velocity -= 5;
    }
    else if (inputString.startsWith("stop"))//setpoint offset
    {
      velocity = 0;
    }
    else if (inputString.startsWith("kp+"))//setpoint offset
    {
      ctlparams.kp += 0.5;
    }
    else if (inputString.startsWith("kp-"))//setpoint offset
    {
      ctlparams.kp -= 0.5;
    }
    else if (inputString.startsWith("ki+"))//setpoint offset
    {
      ctlparams.ki += 2;
    }
    else if (inputString.startsWith("ki-"))//setpoint offset
    {
      ctlparams.ki -= 2;
    }
    else if (inputString.startsWith("kd+"))//setpoint offset
    {
      ctlparams.kd += 0.02;
    }
    else if (inputString.startsWith("kd-"))//setpoint offset
    {
      ctlparams.kd -= 0.02;
    }
    else if (inputString.startsWith("sp+"))//setpoint offset
    {
      ctlparams.sp += 0.05;
    }
    else if (inputString.startsWith("sp-"))//setpoint offset
    {
      ctlparams.sp -= 0.05;
    }
    else if (inputString.startsWith("si+"))//setpoint offset
    {
      ctlparams.si += 0.002;
    }
    else if (inputString.startsWith("si-"))//setpoint offset
    {
      ctlparams.si -= 0.002;
    }
    else if (inputString.startsWith("sd+"))//setpoint offset
    {
      ctlparams.sd += 0.001;
    }
    else if (inputString.startsWith("sd-"))//setpoint offset
    {
      ctlparams.sd -= 0.001;
    }

    // clear the string:
    inputString = "";
    stringComplete = false;

    myPID.SetTunings(ctlparams.kp, ctlparams.ki, ctlparams.kd);
    sPID.SetTunings(ctlparams.sp, ctlparams.si, ctlparams.sd);
    EEPROM.put(0, ctlparams);
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
