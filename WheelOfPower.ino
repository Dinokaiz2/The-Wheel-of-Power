// LoRa plan
// get an idea of how often corruption happen and what they look like (print recvs)
// enableCrc and mess with it until errors look like they decrease
// monitor speed during this, see if time gets increased
// if errors can definitely be 100% eliminated, switch to binary

#include <SparkFun_LIS331.h>
#include <LoRa.h>
#include <SPI.h>
#include <TimerOne.h>
#include <stdlib.h>

// #define MOTOR_DEBUG
// #define LORA_DEBUG
// #define ACCEL_DEBUG

#define LORA_CS 4
#define LORA_RST 3
#define LORA_DIO0 2
#define LORA_SYNC_WORD 0xF3

LIS331 accel;
#define ACCEL_CS 7

#define MOSI 11
#define MISO 12
#define SCK 13

#define MOTOR_L 9
#define MOTOR_R 10

#define MELTY_LED_PIN 8

#define MOTOR_FREQ 4000.0
#define MOTOR_PERIOD (1 / MOTOR_FREQ * 1000000) // 250 us
#define MOTOR_RES 10

#define NO_PWM 0
#define MELTY 1
#define TANK 2

#define NEUTRAL_THROTTLE 753

#define ACCELEROMETER_RADIUS 0.06033

// angle between LED and actual front of the robot
#define MELTY_LED_OFFSET -0.5497787 // radians (CCW is +)

typedef struct {
  int16_t throttle;
  float angle;
  float angularVel;
  long lastMeltyFrameTime; // us
  bool attacking;
  int attackMod;
  float radiusTrim;
  uint8_t mode;
} RobotState;

typedef struct {
  unsigned long timestamp;
  float leftX;
  float leftY;
  float rightX;
  bool leftBumper;
  bool rightBumper;
  bool dpadLeft;
  bool dpadRight;
  bool dpadUp;
  bool dpadDown;
  bool aButton;
  bool bButton;
  bool xButton;
  bool yButton;
} ControllerState;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelState;

void setup()
{
  #if defined(LORA_DEBUG) || defined(ACCEL_DEBUG) || defined(MOTOR_DEBUG)
    Serial.begin(115200);
  #endif
  initMotors();
  initAccelerometer();
  initLoRa();
  pinMode(MELTY_LED_PIN, OUTPUT);
}

void loop()
{
  static ControllerState* controllerState = calloc(1, sizeof(ControllerState));
  static ControllerState* lastControllerState = calloc(1, sizeof(ControllerState));
  static AccelState* accelState = calloc(1, sizeof(AccelState));
  static RobotState* robotState = calloc(1, sizeof(RobotState));
  
  accelRecv(accelState);
  getControllerState(&controllerState, &lastControllerState);

  if (controllerState->bButton) robotState->mode = TANK;
  else if (controllerState->aButton || controllerState->bButton) robotState->mode = MELTY;
  else if (controllerState->yButton) robotState->mode = NO_PWM;

  if (robotState->mode == TANK) curvatureDrive(controllerState);
  else if (robotState->mode == MELTY) meltyDrive(robotState, controllerState, lastControllerState, accelState);
  else if (robotState->mode == NO_PWM) digitalWrite(MELTY_LED_PIN, HIGH);
}

void initMotors()
{
  Timer1.initialize(MOTOR_PERIOD);
  // Timer1.pwm(MOTOR_R, NEUTRAL_THROTTLE); // TODO: add back in?
  // Timer1.pwm(MOTOR_L, NEUTRAL_THROTTLE);
  #ifdef MOTOR_DEBUG
    Serial.println("Motors initialized");
  #endif // MOTOR_DEBUG
}

void initAccelerometer()
{
  pinMode(ACCEL_CS, OUTPUT);
  digitalWrite(ACCEL_CS, HIGH);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  SPI.begin();
  accel.setSPICSPin(ACCEL_CS);
  while (!accel.begin(LIS331::USE_SPI)) {
    #ifdef ACCEL_DEBUG
      Serial.println("Accelerometer initialization failed. Retrying...");
    #endif // ACCEL_DEBUG
  }
  accel.setODR(accel.DR_1000HZ);
  accel.setFullScale(accel.HIGH_RANGE);
  #ifdef ACCEL_DEBUG
    Serial.println("Accelerometer initialized");
  #endif // ACCEL_DEBUG
}

void initLoRa()
{
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  #ifdef LORA_DEBUG
    Serial.print("LoRa initializing");
  #endif // LORA_DEBUG
  while (!LoRa.begin(915E6)) {
    #ifdef LORA_DEBUG 
      Serial.print("."); // TODO: LED code for LoRa failure?
    #endif // LORA_DEBUG
    delay(500);
  }
  LoRa.setTimeout(5); // TODO: might be making every loop w/ new packet take 5ms longer
  #ifdef LORA_DEBUG
    Serial.println("\nLoRa initialized");
  #endif // LORA_DEBUG
}

/**
 * TODO: figure out errors: LoRa.enableCrc(), i think crc might be off by default, do we need to check for crc ok ourselves?
 * TODO: improve speed: reduce packet size, learn about and jiggle bandwidth, spreading factor
 *    on frames we get packets, the frame takes ~20ms, normal frames are ~4ms
 * @return the packet if there was a new one, the empty string otherwise
 */
String loRaRecv()
{
  #ifdef LORA_DEBUG
    long recvStart = micros();
  #endif
  int packetSize = LoRa.parsePacket();
  String packetStr = "";
  if (packetSize) {
    #ifdef LORA_DEBUG
      Serial.print("Received packet: \"");
    #endif
    while (LoRa.available()) packetStr = LoRa.readString(); // TODO: switch to read() into char buffer, gets rid of setTimeout stuff
    #ifdef LORA_DEBUG
      static long lastPacketTime = millis();
      Serial.print(packetStr);
      Serial.print("\", RSSI: ");
      Serial.print(LoRa.packetRssi());
      Serial.print(" dBm, Rx time: ");
      Serial.print(micros() - recvStart);
      Serial.print(" us, Since last: ");
      Serial.print(millis() - lastPacketTime);
      Serial.println("ms");
      lastPacketTime = millis();
    #endif // LORA_DEBUG
  }
  return packetStr;
}

bool loRaAvailable()
{
  int packetSize = LoRa.parsePacket(); // TODO: learn about this and why we need it?
  if (packetSize) return true;
  else return false;
}

void accelRecv(AccelState* accelState)
{
  accel.readAxes(accelState->x, accelState->y, accelState->z);
  #ifdef ACCEL_DEBUG
    Serial.print("X: ");
    Serial.print(accelState->x);
    Serial.print("\tY: ");
    Serial.print(accelState->y);
    Serial.print("\tZ: ");
    Serial.print(accelState->z);
    Serial.print("\tflags: ");
  #endif
}

// TODO: call loRaRecv(), which should read a binary packet into a char buffer
// then pass that sequence to parseControllerState() which puts it in a ControllerState
void getControllerState(ControllerState** controllerState, ControllerState** lastControllerState)
{
  // TODO: probably worth making sure that this isn't copying more than i expect
  // TODO: i could probably make this faster if i don't worry about memory use so much
  //       idea: memcopy controllerState to lastControllerState, then read into controllerState
  //              - no more memcmp
  //              - still have to copy memory, but have to anyway
  if (loRaAvailable()) {
    parseControllerState(*lastControllerState);
    ControllerState* temp = *controllerState;
    *controllerState = *lastControllerState;
    *lastControllerState = temp;
  } else if (memcmp(*controllerState, *lastControllerState, sizeof(ControllerState)) != 0)
    memcpy(*lastControllerState, *controllerState, sizeof(ControllerState));
}

/**
 * Parses a packet from LoRa and reads it into the specified ControllerState.
 * Precondition: packet waiting in LoRa buffer
 */
void parseControllerState(ControllerState* controllerState)
{
  // TODO: switch to binary
  // TODO: deadzone
  while (LoRa.available()) {
    if (LoRa.read() == 'z') {
      // Serial.println("Got a packet");
      controllerState->timestamp = millis();
      controllerState->leftX = LoRa.parseFloat();
      controllerState->leftY = LoRa.parseFloat();
      controllerState->rightX = LoRa.parseFloat();
      controllerState->leftBumper = LoRa.parseInt() ? true : false;
      controllerState->rightBumper = LoRa.parseInt() ? true : false;
      controllerState->aButton = LoRa.parseInt() ? true : false;
      controllerState->bButton = LoRa.parseInt() ? true : false;
      controllerState->xButton = LoRa.parseInt() ? true : false;
      controllerState->yButton = LoRa.parseInt() ? true : false;
      // controllerState->dpadLeft = LoRa.parseInt();
      // controllerState->dpadRight = LoRa.parseInt();
      // controllerState->dpadUp = LoRa.parseInt();
      // controllerState->dpadDown = LoRa.parseInt();
    }
  }
}

// TODO: Optimize
// Benchmark, make changes from float to integer math
void meltyDrive(RobotState* robotState, ControllerState* controllerState, ControllerState* lastControllerState, AccelState* accelState) {
  if (controllerState->rightBumper && !lastControllerState->rightBumper) {
    robotState->throttle += 5;
    robotState->attacking = false;
  }
  if (controllerState->leftBumper && !lastControllerState->leftBumper) {
    robotState->throttle -= 5;
    robotState->attacking = false;
  }
  robotState->throttle = constrain(robotState->throttle, NEUTRAL_THROTTLE, 1023);

  if (controllerState->aButton) {
    robotState->throttle = NEUTRAL_THROTTLE;
    robotState->attacking = false;
  } else if (controllerState->xButton) {
    robotState->throttle = 788; // good power (tested with 50-250)
    robotState->attacking = false;
  } else if (controllerState->yButton) {
    robotState->throttle = 1023; // max power
    robotState->attacking = true;
  } else if (!controllerState->yButton && lastControllerState->yButton) {
    robotState->throttle = 788; // good power (tested with 50-250)
    robotState->attacking = false;
  }

  // if (trim == 1 && lastTrim != 1) radiusTrim += 0.0005;
  // else if (trim == -1  && lastTrim != -1) radiusTrim -= 0.0005;
  // if (trimReset == 1 || trimReset == -1) radiusTrim = 0;
  // lastTrim = trim;

  // TODO: move to controller parsing, we only need to do this on new packet
  float joystickAngle = atan2(controllerState->leftX, -controllerState->leftY);
  float joystickMagnitude = constrain(sqrt(controllerState->leftX * controllerState->leftX
                                          + controllerState->leftY * controllerState->leftY), 0, 1);

  // log time
  long deltaTime = micros() - robotState->lastMeltyFrameTime;
  robotState->lastMeltyFrameTime = micros();
  if (deltaTime > 500000) {
    // just switched to melty mode, bad frame
    deltaTime = 0;
    robotState->angle = PI * 0.5;
    robotState->throttle = NEUTRAL_THROTTLE;
  }

  // calculate change in angle
  float cenAcc = accelUnitsToMS2(sqrt(fabs((long)accelState->x * (long)accelState->x + (long)accelState->y * (long)accelState->y)));
  float angularVel = sqrt(fabs(cenAcc / (ACCELEROMETER_RADIUS))); //  + robotState->radiusTrim)));
  float deltaAngle = ((angularVel + robotState->angularVel) * 0.5) * (deltaTime * 0.000001);
  robotState->angle += deltaAngle;
  robotState->angularVel = angularVel;

  if (fabs(controllerState->rightX) > 0.1)
    robotState->angle += (-controllerState->rightX * 2 * PI) * (deltaTime * 0.000001);

  float angleDifference = fabs(fabs(fmod(robotState->angle, 2 * PI) - joystickAngle) - PI);

  // Serial.println("xAccel:  " + String(xAccel) + "\ttrim:  " + String(radiusFudge, 5) + "\tthrottle:  " + String(throttle));

  robotState->throttle = constrain(robotState->throttle, NEUTRAL_THROTTLE, 1023);
  // draw arc
  if (fmod(robotState->angle, 2.0 * PI) > (1.25 * PI) + MELTY_LED_OFFSET && fmod(robotState->angle, 2.0 * PI) < (1.75 * PI) + MELTY_LED_OFFSET) digitalWrite(MELTY_LED_PIN, HIGH);
  else digitalWrite(MELTY_LED_PIN, LOW);

  // set motor speed
  if (millis() - controllerState->timestamp > 1000) {
    robotState->throttle = NEUTRAL_THROTTLE;
    setMotorsMelty(robotState->throttle, robotState->throttle);
  } else {
    if (fabs(controllerState->leftX) < 0.1 && fabs(controllerState->leftY) < 0.1) {
      setMotorsMelty(robotState->throttle, robotState->throttle);
    } else {
      int deflection = ((robotState->throttle - NEUTRAL_THROTTLE) * fmap(joystickMagnitude, 0, 1, 0, 0.5));
      if (angleDifference > PI * 0.5) setMotorsMelty(robotState->throttle + (deflection * 3), robotState->throttle - deflection);
      else setMotorsMelty(robotState->throttle - deflection, robotState->throttle + (deflection * 3));
    }
  }
}

// NEUTRAL_THROTTLE...1023
void setMotorsMelty(int leftPower, int rightPower) {
  Timer1.pwm(MOTOR_L, constrain(leftPower, NEUTRAL_THROTTLE, 1023));
  Timer1.pwm(MOTOR_R, constrain(rightPower, NEUTRAL_THROTTLE, 1023));
}

float quickStopThreshold = 0.2;
float quickStopAlpha = 0.1;
float quickStopAccumulator;
float invertedReverse = false;
float quickTurnThreshold = 0.2;

void curvatureDrive(ControllerState* controllerState) {
  float angularPower;
  boolean overPower;
  float throttle = -controllerState->leftY;
  float rotation = controllerState->rightX;
  // rotation *= 0.5;
  if (fabs(throttle) < 0.1) throttle = 0;
  if (fabs(rotation) < 0.1) rotation = 0;
  if (throttle < quickTurnThreshold) {
      if (fabs(throttle) < quickStopThreshold)
          quickStopAccumulator = (1 - quickStopAlpha) * quickStopAccumulator + quickStopAlpha * rotation * 2;
      overPower = true;
      angularPower = rotation;
  } else {
      overPower = false;
      angularPower = fabs(throttle) * rotation - quickStopAccumulator;
  }
  if (quickStopAccumulator > 1) quickStopAccumulator--;
  else if (quickStopAccumulator < -1) quickStopAccumulator++;
  else quickStopAccumulator = 0;
  float leftPower;
  float rightPower;
  if (throttle < 0 && invertedReverse) {
      leftPower = throttle - angularPower;
      rightPower = throttle + angularPower;
  } else {
      leftPower = throttle + angularPower;
      rightPower = throttle - angularPower;
  }
  if (throttle < 0 && invertedReverse) {
      leftPower = -leftPower;
      rightPower = -rightPower;
  }
  if (overPower) {
      if (leftPower > 1) {
          rightPower -= leftPower - 1;
          leftPower = 1;
      } else if (rightPower > 1) {
          leftPower -= rightPower - 1;
          rightPower = 1;
      } else if (leftPower < -1) {
          rightPower -= leftPower + 1;
          leftPower = -1;
      } else if (rightPower < -1) {
          leftPower -= rightPower + 1;
          rightPower = -1;
      }
  }
  float maxMagnitude = max(fabs(leftPower), fabs(rightPower));
  if (maxMagnitude > 1) {
      leftPower /= maxMagnitude;
      rightPower /= maxMagnitude;
  }
  if (fabs(leftPower) < 0.05) leftPower = 0.05;
  if (fabs(rightPower) < 0.05) rightPower = 0.05;
  Timer1.pwm(MOTOR_L, (int)fmap(leftPower, -1, 1, 713, 793));
  Timer1.pwm(MOTOR_R, (int)fmap(rightPower, -1, 1, 793, 713));

  Serial.print("l: ");
  Serial.print(fmap(leftPower, -1, 1, 713, 793));
  Serial.print(", r: ");
  Serial.println(fmap(rightPower, -1, 1, 793, 713));
}

float accelUnitsToMS2(float nativeUnits) {
  return ((400.0 * (float)nativeUnits) / 2047) * 9.80665;
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}