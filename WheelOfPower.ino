#include <Wire.h>
#include <BluetoothSerial.h>

// SCL to 22, SDA to 21
#define ADDR_ACCEL 0x19
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28

#define STOP 0
#define TANK 1
#define MELTY 2
#define ESTOP 3

int leftMotorPin = 27;
int rightMotorPin = 23;
int leftPwmChannel = 0;
int rightPwmChannel = 1;
int ledPin = 32;
int powerLedPin = 33;

int throttleFreq = 4000;
int throttleResolution = 10; // TODO: find max resolution

float throttle = 753;

double angularVel = 0;
double lastAngularVel = 0;
double angle = 0;
long currentTime;
long lastTime;

int state = STOP;
int lastState = STOP;

 // if the led drifts with spin, decrease radius
#define ACCELEROMETER_RADIUS 0.05647 // meters
float radiusFudge = 0;

BluetoothSerial bluetooth;
float xAxis, yAxis, rotation, throttleUp, throttleDown, lastThrottleUp, lastThrottleDown, trim, lastTrim, trimReset, attack, hardStop, goodPower, tankButton;
bool attacking, wasAttacking;

long lastTimePacketReceived = 0;

void setup() {
  Serial.begin(115200);
  bluetooth.begin("Wheel of Power");
  Wire.begin();

  // set up oneshot125
  ledcSetup(leftPwmChannel, throttleFreq, throttleResolution);
  ledcSetup(rightPwmChannel, throttleFreq, throttleResolution);
  ledcAttachPin(leftMotorPin, leftPwmChannel);
  ledcAttachPin(rightMotorPin, rightPwmChannel);
  
  // 50Hz: 0x27, 100Hz: 0x2F, 400Hz: 0x37, 1000Hz: 0x3F
  writeI2CReg8Blocking(ADDR_ACCEL, CTRL_REG1, 0x3F);

  // enables block data update
  // 100g: 0x80, 200g: 0x90, 400g: 0xB0
  writeI2CReg8Blocking(ADDR_ACCEL, CTRL_REG4, 0xB0);

  pinMode(ledPin, OUTPUT);
  pinMode(powerLedPin, OUTPUT);
  digitalWrite(powerLedPin, HIGH);

  state = STOP;
}

void loop() {
  while (bluetooth.available() > 0) {
    if ((bluetooth.read()) == 'z') {
        lastTimePacketReceived = millis();
        xAxis = bluetooth.parseFloat();
        yAxis = -bluetooth.parseFloat();
        rotation = bluetooth.parseFloat();
        //state = bluetooth.parseFloat();
        throttleUp = bluetooth.parseFloat();
        throttleDown = bluetooth.parseFloat();
        trim = bluetooth.parseFloat();
        trimReset = bluetooth.parseFloat();
        attack = bluetooth.parseFloat();
        hardStop = bluetooth.parseFloat();
        goodPower = bluetooth.parseFloat();
        tankButton = bluetooth.parseFloat();
    }
  }
  if (tankButton == 1) state = TANK;
  else if (attack == 1 || hardStop == 1 || goodPower == 1) state = MELTY;
  if (state == TANK) curvatureDrive(yAxis, rotation);
  else meltyDrive(xAxis, yAxis, rotation);
  lastState = state;
}

void meltyDrive(float xAxis, float yAxis, float rotation) {
  if (throttleUp == 1 && lastThrottleUp == 0) {
    throttle += 5;
    wasAttacking = false;
  }
  if (throttleDown == 1 && lastThrottleDown == 0) {
    throttle -= 5;
    wasAttacking = false;
  }
  throttle = constrain(throttle, 753, 1023);
  lastThrottleUp = throttleUp;
  lastThrottleDown = throttleDown;

  if (hardStop == 1) {
    throttle = 753;
    wasAttacking = false;
  }
  else if (goodPower == 1) {
    throttle = 823;
    wasAttacking = false;
  }
  else if (attack == 1) {
    throttle = 1023;
    wasAttacking = true;
  }
  else if (attack == 0 && wasAttacking == true) {
    throttle = 823;
    wasAttacking = false;
  }

  // if (trim == 1 && lastTrim != 1) radiusFudge += 0.0005;
  // else if (trim == -1  && lastTrim != -1) radiusFudge -= 0.0005;
  // if (trimReset == 1 || trimReset == -1) radiusFudge = 0;
  // lastTrim = trim;

  float joystickAngle = atan2(xAxis, yAxis);
  float joystickMagnitude = constrain(sqrt(xAxis * xAxis + yAxis * yAxis), 0, 1);

  // if (attacking) {
  //   throttle -= 0.1;
  //   if (throttle < 850) {
  //     throttle = 850;
  //     attacking = false;
  //   }
  // }
  // if (attack == 1) {
  //   attacking = true;
  //   throttle = 900;
  // }

  // get acceleration
  uint8_t accelData[6];
  readI2CRegNBlocking(ADDR_ACCEL, OUT_X_L, 6, accelData);
  int16_t xAccel = (((int16_t) accelData[1]) << 8) | (int16_t) accelData[0];
  int16_t yAccel = (((int16_t) accelData[3]) << 8) | (int16_t) accelData[2];
  int16_t zAccel = (((int16_t) accelData[5]) << 8) | (int16_t) accelData[4];
  xAccel >>= 4;
  yAccel >>= 4;
  zAccel >>= 4;

  // log time
  lastTime = currentTime;
  currentTime = micros();
  long deltaTime = currentTime - lastTime;
  if (deltaTime > 500000) {
    // just switched to melty mode, bad frame
    deltaTime = 0;
    angle = PI * 0.5;
    throttle = 0;
  }

  // calculate change in angle
  double cenAcc = accelUnitsToMS2(xAccel);
  lastAngularVel = angularVel;
  angularVel = sqrt(fabs(cenAcc / (ACCELEROMETER_RADIUS + radiusFudge)));
  double deltaAngle = ((angularVel + lastAngularVel) * 0.5) * (deltaTime * 0.000001);
  angle += deltaAngle;
  if (fabs(rotation) > 0.1) {
    angle += (-rotation * 2 * PI) * (deltaTime * 0.000001);
  }

  float angleDifference = fabs(fabs(fmod(angle, 2 * PI) - joystickAngle) - PI);

  // Serial.println("xAccel:  " + String(xAccel) + "\ttrim:  " + String(radiusFudge, 5) + "\tthrottle:  " + String(throttle));

  throttle = constrain(throttle, 753, 1023);
  
  if (fmod(angle, 2.0 * PI) > PI) digitalWrite(ledPin, HIGH);
  else digitalWrite(ledPin, LOW);

  // set motor speed
  if (millis() - lastTimePacketReceived > 1000) {
    throttle = 753;
    ledcWrite(leftPwmChannel, throttle);
    ledcWrite(rightPwmChannel, throttle);
  }
  else {
    // draw arc
    if (fabs(xAxis) < 0.1 && fabs(yAxis) < 0.1) {
      ledcWrite(leftPwmChannel, throttle);
      ledcWrite(rightPwmChannel, throttle);
    }
    else if (angleDifference > PI * 0.5) {
      ledcWrite(leftPwmChannel, min(throttle + ((throttle - 753) * fmap(joystickMagnitude, 0, 1, 0, 0.9)), 1023));
      ledcWrite(rightPwmChannel, max(throttle - ((throttle - 753) * fmap(joystickMagnitude, 0, 1, 0, 0.9)), 753));
    }
    else {
      ledcWrite(leftPwmChannel, max(throttle - ((throttle - 753) * fmap(joystickMagnitude, 0, 1, 0, 0.9)), 753));
      ledcWrite(rightPwmChannel, min(throttle + ((throttle - 753) * fmap(joystickMagnitude, 0, 1, 0, 0.9)), 1023));
    }
  }
}

float quickStopThreshold = 0.2;
float quickStopAlpha = 0.1;
float quickStopAccumulator;
float invertedReverse = false;
float quickTurnThreshold = 0.2;

void curvatureDrive(float throttle, float rotation) {
  float angularPower;
  boolean overPower;

  //rotation *= 0.5;

  if (fabs(throttle) < 0.1) throttle = 0;
  if (fabs(rotation) < 0.1) rotation = 0;

  if (throttle < quickTurnThreshold) {
      if (fabs(throttle) < quickStopThreshold) {
          quickStopAccumulator = (1 - quickStopAlpha) * quickStopAccumulator + quickStopAlpha * rotation * 2;
      }
      overPower = true;
      angularPower = rotation;
  }
  else {
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
  }
  else {
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

  ledcWrite(leftPwmChannel, fmap(leftPower, -1, 1, 713, 793));
  ledcWrite(rightPwmChannel, fmap(rightPower, -1, 1, 793, 713));
}

void stop() {
  ledcWrite(leftPwmChannel, 753);
  ledcWrite(rightPwmChannel, 753);
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// write a byte to an I2C device
uint8_t writeI2CReg8Blocking(uint8_t addr, uint8_t subaddr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(subaddr);
  Wire.write(data);
  return Wire.endTransmission();
}

//read N bytes from an I2C device
void readI2CRegNBlocking(uint8_t addr, uint8_t subaddr, uint8_t buflen, uint8_t *buf) {
  Wire.beginTransmission(addr);
  Wire.write(subaddr | 0x80); //the current accelerometer requires that the msb be set high to do a multi-byte transfer
  Wire.endTransmission();
  Wire.requestFrom(addr, buflen);
  while(Wire.available()) *(buf++) = Wire.read();
}

// only for 400g max right now
float accelUnitsToGs(int nativeUnits) {
  return ((400.0 * (float)nativeUnits) / 2047);
}

float accelUnitsToMS2(int nativeUnits) {
  return ((400.0 * (float)nativeUnits) / 2047) * 9.80665;
}