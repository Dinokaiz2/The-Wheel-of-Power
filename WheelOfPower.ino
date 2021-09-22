#include <SparkFun_LIS331.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TimerOne.h>
#include <stdlib.h>

// Settings
constexpr uint16_t GOOD_POWER = 803;
constexpr uint16_t NEUTRAL_POWER = 753;
constexpr float TURN_SPEED = 1.3;

constexpr float ACCEL_RADIUS = 0.06108; // meters
constexpr float ACCEL_RADIUS_REVERSE = 0.06033;

bool ENABLE_TRIM = false;

// Angle between LED and actual front of the robot, in radians (CCW is +)
constexpr float MELTY_LED_OFFSET = -1.0210177; // -0.5497787 (based on CAD) - 27deg CW fudge

constexpr bool MOTOR_DEBUG = false;
constexpr bool LORA_DEBUG = true;
constexpr bool ACCEL_DEBUG = true;

// Pins
constexpr uint8_t SPI_MOSI = 11;
constexpr uint8_t SPI_MISO = 12;
constexpr uint8_t SPI_SCK = 13;

constexpr uint8_t LORA_CS = 4;
constexpr uint8_t LORA_RST = 3;
constexpr uint8_t LORA_DIO0 = 2;

constexpr uint8_t ACCEL_CS = 7;

constexpr uint8_t MOTOR_L = 9;
constexpr uint8_t MOTOR_R = 10;

constexpr uint8_t POWER_LED_PIN = A0;
constexpr uint8_t MELTY_LED_PIN = 8;

// Oneshot125: 4000 Hz, 50% (125 us) - 100% (250 us) duty
constexpr float MOTOR_FREQ = 4000.0;
constexpr float MOTOR_PERIOD =  (1 / MOTOR_FREQ * 1000000); // 250 us
constexpr uint8_t MOTOR_RES = 10; // bit resolution

constexpr int SPI_FREQUENCY = 1000000;

enum class Mode { NO_PWM, MELTY, TANK };

typedef struct {
    int16_t power;
    float angle;
    float angularVel;
    long lastMeltyFrameTime; // us
    float radiusTrim;
    bool reversed;
    Mode mode;
} RobotState;

typedef struct {
    byte hash;
    byte leftX;
    byte leftY;
    byte rightX;
    bool start : 1;
    bool back : 1;
    bool yButton : 1;
    bool xButton : 1;
    bool bButton : 1;
    bool aButton : 1;
    bool rightBumper : 1;
    bool leftBumper : 1;
    byte dollar;
} Packet;

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
    bool start;
    bool back;
} ControllerState;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelState;

RH_RF95 loRa(LORA_CS, LORA_DIO0);
LIS331 accel;

void setup() {
    if (LORA_DEBUG || ACCEL_DEBUG || MOTOR_DEBUG) Serial.begin(115200);
    initMotors();
    initAccelerometer();
    initLoRa();
    pinMode(MELTY_LED_PIN, OUTPUT);
    pinMode(POWER_LED_PIN, OUTPUT);
    digitalWrite(POWER_LED_PIN, HIGH);
}

void loop() {
    static ControllerState* controllerState = calloc(1, sizeof(ControllerState));
    static ControllerState* lastControllerState = calloc(1, sizeof(ControllerState));
    static AccelState* accelState = calloc(1, sizeof(AccelState));
    static RobotState* robotState = calloc(1, sizeof(RobotState));
    accelRecv(accelState);
    getControllerState(&controllerState, &lastControllerState);

    if (controllerState->bButton) robotState->mode = Mode::TANK;
    else if (controllerState->aButton || controllerState->xButton || controllerState->yButton) robotState->mode = Mode::MELTY;

    if (robotState->mode == Mode::TANK) curvatureDrive(controllerState);
    else if (robotState->mode == Mode::MELTY) meltyDrive(robotState, controllerState, lastControllerState, accelState);
    else if (robotState->mode == Mode::NO_PWM) digitalWrite(MELTY_LED_PIN, HIGH);
}

void initMotors() {
    Timer1.initialize(MOTOR_PERIOD);
    if (MOTOR_DEBUG) Serial.println("Motors initialized");
}

void initAccelerometer() {
    pinMode(ACCEL_CS, OUTPUT);
    digitalWrite(ACCEL_CS, HIGH);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
    SPI.begin();
    accel.setSPICSPin(ACCEL_CS);
    while (!accel.begin(LIS331::USE_SPI)) {
        if (ACCEL_DEBUG) Serial.println("Accelerometer init failed. Retrying...");
        delay(200);
    }
    accel.setODR(accel.DR_1000HZ);
    accel.setFullScale(accel.HIGH_RANGE);
    if (ACCEL_DEBUG) Serial.println("Accelerometer initialized");
}

void initLoRa() {
    if (LORA_DEBUG) Serial.print("LoRa initializing");
    while (!loRa.init()) {
        if (LORA_DEBUG) Serial.println("LoRa init failed. Retrying...");
        delay(200);
    }
    loRa.setFrequency(915);
    loRa.setSignalBandwidth(250000);
    loRa.setSpreadingFactor(7);
    loRa.setCodingRate4(5);
    if (LORA_DEBUG) Serial.println("LoRa initialized");
}

void accelRecv(AccelState* accelState) {
    // Prevents LoRa from taking over SPI from an interrupt while reading accelerometer, which makes the program hang
    SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    accel.readAxes(accelState->x, accelState->y, accelState->z);
    SPI.endTransaction();
    if (ACCEL_DEBUG) {
        Serial.print("X: ");
        Serial.print(accelState->x);
        Serial.print("\tY: ");
        Serial.print(accelState->y);
        Serial.print("\tZ: ");
        Serial.println(accelState->z);
    }
}

// TODO: call loRaRecv(), which should read a binary packet into a char buffer
// then pass that sequence to parseControllerState() which puts it in a ControllerState
void getControllerState(ControllerState** controllerState, ControllerState** lastControllerState) {
    // TODO: probably worth making sure that this isn't copying more than i expect
    // TODO: i could probably make this faster if i don't worry about memory use so much
    //       idea: memcopy controllerState to lastControllerState, then read into controllerState
    //              - no more memcmp
    //              - still have to copy memory, but have to anyway
    Packet* packet = (Packet*)calloc(1, sizeof(Packet)); // TODO: wish this wasn't dynamically allocd
    uint8_t len = sizeof(Packet);                         // TODO: this isnt what the docs say to do, but the examples have it
    if (loRa.recv((uint8_t*)packet, &len)) { // TODO: make sure we received the whole packet (check len?)
        parseControllerState(packet, *lastControllerState);
        ControllerState* temp = *controllerState;
        *controllerState = *lastControllerState;
        *lastControllerState = temp;
    } else if (memcmp(*controllerState, *lastControllerState, sizeof(ControllerState)) != 0)
        memcpy(*lastControllerState, *controllerState, sizeof(ControllerState));
    free(packet);
}

/**
 * Parses a packet from LoRa and reads it into the specified ControllerState.
 * Precondition: packet waiting in LoRa buffer
 */
void parseControllerState(Packet* packet, ControllerState* controllerState) {
    // TODO: deadzone
    controllerState->timestamp = millis();
    controllerState->leftX = byteToAxis(packet->leftX);
    controllerState->leftY = byteToAxis(packet->leftY);
    controllerState->rightX = byteToAxis(packet->rightX);
    controllerState->leftBumper = packet->leftBumper;
    controllerState->rightBumper = packet->rightBumper;
    controllerState->aButton = packet->aButton;
    controllerState->bButton = packet->bButton;
    controllerState->xButton = packet->xButton;
    controllerState->yButton = packet->yButton;
    controllerState->start = packet->start;
    controllerState->back = packet->back;
}

// TODO: Make this a parseControllerState lambda
float byteToAxis(byte axis) {
    return axis == 127 ? 0 : constrain(((float)axis / 127.5) - 1, -1, 1);
}

// TODO: Optimize
// Benchmark, make changes from float to integer math
void meltyDrive(RobotState* robotState, ControllerState* controllerState, ControllerState* lastControllerState, AccelState* accelState) {
    if (controllerState->rightBumper && !lastControllerState->rightBumper) robotState->power += 5;
    if (controllerState->leftBumper && !lastControllerState->leftBumper) robotState->power -= 5;
    robotState->power = constrain(robotState->power, NEUTRAL_POWER, 1023);

    if (controllerState->aButton) robotState->power = NEUTRAL_POWER;
    else if (controllerState->xButton) robotState->power = GOOD_POWER;
    else if (controllerState->yButton) robotState->power = 1023; // max power
    else if (!controllerState->yButton && lastControllerState->yButton) robotState->power = GOOD_POWER;

    if (ENABLE_TRIM) {
        if (controllerState->start && !lastControllerState->start) robotState->radiusTrim += 0.00025;
        if (controllerState->back && !lastControllerState->back) robotState->radiusTrim -= 0.00025;
        if (controllerState->start && controllerState->back) robotState->radiusTrim = 0;
    } else {
        if (controllerState->start || controllerState->back) robotState->power = NEUTRAL_POWER;
        if (controllerState->start) robotState->reversed = false;
        if (controllerState->back) robotState->reversed = true;
    }

    // TODO: move to controller parsing, we only need to do this on new packet
    // TODO: omnidirectional movement probably won't work with direction reversed
    float joystickAngle = atan2(controllerState->leftX, -controllerState->leftY);
    float joystickMagnitude = constrain(sqrt(sq(controllerState->leftX) + sq(controllerState->leftY)), 0, 1);

    // log time
    long deltaTime = micros() - robotState->lastMeltyFrameTime;
    robotState->lastMeltyFrameTime = micros();
    if (deltaTime > 500000) {
        // just switched to melty mode, bad frame
        deltaTime = 0;
        robotState->angle = PI * 0.5;
        robotState->power = NEUTRAL_POWER;
        robotState->reversed = false;
    }

    // calculate change in angle
    float cenAcc = accelUnitsToMS2(sqrt(fabs(sq((long)accelState->x) + sq((long)accelState->y)))); // TODO: do these have to be longs? probably dont need fabs
    // TODO: reversing should reverse angularVel, which might make turning right
    float angularVel = sqrt(fabs(cenAcc / ((robotState->reversed ? ACCEL_RADIUS_REVERSE : ACCEL_RADIUS) + robotState->radiusTrim)));
    float deltaAngle = ((angularVel + robotState->angularVel) * 0.5) * (deltaTime * 0.000001);
    robotState->angle += deltaAngle;
    robotState->angularVel = angularVel;

    if (fabs(controllerState->rightX) > 0.1) {
        if (robotState->reversed) robotState->angle += controllerState->rightX * 2 * PI * TURN_SPEED * deltaTime * 0.000001;
        else robotState->angle += -controllerState->rightX * 2 * PI * TURN_SPEED * deltaTime * 0.000001;
    }

    float angleDifference = fabs(fabs(fmod(robotState->angle, 2 * PI) - joystickAngle) - PI);

    robotState->power = constrain(robotState->power, NEUTRAL_POWER, 1023);
    // draw arc
    if (fmod(robotState->angle, 2.0 * PI) > (1.25 * PI) + MELTY_LED_OFFSET && fmod(robotState->angle, 2.0 * PI) < (1.75 * PI) + MELTY_LED_OFFSET)
        digitalWrite(MELTY_LED_PIN, HIGH);
    else digitalWrite(MELTY_LED_PIN, LOW);

    // set motor speed
    if (millis() - controllerState->timestamp > 1000) {
        robotState->power = NEUTRAL_POWER;
        setMotorsMelty(robotState->power, robotState->power, robotState->reversed);
    } else {
        if (fabs(controllerState->leftX) < 0.1 && fabs(controllerState->leftY) < 0.1) {
            setMotorsMelty(robotState->power, robotState->power, robotState->reversed);
        } else {
            int deflection = ((robotState->power - NEUTRAL_POWER) * fmap(joystickMagnitude, 0, 1, 0, 0.5));
            if (angleDifference > PI * 0.5) setMotorsMelty(robotState->power + (deflection * 3), robotState->power - deflection, robotState->reversed); // 50-250
            else setMotorsMelty(robotState->power - deflection, robotState->power + (deflection * 3), robotState->reversed);
        }
    }
}

// NEUTRAL_POWER...1023
void setMotorsMelty(int leftPower, int rightPower, bool reversed) {
    if (reversed) {
        leftPower = NEUTRAL_POWER - (leftPower - NEUTRAL_POWER);
        rightPower = NEUTRAL_POWER - (rightPower - NEUTRAL_POWER);
        Timer1.pwm(MOTOR_L, constrain(leftPower, 520, NEUTRAL_POWER));
        Timer1.pwm(MOTOR_R, constrain(rightPower, 520, NEUTRAL_POWER));
    } else {
        Timer1.pwm(MOTOR_L, constrain(leftPower, NEUTRAL_POWER, 1023));
        Timer1.pwm(MOTOR_R, constrain(rightPower, NEUTRAL_POWER, 1023));
    }
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
}

float accelUnitsToMS2(float nativeUnits) {
    return ((400.0 * (float)nativeUnits) / 2047) * 9.80665;
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}