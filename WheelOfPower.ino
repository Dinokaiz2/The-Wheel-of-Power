#include <SparkFun_LIS331.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TimerOne.h>
#include <stdlib.h>

// Settings
constexpr float GOOD_POWER = 0.18;
constexpr float TURN_SPEED = 1.3;

constexpr float ACCEL_RADIUS = 0.061755; // meters
constexpr float ACCEL_RADIUS_REVERSE = 0.06333;

constexpr float MOTOR_LAG_ANGLE = 0.6; // radians

bool ENABLE_TRIM = false;

// Angle between LED and weapon (forward), in radians (CCW is +)
constexpr float MELTY_LED_OFFSET = 2.67035; // 153 degrees CCW

constexpr bool MOTOR_DEBUG = false;
constexpr bool LORA_DEBUG = false;
constexpr bool ACCEL_DEBUG = false;

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
constexpr float MOTOR_PERIOD = (1 / MOTOR_FREQ * 1000000); // 250 us
constexpr uint8_t MOTOR_RES = 10; // bit resolution

constexpr uint16_t MIN_DUTY = 520;
constexpr uint16_t NEUTRAL_DUTY = 753;
constexpr uint16_t MAX_DUTY = 1023;

constexpr int SPI_FREQUENCY = 1000000;

enum class Mode { NO_PWM, MELTY, TANK };

struct Robot {
    float power;
    float angle;
    float prev_ang_vel;
    long last_melty_frame_time; // us
    float radius_trim;
    bool reversed;
    Mode mode;
};

struct Packet {
    byte pound;

    byte left_x;
    byte left_y;
    byte right_x;

    bool a : 1;
    bool b : 1;
    bool x : 1;
    bool y : 1;
    bool left_bumper : 1;
    bool right_bumper : 1;
    bool left_trigger : 1;
    bool right_trigger : 1;

    bool dpad_left: 1;
    bool dpad_right: 1;
    bool dpad_up : 1;
    bool dpad_down: 1;
    bool start : 1;
    bool back : 1;
    bool xbox : 1;
    bool empty : 1;

    byte dollar;
};

struct Gamepad {
    unsigned long timestamp;
    float left_x;
    float left_y;
    float right_x;
    bool a;
    bool b;
    bool x;
    bool y;
    bool left_bumper;
    bool right_bumper;
    bool left_trigger;
    bool right_trigger;
    bool dpad_left;
    bool dpad_right;
    bool dpad_up;
    bool dpad_down;
    bool start;
    bool back;
    bool xbox;
};

struct Acceleration {
    float x;
    float y;
    float z;
};

RH_RF95 lora(LORA_CS, LORA_DIO0);
LIS331 accelerometer;

Gamepad gamepad;
Gamepad prev_gamepad;
Acceleration acceleration;
Robot robot;

void setup() {
    if (LORA_DEBUG || ACCEL_DEBUG || MOTOR_DEBUG) Serial.begin(115200);
    init_motors();
    init_accelerometer();
    init_lora();
    pinMode(MELTY_LED_PIN, OUTPUT);
    pinMode(POWER_LED_PIN, OUTPUT);
    digitalWrite(POWER_LED_PIN, HIGH);
}

void loop() {
    read_accel(&acceleration);
    read_gamepad(&gamepad, &prev_gamepad);

    if (gamepad.b) robot.mode = Mode::TANK;
    else if (gamepad.a || gamepad.x || gamepad.y || gamepad.start) robot.mode = Mode::MELTY;

    if (robot.mode == Mode::TANK) curvature_drive(gamepad);
    else if (robot.mode == Mode::MELTY) melty_drive(&robot, gamepad, prev_gamepad, acceleration);
    else if (robot.mode == Mode::NO_PWM) digitalWrite(MELTY_LED_PIN, HIGH);
}

void init_motors() {
    Timer1.initialize(MOTOR_PERIOD);
    if (MOTOR_DEBUG) Serial.println("Motors initialized");
}

void init_accelerometer() {
    pinMode(ACCEL_CS, OUTPUT);
    digitalWrite(ACCEL_CS, HIGH);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
    SPI.begin();
    accelerometer.setSPICSPin(ACCEL_CS);
    while (!accelerometer.begin(LIS331::USE_SPI)) {
        if (ACCEL_DEBUG) Serial.println("Accelerometer init failed. Retrying...");
        delay(200);
    }
    accelerometer.setODR(accelerometer.DR_1000HZ);
    accelerometer.setFullScale(accelerometer.HIGH_RANGE);
    if (ACCEL_DEBUG) Serial.println("Accelerometer initialized");
}

void init_lora() {
    if (LORA_DEBUG) Serial.print("LoRa initializing");
    while (!lora.init()) {
        if (LORA_DEBUG) Serial.println("LoRa init failed. Retrying...");
        delay(200);
    }
    lora.setFrequency(915);
    lora.setSignalBandwidth(250000);
    lora.setSpreadingFactor(7);
    lora.setCodingRate4(5);
    if (LORA_DEBUG) Serial.println("LoRa initialized");
}

void read_accel(Acceleration* acceleration) {
    int16_t x, y, z;

    // Prevents LoRa from taking over SPI from an interrupt while reading accelerometer, which makes the program hang
    SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    accelerometer.readAxes(x, y, z);
    SPI.endTransaction();

    auto accel_units_to_mps2 = [](float native_units) {
        return ((400.0 * (float)native_units) / 2047) * 9.80665;
    };

    acceleration->x = accel_units_to_mps2(x);
    acceleration->y = accel_units_to_mps2(y);
    acceleration->z = accel_units_to_mps2(z);

    if (ACCEL_DEBUG) {
        Serial.print("X: ");
        Serial.print(acceleration->x);
        Serial.print("\tY: ");
        Serial.print(acceleration->y);
        Serial.print("\tZ: ");
        Serial.println(acceleration->z);
    }
}

void read_gamepad(Gamepad* gamepad, Gamepad* prev_gamepad) {
    memcpy(prev_gamepad, gamepad, sizeof(Gamepad));
    Packet packet = { 0 };
    uint8_t len = sizeof(Packet); // TODO: This isn't what the docs say to do, but the examples have it
    if (lora.recv((uint8_t*)&packet, &len)) parse_gamepad(packet, gamepad);
}

void parse_gamepad(Packet packet, Gamepad* gamepad) {
    if (packet.pound != '#' || packet.dollar != '$') return;
    auto byte_to_axis = [](byte axis) { return axis == 127 ? 0 : constrain(((float)axis / 127.5) - 1, -1, 1); };
    // TODO: deadzone
    gamepad->timestamp = millis();
    gamepad->left_x = 0; // Only forward and backward translation
    gamepad->left_y = -byte_to_axis(packet.left_y); // Make +y forward
    gamepad->right_x = byte_to_axis(packet.right_x);
    gamepad->a = packet.a;
    gamepad->b = packet.b;
    gamepad->x = packet.x;
    gamepad->y = packet.y;
    gamepad->left_bumper = packet.left_bumper;
    gamepad->right_bumper = packet.right_bumper;
    gamepad->left_trigger = packet.left_trigger;
    gamepad->right_trigger = packet.right_trigger;
    gamepad->dpad_left = packet.dpad_left;
    gamepad->dpad_right = packet.dpad_right;
    gamepad->dpad_up  = packet.dpad_up ;
    gamepad->dpad_down = packet.dpad_down;
    gamepad->start = packet.start;
    gamepad->back = packet.back;
    gamepad->xbox = packet.xbox;
}

// TODO: Optimize
// Benchmark, make changes from float to integer math
void melty_drive(Robot* robot, Gamepad gamepad, Gamepad prev_gamepad, Acceleration acceleration) {
    if (gamepad.right_bumper && !prev_gamepad.right_bumper) robot->power += 0.02;
    if (gamepad.left_bumper && !prev_gamepad.left_bumper) robot->power -= 0.02;
    robot->power = constrain(robot->power, 0, 1);

    if (gamepad.a) robot->power = 0;
    else if (gamepad.x) robot->power = GOOD_POWER;

    if (gamepad.y || gamepad.start) robot->power = 1;
    if (gamepad.y) robot->reversed = false;
    if (gamepad.start) robot->reversed = true;

    if (!gamepad.y && prev_gamepad.y) robot->power = GOOD_POWER;
    if (!gamepad.start && prev_gamepad.start) robot->power = GOOD_POWER;


    if (gamepad.dpad_right && !prev_gamepad.dpad_right) robot->radius_trim += 0.00025;
    if (gamepad.dpad_left && !prev_gamepad.dpad_left) robot->radius_trim -= 0.00025;
    if (gamepad.dpad_up) robot->radius_trim = 0;


    auto magnitude = [](float x1, float x2) { return sqrt(sq(x1) + sq(x2)); };
    auto math_mod = [](float x, float n) { return x - floor(x / n) * n; };

    // TODO: Move to controller parsing, we only need to do this on new packet
    float stick_angle = atan2(gamepad.left_y, gamepad.left_x);
    float stick_magnitude = constrain(magnitude(gamepad.left_x, gamepad.left_y), 0, 1);

    // TODO: This is the reverse of what expect. Either I misunderstand or this isn't because of motor lag.
    // Compensate for how long it takes for the motors to accelerate/decelerate
    stick_angle -= MOTOR_LAG_ANGLE * (robot->reversed ? -1 : 1);

    // Get time step
    long time_step = micros() - robot->last_melty_frame_time;
    robot->last_melty_frame_time = micros();
    if (time_step > 500000) {
        // Just switched to melty mode, bad frame
        time_step = 0;
        robot->angle = 0;
        robot->power = 0;
    }

    // Calculate change in angle
    float cen_accel = magnitude(acceleration.x, acceleration.y);

    float angular_vel;
    if (robot->reversed) angular_vel = sqrt(fabs(cen_accel / (ACCEL_RADIUS_REVERSE + robot->radius_trim))); // CCW
    else angular_vel = -sqrt(fabs(cen_accel / (ACCEL_RADIUS + robot->radius_trim))); // CW

    float delta_angle = ((angular_vel + robot->prev_ang_vel) * 0.5) * (time_step * 0.000001);
    robot->angle += delta_angle;
    robot->prev_ang_vel = angular_vel;

    // Turn heading
    if (fabs(gamepad.right_x) > 0.1) robot->angle += gamepad.right_x * 2 * PI * TURN_SPEED * time_step * 0.000001;

    // Angle from the robot's angle to the joystick's angle, from [-PI, PI)
    float angle_diff = math_mod((stick_angle - robot->angle) + PI, 2 * PI) - PI;

    // Draw arc
    float melty_led_angle = math_mod(robot->angle + MELTY_LED_OFFSET, 2.0 * PI);
    if (melty_led_angle > 0.25 * PI && melty_led_angle < 0.75 * PI) digitalWrite(MELTY_LED_PIN, HIGH);
    else digitalWrite(MELTY_LED_PIN, LOW);

    // Set motor speed
    if (millis() - gamepad.timestamp > 1000) {
        robot->power = 0;
        set_motors_melty(robot->power, robot->power, robot->reversed);
    } else {
        if (fabs(gamepad.left_x) < 0.1 && fabs(gamepad.left_y) < 0.1) {
            set_motors_melty(robot->power, robot->power, robot->reversed);
        } else {
            float deflection = robot->power * fmap(stick_magnitude, 0, 1, 0, 0.5);
            if (angle_diff < 0) set_motors_melty(robot->power + deflection * 3, robot->power - deflection, robot->reversed); // 50-250
            else set_motors_melty(robot->power - deflection, robot->power + deflection * 3, robot->reversed);
        }
    }
}

// -1...1
void set_motors_melty(float left_power, float right_power, bool reversed) {
    uint16_t left_power_raw = reversed ? fmap(left_power, 0, 1, NEUTRAL_DUTY, MIN_DUTY) : fmap(left_power, 0, 1, NEUTRAL_DUTY, MAX_DUTY);
    uint16_t right_power_raw = reversed ? fmap(right_power, 0, 1, NEUTRAL_DUTY, MIN_DUTY) : fmap(right_power, 0, 1, NEUTRAL_DUTY, MAX_DUTY);
    if (reversed) {
        Timer1.pwm(MOTOR_L, left_power_raw);
        Timer1.pwm(MOTOR_R, right_power_raw);
    } else {
        Timer1.pwm(MOTOR_L, left_power_raw);
        Timer1.pwm(MOTOR_R, right_power_raw);
    }
}

float quick_stop_threshold = 0.2;
float quick_stop_alpha = 0.1;
float quick_stop_accumulator;
float inverted_reverse = false;
float quick_turn_threshold = 0.2;

void curvature_drive(Gamepad gamepad) {
    float angular_power;
    boolean over_power;
    float throttle = gamepad.left_y;
    float rotation = gamepad.right_x;
    // rotation *= 0.5;
    if (fabs(throttle) < 0.1) throttle = 0;
    if (fabs(rotation) < 0.1) rotation = 0;
    if (throttle < quick_turn_threshold) {
        if (fabs(throttle) < quick_stop_threshold)
            quick_stop_accumulator = (1 - quick_stop_alpha) * quick_stop_accumulator + quick_stop_alpha * rotation * 2;
        over_power = true;
        angular_power = rotation;
    } else {
        over_power = false;
        angular_power = fabs(throttle) * rotation - quick_stop_accumulator;
    }
    if (quick_stop_accumulator > 1) quick_stop_accumulator--;
    else if (quick_stop_accumulator < -1) quick_stop_accumulator++;
    else quick_stop_accumulator = 0;
    float left_power;
    float right_power;
    if (throttle < 0 && inverted_reverse) {
        left_power = throttle - angular_power;
        right_power = throttle + angular_power;
    } else {
        left_power = throttle + angular_power;
        right_power = throttle - angular_power;
    }
    if (throttle < 0 && inverted_reverse) {
        left_power = -left_power;
        right_power = -right_power;
    }
    if (over_power) {
        if (left_power > 1) {
            right_power -= left_power - 1;
            left_power = 1;
        } else if (right_power > 1) {
            left_power -= right_power - 1;
            right_power = 1;
        } else if (left_power < -1) {
            right_power -= left_power + 1;
            left_power = -1;
        } else if (right_power < -1) {
            left_power -= right_power + 1;
            right_power = -1;
        }
    }
    float max_magnitude = max(fabs(left_power), fabs(right_power));
    if (max_magnitude > 1) {
        left_power /= max_magnitude;
        right_power /= max_magnitude;
    }
    if (fabs(left_power) < 0.05) left_power = 0.05;
    if (fabs(right_power) < 0.05) right_power = 0.05;
    Timer1.pwm(MOTOR_L, (int)fmap(left_power, -1, 1, 713, 793));
    Timer1.pwm(MOTOR_R, (int)fmap(right_power, -1, 1, 793, 713));
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}