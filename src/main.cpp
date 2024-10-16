#include "MPU6050_6Axis_MotionApps20.h"
#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

// MPU-6050 configurations
const unsigned SDA_PIN = 8;
const unsigned SCL_PIN = 9;
const unsigned MPU6050_I2C = 0x68;

const unsigned AFS_SEL = 0;
const unsigned FS_SEL = 3;

MPU6050 mpu(MPU6050_I2C);

bool is_connected = false;
bool is_dmp_ready = false;

uint8_t packet[42];
uint16_t packet_size;

Quaternion q;
VectorInt16 av;
VectorInt16 aa;

float gyro_unit;
float accel_unit;
float av_si[3];
float aa_si[3];
long last_ts;

float angle_accel[3];
float angle_gyro[3];

// DRV8835 configurations
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 12;
const int PWM_MAX = (1 << PWM_RESOLUTION);

const unsigned IN1EN_PIN = 3;
const unsigned IN1PH_PIN = 2;
const unsigned IN2EN_PIN = 1;
const unsigned IN2PH_PIN = 0;

void get_angle_from_accel(float aa_si[3], float *angle);
void drive_motor(uint8_t channel, uint8_t pin, int16_t speed);

void setup() {
  Serial.begin(9600);

  // initialize DRV8835 pins
  pinMode(IN1PH_PIN, OUTPUT);
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(IN1EN_PIN, 0);
  ledcWrite(0, 0);

  pinMode(IN2PH_PIN, OUTPUT);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(IN2EN_PIN, 1);
  ledcWrite(1, 0);

  digitalWrite(IN1PH_PIN, HIGH);
  digitalWrite(IN2PH_PIN, HIGH);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(4000000);

  mpu.reset();
  delay(100);
  mpu.initialize();
  is_connected = mpu.testConnection();

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();

  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  packet_size = mpu.dmpGetFIFOPacketSize();

  float int16_max = 1 << 15;

  int gyro_range = mpu.getFullScaleGyroRange();
  gyro_unit = 250.0 * (1 << gyro_range) / int16_max;

  int accel_range = mpu.getFullScaleAccelRange();
  accel_unit = 9.8 * (2 << accel_range) / int16_max;

  last_ts = micros();
}

void loop() {
  float c_p = 10.0;
  float c_i = 0.0;
  float c_d = 0.0;
  float error = 0.0;
  float speed = 0.0;
  long now_ts = last_ts;
  float dt = 0.0;

  float tau = 0.01;
  float angle[3] = {0, 0, 0};

  if (!is_connected) {
    Serial.println("MPU-6050 not available");
  }

  if (mpu.dmpPacketAvailable()) {

    mpu.dmpGetCurrentFIFOPacket(packet);

    mpu.dmpGetGyro(&av, packet);
    mpu.dmpGetAccel(&aa, packet);

    now_ts = micros();
    dt = (now_ts - last_ts) / 1000000.0;

    // degree per sec
    av_si[0] = float(av.x) * gyro_unit;
    av_si[1] = float(av.y) * gyro_unit;
    av_si[2] = float(av.z) * gyro_unit;
    get_angle_from_accel(aa_si, angle_accel);

    // meter per sec^2
    aa_si[0] = float(aa.x) * accel_unit;
    aa_si[1] = float(aa.y) * accel_unit;
    aa_si[2] = float(aa.z) * accel_unit;
    get_angle_from_accel(aa_si, angle_accel);

    float accelRoll = atan2(aa_si[0], aa_si[2]) * (180 / M_PI);
    float accelPitch = atan2(aa_si[1], aa_si[2]) * (180 / M_PI);

    angle[0] = (tau) * (angle[0] + av_si[0] * dt) + (1.0 - tau) * accelPitch;
    angle[1] = (tau) * (angle[1] + av_si[1] * dt) + (1.0 - tau) * accelRoll;
    angle[2] = angle[2] + av_si[1] * dt;

    error = angle[1] / 180.0 * PWM_MAX;
    speed = c_p * error;
    speed = speed > PWM_MAX ? (float)PWM_MAX : speed;
    speed = speed < -PWM_MAX ? (float)(-PWM_MAX) : speed;
    Serial.printf("speed = %d\n", (int16_t)speed);

    drive_motor(0, IN1PH_PIN, speed);
    drive_motor(1, IN2PH_PIN, -speed);
  }
}

void get_angle_from_accel(float aa_si[3], float *angle) {
  angle[0] = atanf(aa_si[0] / sqrtf(aa_si[1] * aa_si[1] + aa_si[2] * aa_si[2]));
  angle[1] = atanf(aa_si[1] / sqrtf(aa_si[0] * aa_si[0] + aa_si[2] * aa_si[2]));
  angle[2] = atanf(aa_si[2] / sqrtf(aa_si[1] * aa_si[1] + aa_si[0] * aa_si[0]));
}

void drive_motor(uint8_t channel, uint8_t pin, int16_t speed) {
  if (speed > 0) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
  ledcWrite(channel, abs(speed));
}
