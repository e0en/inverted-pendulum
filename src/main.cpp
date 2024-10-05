#include "MPU6050_6Axis_MotionApps20.h"
#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

// MPU-6050 configurations
const unsigned SDA_PIN = 8;
const unsigned SCL_PIN = 9;
const unsigned MPU6050_I2C = 0x68;

const unsigned AFS_SEL = 1; // +/- 2g, 16,384 steps/g
const float STEPS_PER_G = 16384.0 / (2 << AFS_SEL);

const unsigned FS_SEL = 0; // +/- 250 deg/s, 131 steps/(deg/s)
const float STEPS_PER_DEG_PER_SECOND = 131.0 / (2 << FS_SEL);

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

float angle_accel[3];
float angle_gyro[3];

// DRV8835 configurations
const int PWM_FREQUENCY = 50000;
const int PWM_RESOLUTION = 8;
const unsigned IN1EN_PIN = 3;
const unsigned IN1PH_PIN = 2;
const unsigned IN2EN_PIN = 1;
const unsigned IN2PH_PIN = 0;

int8_t left_speed = 0;
int8_t right_speed = 0;

void get_angle_from_accel(float aa_si[3], float *angle);
void drive_motor(uint8_t channel, uint8_t pin, int8_t speed);

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

  int gyro_range = mpu.getFullScaleGyroRange();
  gyro_unit = 0.0;
  if (gyro_range == 0) {
    gyro_unit = 250.0;
  } else if (gyro_range == 1) {
    gyro_unit = 500.0;
  } else if (gyro_range == 2) {
    gyro_unit = 1000.0;
  } else if (gyro_range == 3) {
    gyro_unit = 2000.0;
  }
  gyro_unit /= 16384.0;

  int accel_range = mpu.getFullScaleAccelRange();
  accel_unit = 0.0;
  if (accel_range == 0) {
    accel_unit = 2.0;
  } else if (accel_range == 1) {
    accel_unit = 4.0;
  } else if (accel_range == 2) {
    accel_unit = 8.0;
  } else if (accel_range == 3) {
    accel_unit = 16.0;
  }
  accel_unit /= 16384.0;
  accel_unit *= 9.8;
}

void loop() {
  if (!is_connected) {
    Serial.println("MPU-6050 not available");
  }

  if (mpu.dmpPacketAvailable()) {
    mpu.dmpGetCurrentFIFOPacket(packet);

    mpu.dmpGetGyro(&av, packet);
    mpu.dmpGetAccel(&aa, packet);

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

    Serial.printf("v = (%f, %f, %f)\n", av_si[0], av_si[1], av_si[2]);
    Serial.printf("a = (%f, %f, %f)\n", aa_si[0], aa_si[1], aa_si[2]);
    Serial.printf("angle(radian) = (%f, %f, %f)\n", angle_accel[0],
                  angle_accel[1], angle_accel[2]);
    Serial.printf("angle(radian) = (%f, %f, %f)\n", angle_gyro[0],
                  angle_gyro[1], angle_gyro[2]);

    left_speed = int8_t(aa.x >> 6);
    right_speed = -int8_t(aa.x >> 6);
    Serial.printf("speed = (%d, %d)\n", left_speed, right_speed);

    drive_motor(0, IN1PH_PIN, left_speed);
    drive_motor(1, IN2PH_PIN, right_speed);
  }
  delay(100);
}

void get_angle_from_accel(float aa_si[3], float *angle) {
  angle[0] = atanf(aa_si[0] / sqrtf(aa_si[1] * aa_si[1] + aa_si[2] * aa_si[2]));
  angle[1] = atanf(aa_si[1] / sqrtf(aa_si[0] * aa_si[0] + aa_si[2] * aa_si[2]));
  angle[2] = atanf(aa_si[2] / sqrtf(aa_si[1] * aa_si[1] + aa_si[0] * aa_si[0]));
}

void drive_motor(uint8_t channel, uint8_t pin, int8_t speed) {
  if (speed > 0) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
  ledcWrite(channel, abs(speed));
}
