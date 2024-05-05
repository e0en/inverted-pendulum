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

uint8_t buffer[64];

Quaternion q;
VectorInt16 av;
VectorInt16 aa;
int16_t gravity[3];
float ypr[3];

// DRV8835 configurations
const int PWM_FREQUENCY = 50000;
const int PWM_RESOLUTION = 8;
const unsigned IN1EN_PIN = 3;
const unsigned IN1PH_PIN = 2;
const unsigned IN2EN_PIN = 1;
const unsigned IN2PH_PIN = 0;

int8_t left_speed = 0;
int8_t right_speed = 0;

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
}

void loop() {
  if (!is_connected) {
    Serial.println("MPU-6050 not available");
  }

  int gyro_range = mpu.getFullScaleGyroRange();
  int accel_range = mpu.getFullScaleAccelRange();

  int16_t vx, vy, vz;
  int16_t ax, ay, az;
  mpu.getRotation(&vx, &vy, &vz);
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.printf("av\t%d\t%d\t\%d\n", vx, vy, vz);
  Serial.printf("aa\t%d\t%d\t\%d\n", ax, ay, az);

  left_speed = int8_t(ax >> 8);
  right_speed = -int8_t(ax >> 8);

  drive_motor(0, IN1PH_PIN, left_speed);
  drive_motor(1, IN2PH_PIN, right_speed);
  delay(1000);
}

void drive_motor(uint8_t channel, uint8_t pin, int8_t speed) {
  if (speed > 0) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
  ledcWrite(channel, abs(speed));
}
