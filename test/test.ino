#include <L298n.h>
#include <IMU.h>

#define pwma 2
#define fora 3
#define reva 4
#define pwmb 5
#define forb 6
#define revb 7

IMU* imu;
L298n* motor_driver;

const float period = 2.0;
const int half_period_micros = 1000000 * period;

int duty;
int freq;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  motor_driver = new L298n(pwma, fora, reva, pwmb, forb, revb);
  imu = new IMU();
  Serial.print("here");
  duty = 0;
}

void loop() {
  if (Serial.available() > 0) {
    duty = Serial.parseInt();
  }

  bool new_data_gyro = imu->read_gyro();
  bool new_data_acc = imu->read_accelerometer();

  Serial.print(imu->za + 9);
  Serial.print(" ");
//  Serial.print(micros() % (2*half_period_micros) < half_period_micros);
//  Serial.print(" ");

  motor_driver->set(duty, duty);
  Serial.print((float) duty / 255.0);

//  if ((micros() % (2*half_period_micros)) < half_period_micros) {
//    motor_driver->set(duty, duty);
//    Serial.print((float) duty / 255.0);
//  } else {
//    motor_driver->set(-duty, -duty);
//    Serial.print(-(float)duty / 255.0);
//  }

  Serial.println("");

  delayMicroseconds(1300);
}
