#include <IMU.h>
#include <Wire.h>
#include <L298n.h>
#include <BasicLinearAlgebra.h>
#include "NotchFilter.h"

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define GRAVITY -9.8 // m/s^2

#define IMU_Y_HEIGHT .182 // meters
#define COM_Y_HEIGHT .243 // meters

#define pwma 2
#define fora 4
#define reva 3
#define pwmb 5
#define forb 6
#define revb 7

#define onoff_switch 21

L298n* motor_driver;
const uint8_t min_pwm = 0; //71;
const float max_duty_change_per_sec = 10000.0;
int16_t prev_duty = 0;
float p = 13.0;
float i = 0.0; //4.5;
float d = 2.0;
float setpoint = .089; // radians
float err_int = 0;
float decay_rate = .9;

const float xw_offset = -1.481;

IMU* imu;
// Kalman filter (terms follow wikipedia page on Kalman filters)
// state = [theta (radians from vertical around x axis), theta_dot (radians/sec)]
// observations z_acc = [y_acc - GRAVITY, z_acc] z_gyr = [x_gyr]
BLA::Matrix<2> x = {0, 0}; // state vector
BLA::Matrix<2, 2> P = { // state covariance
  pow(2.0, 2), 0,
  0, pow(1.0, 2)
};

BLA::Matrix<2, 2> getF(float dt_seconds) {
  BLA::Matrix<2, 2> F = {
    1, dt_seconds,
    0, 1  
  };
  return F;
}

BLA::Matrix<2, 2> Q = { // process covariance (per second)
  pow(0.1, 2), 0,
  0, pow(0.25, 2)
};

BLA::Matrix<2, 2> getHacc(BLA::Matrix<2> x_t) {
  float th_t = x_t(0);
  float th_dot_t = x_t(1);
  BLA::Matrix<2, 2> H = {
    -th_t/2*GRAVITY, 0, 
    -GRAVITY, 0
  };
  return H;
}
BLA::Matrix<1, 2> H_gyr = {0, 180 / PI};

float R_gyr = 0.16772075;
BLA::Matrix<2, 2> R_acc = {
  0.0099981, -0.00600238,
 -0.00600238, 0.03156589
};

long previous_micros;

BLA::Matrix<2, 2> I = {1, 0, 0, 1}; // identity matrix needed for Kalman filter update

void update_kalman_filter_gyr(BLA::Matrix<1> z) {
  BLA::Matrix<1> y = z - (H_gyr*x);
  BLA::Matrix<1, 1> S_inv = (H_gyr * P * (~H_gyr)) + R_gyr;
  bool is_nonsinglar = Invert(S_inv);
  BLA::Matrix<2, 1> K = P * (~H_gyr) * S_inv;
  
  x = x + K*y;
  P = (I - K*H_gyr) * P;
}

void update_kalman_filter_acc(BLA::Matrix<2> z) {
  BLA::Matrix<2, 2> H = getHacc(x);
  BLA::Matrix<2> y = z - (H*x);
  BLA::Matrix<2, 2> S_inv = (H * P * (~H)) + R_acc;
  bool is_nonsinglar = Invert(S_inv);
  BLA::Matrix<2, 2> K = P * (~H) * S_inv;
  
  x = x + K*y;
  P = (I - K*H) * P;
}

NotchFilter gyr_x_nf1 = NotchFilter(320.0*2.0*PI, 2.0*PI*5.0, 800.0);
NotchFilter gyr_x_nf2 = NotchFilter(320.0*2.0*PI, 2.0*PI*5.0, 800.0);
NotchFilter acc_y_nf = NotchFilter(244.9*2.0*PI, 2.0*PI*30.0, 1344.0);
NotchFilter acc_z_nf = NotchFilter(244.9*2.0*PI, 2.0*PI*30.0, 1344.0);

void setup() {
  pinMode(onoff_switch, INPUT_PULLDOWN);

  Serial.begin(115200);
  // while (!Serial);

  Wire.begin();
	Wire.setClock(400000);

  imu = new IMU();
  AccSetting acc_settings;
  acc_settings.scale = AccSetting::ACC_2g;
  acc_settings.data_rate = AccSetting::ACC_1344_5376HZ;
  acc_settings.power_mode = AccSetting::NORMAL;
  acc_settings.fifo_enabled = AccSetting::ENABLED;
  acc_settings.fifo_mode = AccSetting::BYPASS; // must enter bypass mode to reset the FIFO buffer
  imu->apply_acc_settings(acc_settings);

  acc_settings.fifo_mode = AccSetting::FIFO;
  imu->apply_acc_settings(acc_settings);

  GyrSetting gyr_settings;
  gyr_settings.scale = GyrSetting::GYR_250DPS;
  gyr_settings.data_rate = GyrSetting::GYR_800HZ;
  gyr_settings.fifo_enabled = GyrSetting::ENABLED;
  gyr_settings.fifo_mode = GyrSetting::BYPASS; // must enter bypass mode first to reset the FIFO buffer
  imu->apply_gyr_settings(gyr_settings);

  gyr_settings.fifo_mode = GyrSetting::FIFO;
  imu->apply_gyr_settings(gyr_settings);
  
  motor_driver = new L298n(pwma, fora, reva, pwmb, forb, revb);
  previous_micros = micros();

  Serial.print(p);
 Serial.print(" ");
  Serial.print(i);
  Serial.print(" ");
  Serial.print(d);
  Serial.print(" ");
  Serial.println(setpoint);
}

void loop() {
  // if (!Serial) {
  //   motor_driver->set(0, 0);
  //   delay(100);
  //   CPU_RESTART
  // }

  long t = micros();
  float dt_seconds = ((float) (t - previous_micros)) / 1000000.0;
  previous_micros = t;
  BLA::Matrix<2, 2> F = getF(dt_seconds);
  x = F * x;
  P = (F * P * (~F)) + (Q*dt_seconds);

  size_t n_gyr = imu->read_gyro();
  size_t n_acc = imu->read_accelerometer();
  if (n_gyr > 0) {
    xyz_reading_raw reading = imu->gyr_buffer.pop();
    float val = reading.x*imu->gyr_factor - xw_offset;
    val = gyr_x_nf1.next(val);
    val = gyr_x_nf2.next(val);
    BLA::Matrix<1> z_gyr = {val};
    update_kalman_filter_gyr(z_gyr);
  }
  if (n_acc > 0) {
    xyz_reading_raw reading = imu->acc_buffer.pop();
    float val_y = GRAVITY - reading.y*imu->acc_factor;
    val_y = acc_y_nf.next(val_y);
    float val_z = reading.z*imu->acc_factor;
    val_z = acc_z_nf.next(val_z);
    BLA::Matrix<2> z_acc = {val_y, val_z};
    update_kalman_filter_acc(z_acc);
  }

  // Serial.print(x(0), 4);
  // Serial.print(" ");
  // Serial.print(x(1));
  // Serial.println();

  float err = x(0) - setpoint;
  // err_int = (err_int*decay_rate) + (err*dt_seconds / 0.0013); // this scales the err_int to be similar to err
  float control = p*err + i*err_int*(1 - decay_rate) + d*x(1);

  control = constrain(control, -1.0, 1.0);
  int control_sign = (control > 0) - (control < 0);

  int16_t max_duty_change = (int16_t) max_duty_change_per_sec*dt_seconds;
  int16_t duty = constrain(
    min_pwm*control_sign + control*(255 - min_pwm), 
    prev_duty - max_duty_change,
    prev_duty + max_duty_change
  );
  prev_duty = duty;

  if (digitalRead(onoff_switch)) {
    motor_driver->set(duty, duty);
  } else {
    motor_driver->set(0, 0);
  }

  if (Serial.available() > 0) {
    p = Serial.parseFloat();
    i = Serial.parseFloat();
    d = Serial.parseFloat();
    setpoint = Serial.parseFloat();
    err_int = 0;
    Serial.clear();

    Serial.print(p);
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(d);
    Serial.print(" ");
    Serial.println(setpoint);
  }

  // long delay_time = 1300 - (micros() - t);
  // if (delay_time > 0) {
  //   delayMicroseconds(delay_time);
  // }
}
