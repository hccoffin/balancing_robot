#include <IMU.h>
#include <L298n.h>
#include <BasicLinearAlgebra.h>

#define GRAVITY -9.8 // m/s^2
#define IMU_Y_HEIGHT .182 // meters
#define COM_Y_HEIGHT .243 // meters

#define pwma 2
#define fora 3
#define reva 4
#define pwmb 5
#define forb 6
#define revb 7

L298n* motor_driver;
const uint8_t min_pwm = 0; //71;
const uint8_t max_duty_change = 8;
int16_t prev_duty = 0;
float p = 9.4;
float i = 4.5;
float d = 2.65;
float setpoint = .13; //.155; // radians
float err_int = 0;
float decay_rate = .9;

int temp = 0;
float temp_sum = 0;

const float xw_offset = -1.48;
const float yw_offset = 0.60;
const float zw_offset = 0.63;

IMU* imu;
// Kalman filter (terms follow wikipedia page on Kalman filters)
// state = [theta (radians from vertical around x axis), theta_dot (radians/sec)]
// observation = [z_acc (m/s^2), y_acc - GRAVITY (m/s^2), x_gyr (deg/sec)]
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
BLA::Matrix<3, 2> getH(BLA::Matrix<2> x_t) { // measurement model
  float th_t = x_t(0);
  float th_t2 = th_t * th_t;
  float th_t3 = th_t2 * th_t;
//  float sin_th_t = sin(th_t);
//  float cos_th_t = cos(th_t);
//  float divisor = th_t;
//  if (abs(divisor) < 1e-9) {
//    float sign = (th_t >= 0) - (th_t < 0);
//    divisor = 1e-9 * sign;
//  }
  float th_dot_t = x_t(1);

//  Serial.println((1 - th_t2/6)*GRAVITY);
//  Serial.println((-th_t + th_t3/24)*GRAVITY);
  BLA::Matrix<3, 2> H = {
    -(1 - th_t2/6)*GRAVITY, 0, // assume Euler force (from angular acceleration) is negligible / taken care of in noise calcs
    (-th_t + th_t3/24)*GRAVITY, th_dot_t*IMU_Y_HEIGHT, // centrifugal force is felt by acc since it is non inertial (its probably not actually worth calculating)
    0, 180 / PI
  };
  return H;
}
BLA::Matrix<3, 3> R = { // measurement covariance
  pow(.1, 2), 0, .1*.2*pow(.8, 2),
  0, pow(.1, 2), 0,
  .1*.2*pow(.8, 2), 0, pow(.2, 2)
};
long previous_micros;

BLA::Matrix<2, 2> I = {1, 0, 0, 1}; // identity matrix needed for Kalman filter update

void update_kalman_filter (BLA::Matrix<3> z, float dt_seconds) {

//  Serial.print("z "); Serial << z; Serial.println();
  BLA::Matrix<2, 2> F = getF(dt_seconds);
//  Serial.print("F "); Serial << F; Serial.println();
  x = F * x;
  P = (F * P * (~F)) + (Q*dt_seconds);

  BLA::Matrix<3, 2> H = getH(x);
//  Serial.print("H "); Serial << H; Serial.println("");
  BLA::Matrix<3> y = z - (H*x);
//  Serial.print("y "); Serial << y; Serial.println("");
  BLA::Matrix<3, 3> S_inv = (H * P * (~H)) + R;
  bool is_nonsinglar = Invert(S_inv);
  BLA::Matrix<2, 3> K = P * (~H) * S_inv;
//  Serial.print("K "); Serial << K; Serial.println();
//  Serial.print("K*y "); Serial << K * y; Serial.println();
  
  x = x + K*y;
  P = (I - K*H) * P;
}


void setup() {
  Serial.begin(115200);
//  while (!Serial);
  imu = new IMU();
  motor_driver = new L298n(pwma, fora, reva, pwmb, forb, revb);
  previous_micros = micros();
}

void loop() {
  long t = micros();

  bool new_data_gyro = imu->read_gyro();
  if (!new_data_gyro) {
//    Serial.println("Waiting Gyro");
    long delay_time = 1300 - (micros() - t);
    delayMicroseconds(delay_time);
    return;
  }
  bool new_data_acc = imu->read_accelerometer();
  if (!new_data_acc) {
//    Serial.println("Waiting Acc");
    return;
  }

//  temp = (temp + 1) % 1000;
//  temp_sum = temp_sum + imu->xw;
  
  float dt_seconds = ((float) (t - previous_micros)) / 1000000.0;
  previous_micros = t;
  BLA::Matrix<3> z = {imu->za, imu->ya - GRAVITY, imu->xw - xw_offset};
  update_kalman_filter(z, dt_seconds);

//  Serial.print(z(0)); Serial.print(" ");
//  Serial.print(z(1)); Serial.print(" ");
//  Serial.print(z(2) * PI / 180); Serial.print(" ");
//  Serial.print(x(0), 4); Serial.print(" ");
//  Serial.print(x(1), 4); Serial.print(" ");

  float err = x(0) - setpoint;
  err_int = (err_int*decay_rate) + (err*dt_seconds / 0.0013); // this scales the err_int to be similar to err
  float control = p*err + i*err_int*(1-decay_rate) + d*x(1);

  control = constrain(control, -1.0, 1.0);
  int control_sign = (control > 0) - (control < 0);

  int16_t duty = constrain(
    min_pwm*control_sign + control*(255 - min_pwm), 
    prev_duty - max_duty_change,
    prev_duty + max_duty_change
  );
//  Serial.println(min_pwm*control_sign + control*(255 - min_pwm));
//  Serial.println(prev_duty - max_duty_change);
//  Serial.println(prev_duty + max_duty_change);
//  Serial.println(duty);
//  Serial.println(prev_duty);
  prev_duty = duty;
  motor_driver->set(duty, duty);

  Serial.print(err);
  Serial.print(" ");
  Serial.print(x(1));
  Serial.print(" ");
  Serial.print(p);
  Serial.print(" ");
  Serial.print(i);
  Serial.print(" ");
  Serial.print(d);
  Serial.print(" ");
  Serial.print(setpoint, 3);
  Serial.print(" ");
//  Serial.print(err_int*(1-decay_rate));
//  Serial.print(" ");
//  Serial.print(control);
//  Serial.print(" ");
//  Serial.print(duty);
  Serial.println();

  if (Serial.available() > 0) {
    p = Serial.parseFloat();
    i = Serial.parseFloat();
    d = Serial.parseFloat();
    setpoint = Serial.parseFloat();
    err_int = 0;
    Serial.clear();
  }
  
//    Serial.print(dt_seconds * 1000000);
//    Serial.print("      ");

//  if (temp == 0) {
////    Serial.print(dt_seconds * 1000000);
////    Serial.print("      ");
////    Serial.print(z(0), 3);
////    Serial.print(" ");
////    Serial.print(z(1), 3);
////    Serial.print(" ");
////    Serial.print(z(2), 3);
//    
////    Serial.print(imu->xw - xw_offset, 2);
////    Serial.print(" ");
////    Serial.print(imu->yw - yw_offset, 2);
////    Serial.print(" ");
////    Serial.print(imu->zw - zw_offset, 2);
////    
////    Serial.print("      ");
////    Serial.print(imu->xa, 2);
////    Serial.print(" ");
////    Serial.print(imu->ya, 2);
////    Serial.print(" ");
////    Serial.print(imu->za, 2);
//
//    Serial.print("      ");
//    Serial.println(temp_sum / 1000.0, 5);
//    temp_sum = 0;
////    Serial.print(" ");
////    Serial.print(x(1) * 180 / PI);
//    
//    Serial.println(" ");
//  }

  // reading registers from gyro too often seems to slow the board down
  // this caps the read rate to slightly less than 800 Hz, which is the max update rate for the gyro
  long delay_time = 1300 - (micros() - t);
  if (delay_time > 0) {
    delayMicroseconds(delay_time);
  }
}
