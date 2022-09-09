#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "wheel.cpp"
#include "linalg.h"
#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68

WiFiMulti wifiMulti;
HTTPClient http;
TaskHandle_t readHttpTask;

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

//Wheel right(5.403, 0, 16, 17, 4, 33, 32, 0);
//Wheel left(5.371, 0, 5, 18, 19, 25, 26, 1);
Wheel right(179.0, 16, 17, 4, 33, 32, 0);
Wheel left(190.0, 5, 18, 19, 25, 26, 1);

float p, i, d, setpoint;
const size_t n_deltas = 50;
float deltas[n_deltas];
float delta_summed;
size_t delta_index;

float prev_control = 0;

// Kalman filter stuff
const float sigma_theta2 = 5*5;
const float sigma_thetadot2 = .2*.2;
const float process_theta2 = 1*1;
const float process_thetadot2 = 5*5;
vec x;
mat Fmat = (mat){1, 1, 0, 1};
mat Hmat = (mat){1, 0, 0, 1};
mat Pmat = (mat){sigma_theta2, 0, 0, sigma_thetadot2};
mat Qmat = (mat){process_theta2, 0, 0, process_thetadot2};
mat Rmat = (mat){sigma_theta2, 0, 0, sigma_thetadot2};
mat identity = (mat){1, 0, 0, 1};

void requestPIDSetpoint(float &p, float &i, float &d, float &setpoint) {
//  long ts = millis();
	if((wifiMulti.run() == WL_CONNECTED)) {
		http.begin("192.168.1.68", 8000, "/pidsetpoint");
		int httpCode = http.GET();
//    Serial.println("");
//    Serial.println("");
//    Serial.println("");
//    Serial.print("time: ");
//    Serial.print(millis() - ts);
//    Serial.print(" Code: ");
//    Serial.print(httpCode);
//    Serial.print(" Size: ");
//    Serial.println(http.getSize());
		
		if(httpCode == HTTP_CODE_OK) {
			WiFiClient data = http.getStream();
//      Serial.print("Bytes Available: ");
//      Serial.println(data.available());
			uint8_t bytes[16];
			while(data.available() != 16) {
//        Serial.print(".");
				delay(10);
			}
//      Serial.println();
			data.read(bytes, 16);
//      Serial.print("Read: ");
//      for (int i = 0; i < 16; i++) {
//        Serial.print(bytes[i]);
//        Serial.print(" ");
//      }
//      Serial.println("");
			memcpy(&p, &bytes, sizeof(float));
			memcpy(&i, &bytes[4], sizeof(float));
			memcpy(&d, &bytes[8], sizeof(float));
			memcpy(&setpoint, &bytes[12], sizeof(float));
		}
		http.end();
	}
}

void readHttpTaskLoop(void * parameter) {
	for (;;) {
		requestPIDSetpoint(p, i, d, setpoint);
	}
}

void updateState () {
	xyzFloat gValue = myMPU6500.getGValues();
	xyzFloat gyr = myMPU6500.getGyrValues();

	vec z = (vec){atan2(-gValue.x, gValue.z), gyr.z};
	x = prod(Fmat, x);
	Pmat = sum(prod(Fmat, prod(Pmat, trans(Fmat))), Qmat);
	mat Smat = sum(prod(Hmat, prod(Pmat, trans(Hmat))), Rmat);
	mat Kmat = prod(Pmat, prod(trans(Hmat), inv(Smat)));
	x = sum(x, prod(Kmat, sum(z, prod(Hmat, prod(-1, x)))));
	Pmat = prod(sum(identity, prod(-1, prod(Kmat, Hmat))), Pmat);

	Serial.print("Accel:");
	Serial.print(gValue.z);
	Serial.print(" Accel-x:");
	Serial.print(gValue.x);
	Serial.print(" Gyro:");
	Serial.print(gyr.z);
	Serial.print(" Angle:");
	Serial.print(atan2(-gValue.x, gValue.z));
	Serial.print(" theta:");
	Serial.print(x.x1);
	Serial.print(" theta_dot:");
	Serial.print(x.x2);
	Serial.println("");
}

void setup() {
	Serial.begin(115200);

	// setting up the mpu6500 gyro/accelerometer
	Wire.begin(27, 14);
	assert(myMPU6500.init());
	delay(1000);
	myMPU6500.autoOffsets();
	// gyro
	myMPU6500.enableGyrDLPF();
	myMPU6500.setGyrDLPF(MPU6500_DLPF_0);
	myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
	// accelerometer
	myMPU6500.enableAccDLPF(true);
	myMPU6500.setAccDLPF(MPU6500_DLPF_0);
	myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

	updateState();

//  http.setReuse(true);
	wifiMulti.addAP("myluco-fiber", "0123456789abcdef01deadbeef");
	while (!wifiMulti.run() == WL_CONNECTED) {
		Serial.println("Didn't Connect.");
	}
	
	// set default values for p, i, d, setpoint to 0 and set all values in deltas to 0
	p = 0;
	i = 0;
	d = 0;
	setpoint = 0;
	for (int i = 0; i < n_deltas; i++) {
		deltas[i] = 0;
	}
	delta_summed = 0;
	
	// get PID setpoint once and then start readHttp task
	requestPIDSetpoint(p, i, d, setpoint);
	xTaskCreatePinnedToCore(
			readHttpTaskLoop, /* Function to implement the task */
			"readHttpTask", /* Name of the task */
			10000,  /* Stack size in words */
			NULL,  /* Task input parameter */
			0,  /* Priority of the task */
			&readHttpTask,  /* Task handle. */
			0); /* Core where the task should run */
		
	delay(1000);
}

void loop() {
	long ts = micros();
	updateState();
	
//	xyzFloat gValue = myMPU6500.getGValues();
//	xyzFloat gyr = myMPU6500.getGyrValues();

	float delta = setpoint - x.x1;
	float delta_diff = delta - deltas[delta_index];
	delta_summed += delta / n_deltas;
	delta_index = (delta_index + 1) % n_deltas;
	delta_summed -= deltas[delta_index] / n_deltas;
	deltas[delta_index] = delta;

	float control = p*delta + d*delta_diff + i*delta_summed;
//	control = constrain(control, prev_control - 10, prev_control + 10);
//	prev_control = control;
	
//	Serial.print("val: ");
//	Serial.print(gValue.z, 4);
//	Serial.print(" p:");
//	Serial.print(p);
//	Serial.print(" i:");
//	Serial.print(i);
//	Serial.print(" d:");
//	Serial.print(d);
//	Serial.print(" setpoint:");
//	Serial.print(setpoint);
//	Serial.print(" delta_summed:");
//	Serial.print(delta_summed);
//	Serial.print(" control:");
//	Serial.println(control);

//	Serial.print("Accel:");
//	Serial.print(gValue.z);
//	Serial.print(" Accel-x:");
//	Serial.print(gValue.x);
//	Serial.print(" Accel-y:");
//	Serial.print(gValue.y);
//	Serial.print(" Gyro:");
//	Serial.print(gyr.z);
//	Serial.print(" Control:");
//	Serial.print(control);
//	Serial.println("");

	right.set(control);
	left.set(control);

//	long t = micros();
//	while(t - ts < 10000) {
//		t = micros();
//	}
	delay(10);
}
