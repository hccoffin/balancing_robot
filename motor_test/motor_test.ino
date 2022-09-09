#include <Encoder.h>
#include "wheel.cpp"

Wheel right(0, 0, 16, 17, 4, 33, 32, 0);
Wheel left(0, 0, 5, 18, 19, 25, 26, 1);

long prev_change_time;
int duty = 1023;
int print_i = 0;

const size_t buffer_size = 10;
long timestamp_buffer[buffer_size];
float right_tick_buffer[buffer_size];
float left_tick_buffer[buffer_size];

size_t read_index = 0;

void setup() {
	Serial.begin(115200);
	delay(1000);
	prev_change_time = micros() + 2000000; // add some extra time for first duty set
	for (int i = 0; i < buffer_size; i++) {
		timestamp_buffer[i] = micros();
		right_tick_buffer[i] = 0;
		left_tick_buffer[i] = 0;
	}	
}

//void loop() {
//	int right_tick = right.read_encoder();
//	int left_tick = left.read_encoder();
//	Serial.print(right_tick);
//	Serial.print(" ");
//	Serial.println(left_tick);
//	delay(1000);
//}

void loop() {
	right.set_duty(duty, false);
	left.set_duty(duty, false);

	int right_tick = right.read_encoder();
	int left_tick = left.read_encoder();
	long timestamp = micros();

	float dt = (float)(timestamp - timestamp_buffer[read_index]) / (1000000.0 * 60.0);
	float dticks_right = right_tick - right_tick_buffer[read_index];
	float dticks_left = left_tick - left_tick_buffer[read_index];
	float rpm_right = (dticks_right / TICKS_PER_REV) / dt;
	float rpm_left = (dticks_left / TICKS_PER_REV) / dt;

	right_tick_buffer[read_index] = right_tick;
	left_tick_buffer[read_index] = left_tick;
	timestamp_buffer[read_index] = timestamp;

	read_index = (read_index + 1) % buffer_size;

	if ((timestamp - prev_change_time) > 500000) {
		Serial.print(print_i);
		Serial.print(" ");
		Serial.print(duty);
		Serial.print(" ");
		Serial.print(rpm_right, 4);
		Serial.print(" ");
		Serial.print(rpm_left, 4);
		Serial.print(" ");
		Serial.print(dticks_right);
		Serial.print(" ");
		Serial.print(dticks_left);
		Serial.print("\n");
		if (abs(rpm_right) < 5 && abs(rpm_left) < 5) {
			duty = 0;
		} else {
			duty -= 2;
		}
		prev_change_time = timestamp;
		print_i = print_i + 1;
	}
	delay(20);
}
