#include <Encoder.h>
#include <math.h>

#define TICKS_PER_REV 445.2
//#define PWM_FREQ 2000
#define PWM_FREQ 89
#define PWM_RES 10

class Wheel {
	public:
		Wheel(float rate, float intercept, uint8_t pinf, uint8_t pinr, uint8_t pin_pwm, uint8_t enc1, uint8_t enc2, uint8_t motor_chn) 
		  : encoder(enc1, enc2)
		{
			this->rate = rate;
			this->intercept = intercept;

			this->pinf = pinf;
			this->pinr = pinr;
			this->motor_chn = motor_chn;

			pinMode(pinf, OUTPUT);
			pinMode(pinr, OUTPUT);
			pinMode(pin_pwm, OUTPUT);

			ledcSetup(motor_chn, PWM_FREQ, PWM_RES);
			ledcAttachPin(pin_pwm, motor_chn);
		}

		void set(float rpm) {
			if (abs(rpm) < 1) {
				uint8_t duty = 0;
				ledcWrite(this->motor_chn, duty);
			} else if (rpm > 0) {
				uint8_t duty = min(round((rpm / this->rate) + this->intercept), 255.0);
				Serial.println(duty);
				digitalWrite(this->pinf, HIGH);
				digitalWrite(this->pinr, LOW);
				ledcWrite(this->motor_chn, duty);
			} else {
				uint8_t duty = min(round((-rpm / this->rate) + this->intercept), 255.0);
				Serial.println(duty);
				digitalWrite(this->pinf, LOW);
				digitalWrite(this->pinr, HIGH);
				ledcWrite(this->motor_chn, duty);
			}
		}

		void set_duty(int duty, boolean forward=true) {
			if (forward) {
				digitalWrite(this->pinf, HIGH);
				digitalWrite(this->pinr, LOW);
			} else {
				digitalWrite(this->pinf, LOW);
				digitalWrite(this->pinr, HIGH);
			}
			ledcWrite(this->motor_chn, duty);
		}

		int read_encoder() {
			return encoder.read();
		}

	private:
//        const float min_rpm = 120;
		float rate;
		float intercept;

		uint8_t pinf;
		uint8_t pinr;
		uint8_t motor_chn;

		Encoder encoder;
};
