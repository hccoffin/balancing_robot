#include <Encoder.h>
#include <math.h>

#define TICKS_PER_REV 445.2
#define PWM_FREQ 89
#define PWM_RES 10

class Wheel {
	public:
		Wheel(float intercept, uint8_t pinf, uint8_t pinr, uint8_t pin_pwm, uint8_t enc1, uint8_t enc2, uint8_t motor_chn) 
		  : encoder(enc1, enc2)
		{
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

		void set(float value) {
			if (abs(value) < 1) {
				int duty = 0;
				ledcWrite(this->motor_chn, duty);
			} else if (value > 0) {
				int duty = min(round(value + this->intercept), 1023.0);
				digitalWrite(this->pinf, HIGH);
				digitalWrite(this->pinr, LOW);
				ledcWrite(this->motor_chn, duty);
			} else {
				int duty = min(round(-value + this->intercept), 1023.0);
				digitalWrite(this->pinf, LOW);
				digitalWrite(this->pinr, HIGH);
				ledcWrite(this->motor_chn, duty);
			}
		}

	private:
//        const float min_rpm = 120;
		float intercept;

		uint8_t pinf;
		uint8_t pinr;
		uint8_t motor_chn;

		Encoder encoder;
};
