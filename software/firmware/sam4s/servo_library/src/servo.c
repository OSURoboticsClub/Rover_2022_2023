#include "servo.h"

#define PWM_CLK_SPEED 1000000
#define PWM_FREQ 50
#define PWM_PERIOD (PWM_CLK_SPEED / PWM_FREQ)
#define MICROSECONDS 1000000.0

static void _init_pwm(servo_s *servo) {
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, servo->pwm_channel_num);
	
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_CLK_SPEED,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_bus_hz(PWM)
	};
	pwm_init(PWM, &clock_setting);
}

static void _pwm_set_duty(servo_s *servo, unsigned duty_us) {
	// Duty cycle is given in microseconds so convert to actual clock ticks
	unsigned duty_ticks = (PWM_CLK_SPEED / MICROSECONDS) * duty_us;
	
	pwm_channel_t pwm_channel_instance = {
		.ul_prescaler = PWM_CMR_CPRE_CLKA,
		.ul_period = PWM_PERIOD,
		.ul_duty = PWM_PERIOD - duty_ticks,
		.channel = servo->pwm_channel_num
	};
	servo->pwm_channel = pwm_channel_instance;
	
	pwm_channel_disable(PWM, servo->pwm_channel_num);
	pwm_channel_init(PWM, &servo->pwm_channel);
	pwm_channel_enable(PWM, servo->pwm_channel_num);
}

void servo_setup(servo_s *servo, uint32_t pwm_channel_num, unsigned us_min, unsigned us_max, unsigned us_center) {
	servo->pwm_channel_num = pwm_channel_num;
	servo->us_min = us_min;
	servo->us_max = us_max;
	servo->us_center = us_center;
	
	uint32_t pwm_pin;
	switch (pwm_channel_num) {
		case PWM_CHANNEL_0:
			pwm_pin = PIO_PA11;
			break;
		case PWM_CHANNEL_1:
			pwm_pin = PIO_PA12;
			break;
		case PWM_CHANNEL_2:
			pwm_pin = PIO_PA13;
			break;
		case PWM_CHANNEL_3:
			pwm_pin = PIO_PA7;
			break;
		default:
			return;
	}
	
	pio_set_peripheral(PIOA, PIO_PERIPH_B, pwm_pin);
	_init_pwm(servo);
	
	servo->position = us_center;
	_pwm_set_duty(servo, us_center);
}

void servo_write_us(servo_s *servo, unsigned us) {
	if (us > servo->us_max) {
		us = servo->us_max;
	} else if (us < servo->us_min) {
		us = servo->us_min;
	}
	
	_pwm_set_duty(servo, us);
	servo->position = us;
}

void servo_write_angle(servo_s *servo, unsigned angle) {
	int us = angle * (servo->us_max - servo->us_min) / (180) + servo->us_min;
	servo_write_us(servo, us);
}
