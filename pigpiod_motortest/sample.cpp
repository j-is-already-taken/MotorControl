#include <pigpiod_if2.h>
#include <chrono>
#include <iostream>
#include <thread>

void encoder_1_call_back(int pi, uint32_t user_gpio, uint32_t level, uint32_t tick)
{
	if(user_gpio == 7){
		std::cout << "gpio pin 7 rising" << std::endl;
		std::cout << level << std::endl;
		std::cout << "--------------" << std::endl;
	}else if(user_gpio == 8){
		std::cout << "gpio pin 8 rising" << std::endl;
		std::cout << level << std::endl;
		std::cout << "--------------" << std::endl;
	}else { 
		std::cout << "otherwise" << std::endl;
		std::cout << level << std::endl;
	}
}

int main()
{
	uint32_t pwm_pin = 12;
	uint32_t motor_control1 = 26;
	uint32_t motor_control2 = 16;

	uint32_t encoder_1 = 7;
	uint32_t encoder_2 = 8;

	int pi_state = pigpio_start(nullptr, nullptr);
	if(pi_state < 0){
		std::cerr << "not started " << pi_state<< std::endl;
		return -1;
	}
	int ret = 0;
	/*
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	ret |= gpio_write(pi_state, motor_control2, PI_LOW);

	pigpio_stop(pi_state);
	return -1;
	*/

	ret |= set_mode(pi_state, pwm_pin, PI_OUTPUT);
	if(ret){
		std::cerr << "error setmode pwm" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_mode(pi_state, motor_control1, PI_OUTPUT);
	if(ret){
		std::cerr << "error setmode motor control1" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_mode(pi_state, motor_control2, PI_OUTPUT);
	if(ret){
		std::cerr << "error setmode motor control2" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_mode(pi_state, encoder_1, PI_INPUT);
	if(ret){
		std::cerr << "error setmode encorder1" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_mode(pi_state, encoder_2, PI_INPUT);
	if(ret){
		std::cerr << "error setmode encorder2" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_pull_up_down(pi_state, encoder_1, PI_PUD_DOWN);
	if(ret){
		std::cerr << "error set pull up down" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= set_pull_up_down(pi_state, encoder_2, PI_PUD_DOWN);
	if(ret){
		std::cerr << "error set pull up down" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}

	ret |= callback(pi_state, encoder_1, 1, encoder_1_call_back);
	if(ret == pigif_bad_malloc){
		std::cerr << "error setmode call_back" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= callback(pi_state, encoder_2, 1, encoder_1_call_back);
	if(ret == pigif_bad_malloc){
		std::cerr << "error setmode call_back" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}

	ret |= set_PWM_dutycycle(pi_state, pwm_pin, 255);
	if(ret == PI_BAD_USER_GPIO || ret == PI_NOT_PERMITTED){
		std::cerr << "error pwm ducycycle" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	uint32_t frequency = 50;
	ret |= set_PWM_frequency(pi_state, pwm_pin, frequency);
	if(ret == PI_BAD_USER_GPIO || ret == PI_NOT_PERMITTED){
		std::cerr << "error pwm frequency " << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	std::cout << "frequency = "  << ret << std::endl;

	ret = 0;
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	if(ret){
		std::cerr << "error gpio_write" << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	ret |= gpio_write(pi_state, motor_control2, PI_HIGH);
	if(ret){
		std::cerr << "error gpio_write " << ret << std::endl;
		pigpio_stop(pi_state);
		return -1;
	}
	using namespace std::literals::chrono_literals;
	std::cout << "all clear" << std::endl;
	std::this_thread::sleep_for(3s);

	//---------------------------------------------
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	ret |= gpio_write(pi_state, motor_control2, PI_HIGH);
	std::this_thread::sleep_for(3s);
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	ret |= gpio_write(pi_state, motor_control2, PI_LOW);
	std::this_thread::sleep_for(3s);
	ret |= gpio_write(pi_state, motor_control1, PI_HIGH);
	ret |= gpio_write(pi_state, motor_control2, PI_LOW);
	std::this_thread::sleep_for(3s);
	ret |= gpio_write(pi_state, motor_control1, PI_HIGH);
	ret |= gpio_write(pi_state, motor_control2, PI_HIGH);
	std::this_thread::sleep_for(3s);
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	ret |= gpio_write(pi_state, motor_control2, PI_LOW);

	pigpio_stop(pi_state);
	return -1;
	//--------------------------------------------


	std::cout << "all clear" << std::endl;
	for(int i=0;i<255;i++)
	{
		std::cout << i << " a, " ;
		ret |= set_PWM_dutycycle(pi_state, pwm_pin, i);
		if(ret){
			std::cerr << "error duty_cycle" << ret << std::endl;
			pigpio_stop(pi_state);
			return -1;
		}
		std::this_thread::sleep_for(3s);
	}
	ret |= gpio_write(pi_state, motor_control1, PI_LOW);
	ret |= gpio_write(pi_state, motor_control2, PI_LOW);

	pigpio_stop(pi_state);
}
