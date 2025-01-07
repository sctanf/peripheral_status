#include "../globals.h"
#include "../util.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define LED0_NODE DT_NODELABEL(pwm_led0)
#define LED1_NODE DT_NODELABEL(pwm_led1)
#define LED2_NODE DT_NODELABEL(pwm_led2)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#elif DT_NODE_EXISTS(DT_ALIAS(led0))
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#warning "LED GPIO does not exist"
//static const struct gpio_dt_spec led = {0};
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

#if DT_NODE_EXISTS(LED0_NODE)
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(LED0_NODE);
#else
#warning "PWM LED node does not exist"
#endif
#if DT_NODE_EXISTS(LED1_NODE)
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(LED1_NODE);
#endif
#if DT_NODE_EXISTS(LED2_NODE)
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(LED2_NODE);
#endif

#if LED_EXISTS
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH] = {[0 ... (SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static enum sys_led_pattern current_led_pattern;
static int current_priority;
static int led_pattern_state;

static int led_pin_init(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#endif
#endif

#ifdef LED_RGB_COLOR
static float led_pwm_period[4][3] = {
	{0.33, 0.33, 0.33}, // Default
	{0.44, 0.56, 0}, // Success
	{1, 0, 0}, // Error
	{0.73, 0.27, 0}, // Charging
};
#elif defined(LED_TRI_COLOR)
static float led_pwm_period[4][3] = {
	{0, 0, 1}, // Default
	{0, 1, 0}, // Success
	{1, 0, 0}, // Error
	{0.5, 0.5, 0}, // Charging
};
#elif defined(LED_DUAL_COLOR)
static float led_pwm_period[4][2] = {
	{0, 1}, // Default
	{0, 1}, // Success
	{1, 0}, // Error
	{0.5, 0.5}, // Charging
};
#else
static float led_pwm_period[4][1] = {
	{1}, // Default
	{1}, // Success
	{1}, // Error
	{1}, // Charging
};
#endif

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
static void led_pin_set(enum sys_led_color color, float brightness, float value)
{
	if (brightness < 0)
		brightness = 0;
	else if (brightness > 1)
		brightness = 1;
	if (value < 0)
		value = 0;
	else if (value > 1)
		value = 1;
#if PWM_LED_EXISTS
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period * led_pwm_period[color][0] * value * brightness);
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period * led_pwm_period[color][1] * value * brightness);
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period * led_pwm_period[color][2] * value * brightness);
#endif
#endif
#else
	gpio_pin_set_dt(&led, value > 0.5f);
#endif
}

// reset all led pins
static void led_pin_reset()
{
// setting PWM to 0 constantly is expensive..
//#if PWM_LED_EXISTS
//	pwm_set_pulse_dt(&pwm_led, 0);
//#endif
//#if PWM_LED1_EXISTS
//	pwm_set_pulse_dt(&pwm_led1, 0);
//#endif
//#if PWM_LED2_EXISTS
//	pwm_set_pulse_dt(&pwm_led2, 0);
//#endif
	led_pin_init(); // reinit led
	gpio_pin_set_dt(&led, 0);
#if LED1_EXISTS
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_set_dt(&led2, 0);
#endif
}
#endif

void set_led(enum sys_led_pattern led_pattern, int priority)
{
#if LED_EXISTS
	if (led_pattern <= SYS_LED_PATTERN_OFF && k_current_get() == led_thread_id)
		led_patterns[current_priority] = led_pattern;
	else
		led_patterns[priority] = led_pattern;
	for (priority = 0; priority < SYS_LED_PATTERN_DEPTH; priority++)
	{
		if (led_patterns[priority] == SYS_LED_PATTERN_OFF)
			continue;
		led_pattern = led_patterns[priority];
		break;
	}
	if (led_pattern == current_led_pattern && led_pattern > SYS_LED_PATTERN_OFF)
		return;
	current_led_pattern = led_pattern;
	current_priority = priority;
	led_pattern_state = 0;
	if (current_led_pattern <= SYS_LED_PATTERN_OFF)
	{
		led_pin_reset();
		k_thread_suspend(led_thread_id);
	}
	else if (k_current_get() != led_thread_id) // do not suspend if called from thread
	{
		k_thread_suspend(led_thread_id);
		k_thread_resume(led_thread_id);
	}
	else
	{
		k_thread_resume(led_thread_id);
	}
#endif
}

static void led_thread(void)
{
#if !LED_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	while (1)
	{
		led_pin_reset();
		switch (current_led_pattern)
		{
		case SYS_LED_PATTERN_ON:
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, 1);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_SHORT:
			led_pattern_state = (led_pattern_state + 1) % 2;
//			gpio_pin_set_dt(&led, led_pattern_state);
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, led_pattern_state);
			k_msleep(led_pattern_state == 1 ? 100 : 900);
			break;
		case SYS_LED_PATTERN_LONG:
			led_pattern_state = (led_pattern_state + 1) % 2;
//			gpio_pin_set_dt(&led, led_pattern_state);
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, led_pattern_state);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_FLASH:
			led_pattern_state = (led_pattern_state + 1) % 2;
//			gpio_pin_set_dt(&led, led_pattern_state);
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, led_pattern_state);
			k_msleep(200);
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
//			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, !(led_pattern_state % 2));
			if (led_pattern_state == 7)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0)
//#if PWM_LED_EXISTS
//				pwm_set_pulse_dt(&pwm_led, pwm_led.period * (202 - led_pattern_state) / 200);
//				led_pin_set(SYS_LED_COLOR_DEFAULT, 1, (202 - led_pattern_state) / 200.f);
//#else
//				gpio_pin_set_dt(&led, 202 - led_pattern_state);
//				led_pin_set(SYS_LED_COLOR_DEFAULT, 1, 202 - led_pattern_state);
//#endif
				led_pin_set(SYS_LED_COLOR_DEFAULT, (202 - led_pattern_state) / 200.f, 202 - led_pattern_state);
			else
//				gpio_pin_set_dt(&led, 0);
				led_pin_set(SYS_LED_COLOR_DEFAULT, 1, 0);
			if (led_pattern_state == 202)
				set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			else if (led_pattern_state == 1)
				k_msleep(250);
			else
				k_msleep(5);
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			led_pattern_state++;
//			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			led_pin_set(SYS_LED_COLOR_SUCCESS, 1, !(led_pattern_state % 2));
			if (led_pattern_state == 5)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			led_pattern_state++;
//			gpio_pin_set_dt(&led, !(led_pattern_state % 2));
			led_pin_set(SYS_LED_COLOR_SUCCESS, 1, !(led_pattern_state % 2));
			if (led_pattern_state == 9)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			else
				k_msleep(200);
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
//#if PWM_LED_EXISTS
//			pwm_set_pulse_dt(&pwm_led, pwm_led.period / 5); // 20% duty cycle, should look like ~50% brightness
//#else
//			gpio_pin_set_dt(&led, 1);
//#endif
			led_pin_set(SYS_LED_COLOR_SUCCESS, 0.2, 1);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 2;
//#if PWM_LED_EXISTS
//			pwm_set_pulse_dt(&pwm_led, led_pattern_state ? pwm_led.period / 5 : 0); // 20% duty cycle, should look like ~50% brightness
//#else
//			gpio_pin_set_dt(&led, led_pattern_state);
//#endif
			led_pin_set(SYS_LED_COLOR_CHARGING, 0.2, led_pattern_state);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 1000;
			float led_value = sinf(led_pattern_state * (M_PI / 1000));
//#if PWM_LED_EXISTS
//			pwm_set_pulse_dt(&pwm_led, pwm_led.period * led_value);
//#else
//			gpio_pin_set_dt(&led, led_value > 0.5f);
//#endif
			led_pin_set(SYS_LED_COLOR_CHARGING, 1, led_value);
			k_msleep(5);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST: // off duration first because the device may turn on multiple times rapidly and waste battery power
			led_pattern_state = (led_pattern_state + 1) % 2;
//			gpio_pin_set_dt(&led, !led_pattern_state);
			led_pin_set(SYS_LED_COLOR_DEFAULT, 1, !led_pattern_state);
			k_msleep(led_pattern_state ? 9700 : 300);
			break;

		case SYS_LED_PATTERN_ERROR_A: // TODO: should this use 20% duty cycle?
			led_pattern_state = (led_pattern_state + 1) % 10;
//			gpio_pin_set_dt(&led, led_pattern_state < 4 && led_pattern_state % 2);
			led_pin_set(SYS_LED_COLOR_ERROR, 1, led_pattern_state < 4 && led_pattern_state % 2);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_B:
			led_pattern_state = (led_pattern_state + 1) % 10;
//			gpio_pin_set_dt(&led, led_pattern_state < 6 && led_pattern_state % 2);
			led_pin_set(SYS_LED_COLOR_ERROR, 1, led_pattern_state < 6 && led_pattern_state % 2);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_C:
			led_pattern_state = (led_pattern_state + 1) % 10;
//			gpio_pin_set_dt(&led, led_pattern_state < 8 && led_pattern_state % 2);
			led_pin_set(SYS_LED_COLOR_ERROR, 1, led_pattern_state < 8 && led_pattern_state % 2);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_D:
			led_pattern_state = (led_pattern_state + 1) % 2;
//			gpio_pin_set_dt(&led, led_pattern_state);
			led_pin_set(SYS_LED_COLOR_ERROR, 1, led_pattern_state);
			k_msleep(500);
			break;

		default:
			k_thread_suspend(led_thread_id);
		}
	}
#endif
}
