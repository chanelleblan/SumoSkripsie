#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x.h"

#define LED_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_23
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define RIGHT_SENSOR_XSHUT_GPIO GPIO_NUM_18
#define LEFT_SENSOR_XSHUT_GPIO GPIO_NUM_19
#define RIGHT_SENSOR_ADDRESS 0x30
#define LEFT_SENSOR_ADDRESS 0x31

#define LED_DELAY_MS 4000
#define MOTOR_START_DELAY_MS 5000
#define TARGET_DISTANCE_MM 770
#define DRIVE_SPEED 80
#define TURN_SPEED 80
#define TURN_10_DEGREES_MS 150
#define TARGET_LOST_LIMIT 8

#define MOTOR_A1_GPIO GPIO_NUM_27
#define MOTOR_A2_GPIO GPIO_NUM_26
#define MOTOR_A_PWM_GPIO GPIO_NUM_14
#define MOTOR_B1_GPIO GPIO_NUM_25
#define MOTOR_B2_GPIO GPIO_NUM_33
#define MOTOR_B_PWM_GPIO GPIO_NUM_32

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT

typedef enum {
	ROBOT_IDLE,
	ROBOT_WAITING,
	ROBOT_SEARCHING,
	ROBOT_APPROACHING
} robot_state_t;

static void set_motor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel, int speed)
{
	uint32_t duty = speed >= 0 ? (uint32_t)speed : (uint32_t)(-speed);

	if (speed > 0) {
		gpio_set_level(in1, 1);
		gpio_set_level(in2, 0);
	} else if (speed < 0) {
		gpio_set_level(in1, 0);
		gpio_set_level(in2, 1);
	} else {
		gpio_set_level(in1, 0);
		gpio_set_level(in2, 0);
	}

	ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void stop_motors(void)
{
	set_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, 0);
	set_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, 0);
}

static void search_for_target(void)
{
	stop_motors();
}

static void drive_towards_target(void)
{
	set_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, DRIVE_SPEED);
	set_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, -DRIVE_SPEED);
}

static void turn_left_10_degrees(void)
{
	set_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, -TURN_SPEED);
	set_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, -TURN_SPEED);
	vTaskDelay(pdMS_TO_TICKS(TURN_10_DEGREES_MS));
	stop_motors();
}

static void turn_right_10_degrees(void)
{
	set_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, TURN_SPEED);
	set_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, TURN_SPEED);
	vTaskDelay(pdMS_TO_TICKS(TURN_10_DEGREES_MS));
	stop_motors();
}

static void configure_sensor_xshut_gpio(void)
{
	const gpio_config_t xshut_config = {
		.pin_bit_mask = (1ULL << RIGHT_SENSOR_XSHUT_GPIO) | (1ULL << LEFT_SENSOR_XSHUT_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	gpio_config(&xshut_config);
	gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 0);
	gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 0);
}

static void configure_motor_gpio(void)
{
	const gpio_config_t motor_gpio_config = {
		.pin_bit_mask = (1ULL << MOTOR_A1_GPIO) | (1ULL << MOTOR_A2_GPIO) |
			(1ULL << MOTOR_B1_GPIO) | (1ULL << MOTOR_B2_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	gpio_config(&motor_gpio_config);

	const ledc_timer_config_t pwm_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = MOTOR_PWM_RESOLUTION,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = MOTOR_PWM_FREQUENCY_HZ,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm_timer);

	const ledc_channel_config_t pwm_channels[] = {
		{.gpio_num = MOTOR_A_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
		 .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
		{.gpio_num = MOTOR_B_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
		 .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0}
	};
	ledc_channel_config(&pwm_channels[0]);
	ledc_channel_config(&pwm_channels[1]);
	stop_motors();
}

static bool configure_sensor_bus(i2c_master_bus_handle_t *bus)
{
	const i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = I2C_SDA_GPIO,
		.scl_io_num = I2C_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true
	};
	return i2c_new_master_bus(&bus_config, bus) == ESP_OK;
}

void app_main(void)
{
	robot_state_t state = ROBOT_IDLE;
	TickType_t sequence_start = 0;
	TickType_t sensor_poll_time = 0;
	unsigned int target_lost_count = 0;
	i2c_master_bus_handle_t sensor_bus;
	vl53l0x_t left_sensor;
	vl53l0x_t right_sensor;
	bool sensors_ready = false;

	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_GPIO, 0);
	gpio_reset_pin(BUTTON_GPIO);
	gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_GPIO, GPIO_FLOATING);
	configure_motor_gpio();
	configure_sensor_xshut_gpio();

	if (configure_sensor_bus(&sensor_bus)) {
		gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		bool right_sensor_ready = vl53l0x_init(&right_sensor, sensor_bus, RIGHT_SENSOR_ADDRESS);

		gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		bool left_sensor_ready = vl53l0x_init(&left_sensor, sensor_bus, LEFT_SENSOR_ADDRESS);
		sensors_ready = right_sensor_ready && left_sensor_ready;
	}

	while (1) {
		TickType_t now = xTaskGetTickCount();

		if (state == ROBOT_IDLE && gpio_get_level(BUTTON_GPIO)) {
			state = ROBOT_WAITING;
			sequence_start = now;
			gpio_set_level(LED_GPIO, 0);
			stop_motors();
		}

		if (state == ROBOT_WAITING && now - sequence_start >= pdMS_TO_TICKS(LED_DELAY_MS)) {
			gpio_set_level(LED_GPIO, 1);
		}

		if (state == ROBOT_WAITING && now - sequence_start >= pdMS_TO_TICKS(MOTOR_START_DELAY_MS)) {
			state = ROBOT_SEARCHING;
			target_lost_count = 0;
			sensor_poll_time = now;
			search_for_target();
		}

		if (sensors_ready && (state == ROBOT_SEARCHING || state == ROBOT_APPROACHING) &&
			now - sensor_poll_time >= pdMS_TO_TICKS(50)) {
			uint16_t left_range_mm;
			uint16_t right_range_mm;
			bool left_target_detected = vl53l0x_read_range_mm(&left_sensor, &left_range_mm) &&
				left_range_mm <= TARGET_DISTANCE_MM;
			bool right_target_detected = vl53l0x_read_range_mm(&right_sensor, &right_range_mm) &&
				right_range_mm <= TARGET_DISTANCE_MM;
			bool target_detected = left_target_detected || right_target_detected;
			sensor_poll_time = now;

			if (target_detected) {
				state = ROBOT_APPROACHING;
				target_lost_count = 0;
				if (left_target_detected && right_target_detected && left_range_mm < right_range_mm) {
					turn_left_10_degrees();
				} else if (left_target_detected && !right_target_detected) {
					turn_left_10_degrees();
				} else if (right_target_detected && !left_target_detected) {
					turn_right_10_degrees();
				} else if (right_target_detected && left_range_mm > right_range_mm) {
					turn_right_10_degrees();
				} else {
					drive_towards_target();
				}
			} else if (state == ROBOT_APPROACHING &&
				++target_lost_count < TARGET_LOST_LIMIT) {
				drive_towards_target();
			} else {
				state = ROBOT_SEARCHING;
				target_lost_count = 0;
				search_for_target();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
