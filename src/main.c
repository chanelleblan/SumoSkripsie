#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "pid.h"
#include "telemetry.h"
#include "vl53l0x.h"

#define LED_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_23
#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define RIGHT_SENSOR_XSHUT_GPIO GPIO_NUM_18
#define LEFT_SENSOR_XSHUT_GPIO GPIO_NUM_19
#define IMU_INT1_GPIO GPIO_NUM_4
#define RIGHT_SENSOR_ADDRESS 0x30
#define LEFT_SENSOR_ADDRESS 0x31

#define LED_DELAY_MS 4000
#define MOTOR_START_DELAY_MS 5000
#define TARGET_DISTANCE_MM 770
#define DRIVE_SPEED 80
#define TURN_SPEED 80
#define SEARCH_SPEED 60
#define TARGET_LOST_LIMIT 8

#define MOTOR_A1_GPIO GPIO_NUM_27
#define MOTOR_A2_GPIO GPIO_NUM_26
#define MOTOR_A_PWM_GPIO GPIO_NUM_14
#define MOTOR_B1_GPIO GPIO_NUM_25
#define MOTOR_B2_GPIO GPIO_NUM_33
#define MOTOR_B_PWM_GPIO GPIO_NUM_32

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_MAX_OUTPUT 255

/* ---- PID gains -------------------------------------------------------------
 * Tune here, reflash, run, then plot with tools/telemetry.py. The gains are
 * written into every log's header, so each plot is labelled with the values
 * that produced it - without that, twenty tuning runs become twenty
 * indistinguishable graphs.
 *
 * Starting point: P only. Set PID_KI and PID_KD to 0, raise PID_KP until the
 * robot just begins to oscillate about the target, then halve it and bring in
 * PID_KD to damp what remains. PID_KI last, and sparingly - on a robot that
 * spends most of its time saturated, integral mostly buys you windup.
 * -------------------------------------------------------------------------- */
#define PID_KP 2.2f
#define PID_KI 0.0f
#define PID_KD 0.12f
#define PID_INTEGRAL_LIMIT 60.0f

/* ToF bearing estimate. The difference between the two ranges is a crude proxy
 * for bearing; this converts it to degrees, capped so a single wild reading
 * cannot command a huge setpoint jump. */
#define BEARING_DEG_PER_MM 0.06f
#define BEARING_MAX_DEG 30.0f
#define BEARING_SINGLE_SENSOR_DEG 25.0f
/* Below this range difference the target counts as dead ahead. Without the
 * dead-band the robot wags left-right instead of committing to a charge. */
#define BEARING_DEADBAND_MM 40

#define SENSOR_POLL_INTERVAL_MS 50
#define CONTROL_LOOP_TIMEOUT_MS 50

/* Commanded full drive, but barely moving and barely turning: the robot is
 * pushing something that will not move. That is a shove, not a fault. */
#define CONTACT_ACCEL_G 0.15f
#define CONTACT_YAW_RATE_DPS 20.0f

#define MATCH_TIMEOUT_MS 180000

typedef enum {
	ROBOT_IDLE,
	ROBOT_WAITING,
	ROBOT_SEARCHING,
	ROBOT_APPROACHING,
	ROBOT_STOPPED
} robot_state_t;

static const char *TAG = "sumo";

/* Written by the sensor task, read by the control task. Both are 32-bit aligned
 * scalars, but the pair must be consistent, so a spinlock guards them. */
static portMUX_TYPE target_mux = portMUX_INITIALIZER_UNLOCKED;
static float target_bearing_deg;
static bool target_visible;
static uint16_t target_range_left_mm = UINT16_MAX;
static uint16_t target_range_right_mm = UINT16_MAX;
static uint32_t target_seq;

static volatile robot_state_t robot_state = ROBOT_IDLE;

static imu_t imu;
static pid_t heading_pid;
static vl53l0x_t left_sensor;
static vl53l0x_t right_sensor;
static bool sensors_ready;
static bool imu_ready;

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

/* Motor mixing.
 *
 * The two motors are wired mirrored, which is why the original code drove them
 * with opposite signs to go forward and equal signs to spin. Preserving that:
 *
 *   forward f  ->  A += f, B -= f
 *   turn    u  ->  A -= u, B -= u      (positive u = counter-clockwise = left)
 *
 * so A = f - u and B = -f - u. Check against the original helpers: f=0, u=+t
 * gives (-t, -t), the old turn_left; f=v, u=0 gives (+v, -v), the old
 * drive_towards_target. */
static void drive_mixed(float forward, float turn, int16_t *out_a, int16_t *out_b)
{
	float a = forward - turn;
	float b = -forward - turn;

	float magnitude = fmaxf(fabsf(a), fabsf(b));
	if (magnitude > MOTOR_MAX_OUTPUT) {
		float scale = MOTOR_MAX_OUTPUT / magnitude;
		a *= scale;
		b *= scale;
	}

	*out_a = (int16_t)a;
	*out_b = (int16_t)b;
	set_motor(MOTOR_A1_GPIO, MOTOR_A2_GPIO, LEDC_CHANNEL_0, *out_a);
	set_motor(MOTOR_B1_GPIO, MOTOR_B2_GPIO, LEDC_CHANNEL_1, *out_b);
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

/* ---- Sensor task -----------------------------------------------------------
 * The VL53L0X driver is single-shot and busy-waits on the interrupt bit, so a
 * read can block for over 100 ms. Keeping it out of the control task is the
 * whole reason for the split: the PID loop must not stall behind a ranging
 * cycle. This task publishes a bearing; the control task consumes it.
 * -------------------------------------------------------------------------- */
static void sensor_task(void *arg)
{
	(void)arg;
	unsigned int lost_count = 0;

	while (1) {
		robot_state_t state = robot_state;
		if (state != ROBOT_SEARCHING && state != ROBOT_APPROACHING) {
			vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
			continue;
		}

		uint16_t left_mm = UINT16_MAX;
		uint16_t right_mm = UINT16_MAX;
		bool left_hit = vl53l0x_read_range_mm(&left_sensor, &left_mm) &&
			left_mm <= TARGET_DISTANCE_MM;
		bool right_hit = vl53l0x_read_range_mm(&right_sensor, &right_mm) &&
			right_mm <= TARGET_DISTANCE_MM;

		float bearing = 0.0f;
		bool visible = left_hit || right_hit;

		if (left_hit && right_hit) {
			int difference = (int)right_mm - (int)left_mm;
			if (abs(difference) <= BEARING_DEADBAND_MM) {
				bearing = 0.0f; /* dead ahead - commit to the charge */
			} else {
				bearing = (float)difference * BEARING_DEG_PER_MM;
				bearing = fmaxf(-BEARING_MAX_DEG, fminf(BEARING_MAX_DEG, bearing));
			}
		} else if (left_hit) {
			bearing = BEARING_SINGLE_SENSOR_DEG;
		} else if (right_hit) {
			bearing = -BEARING_SINGLE_SENSOR_DEG;
		}

		/* A ranging cycle blocks for over 100 ms, and the run can be stopped
		 * during it. Re-read the state before publishing a new one, or a stop
		 * press gets overwritten and the robot restarts itself. */
		robot_state_t state_now = robot_state;
		bool still_running = state_now == ROBOT_SEARCHING || state_now == ROBOT_APPROACHING;

		if (!still_running) {
			lost_count = 0;
		} else if (visible) {
			lost_count = 0;
			robot_state = ROBOT_APPROACHING;
		} else if (state_now == ROBOT_APPROACHING && ++lost_count < TARGET_LOST_LIMIT) {
			/* Hold the last setpoint. The gyro keeps steering toward where the
			 * target was, which is exactly what the ToF sensors cannot do at
			 * 20 Hz. */
		} else {
			lost_count = 0;
			robot_state = ROBOT_SEARCHING;
		}

		portENTER_CRITICAL(&target_mux);
		target_bearing_deg = bearing;
		target_visible = visible;
		target_range_left_mm = left_hit ? left_mm : UINT16_MAX;
		target_range_right_mm = right_hit ? right_mm : UINT16_MAX;
		if (visible) {
			target_seq++;
		}
		portEXIT_CRITICAL(&target_mux);

		vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
	}
}

/* ---- Control task ----------------------------------------------------------
 * Clocked by the IMU's data-ready interrupt at IMU_SAMPLE_RATE_HZ, so dt is set
 * by hardware rather than by however long the last iteration happened to take.
 * -------------------------------------------------------------------------- */
static void control_task(void *arg)
{
	(void)arg;
	float setpoint_deg = 0.0f;
	uint32_t last_seq = 0;
	int64_t run_start_us = 0;
	int64_t search_flip_us = 0;
	float search_direction = 1.0f;

	while (1) {
		if (!imu_wait_drdy(&imu, CONTROL_LOOP_TIMEOUT_MS)) {
			continue;
		}

		imu_sample_t sample;
		if (!imu_read(&imu, &sample)) {
			continue;
		}

		robot_state_t state = robot_state;
		if (state != ROBOT_SEARCHING && state != ROBOT_APPROACHING) {
			/* Between runs, so the next one starts its clock and its setpoint
			 * from scratch rather than inheriting the last one's. */
			run_start_us = 0;
			last_seq = 0;
			setpoint_deg = 0.0f;
			continue;
		}

		int64_t now_us = esp_timer_get_time();
		if (run_start_us == 0) {
			run_start_us = now_us;
			search_flip_us = now_us;
		}

		float bearing;
		bool visible;
		uint16_t range_left;
		uint16_t range_right;
		uint32_t seq;
		portENTER_CRITICAL(&target_mux);
		bearing = target_bearing_deg;
		visible = target_visible;
		range_left = target_range_left_mm;
		range_right = target_range_right_mm;
		seq = target_seq;
		portEXIT_CRITICAL(&target_mux);

		/* Latch a new heading setpoint only when the sensors produced a fresh
		 * fix. Between fixes the setpoint stays put and the gyro closes the
		 * loop against it - that memory is what the IMU buys us. */
		if (visible && seq != last_seq) {
			last_seq = seq;
			setpoint_deg = sample.yaw_deg + bearing;
		}

		float forward = 0.0f;
		float turn = 0.0f;
		float error = 0.0f;

		if (state == ROBOT_APPROACHING) {
			error = imu_wrap180(setpoint_deg - sample.yaw_deg);
			turn = pid_update(&heading_pid, error, sample.yaw_rate_dps, sample.dt_s);

			/* Taper forward speed with heading error: spin in place when badly
			 * misaligned, charge when lined up. */
			float alignment = 1.0f - fabsf(error) / 45.0f;
			forward = DRIVE_SPEED * fmaxf(0.0f, alignment);
		} else {
			/* Sweep for the opponent, reversing every 1.5 s so the robot does
			 * not simply spin forever in one direction. */
			if (now_us - search_flip_us > 1500000) {
				search_flip_us = now_us;
				search_direction = -search_direction;
			}
			turn = search_direction * SEARCH_SPEED;
			pid_reset(&heading_pid);
			setpoint_deg = sample.yaw_deg;
		}

		int16_t motor_a;
		int16_t motor_b;
		drive_mixed(forward, turn, &motor_a, &motor_b);

		bool contact = state == ROBOT_APPROACHING && forward > DRIVE_SPEED * 0.5f &&
			fabsf(sample.accel_fwd_g) < CONTACT_ACCEL_G &&
			fabsf(sample.yaw_rate_dps) < CONTACT_YAW_RATE_DPS;

		telem_record_t record = {
			.t_us = (uint32_t)(now_us - run_start_us),
			.yaw_ddeg = (int16_t)(sample.yaw_deg * TELEM_ANGLE_SCALE),
			.setpoint_ddeg = (int16_t)(setpoint_deg * TELEM_ANGLE_SCALE),
			.yaw_rate_ddps = (int16_t)(sample.yaw_rate_dps * TELEM_RATE_SCALE),
			.accel_fwd_mg = (int16_t)(sample.accel_fwd_g * TELEM_ACCEL_SCALE),
			.accel_lat_mg = (int16_t)(sample.accel_lat_g * TELEM_ACCEL_SCALE),
			.p_term = (int16_t)(heading_pid.p_term * TELEM_TERM_SCALE),
			.i_term = (int16_t)(heading_pid.i_term * TELEM_TERM_SCALE),
			.d_term = (int16_t)(heading_pid.d_term * TELEM_TERM_SCALE),
			.motor_a = motor_a,
			.motor_b = motor_b,
			.range_left_mm = range_left,
			.range_right_mm = range_right,
			.state = (uint8_t)state,
			.flags = (uint8_t)((visible ? TELEM_FLAG_TARGET_VISIBLE : 0) |
				(contact ? TELEM_FLAG_CONTACT : 0)),
			.reserved = 0
		};
		telemetry_log(&record);

		if (telemetry_is_full() || now_us - run_start_us > (int64_t)MATCH_TIMEOUT_MS * 1000) {
			ESP_LOGI(TAG, "run complete (%s)",
				telemetry_is_full() ? "buffer full" : "match timeout");
			robot_state = ROBOT_STOPPED;
		}
	}
}

void app_main(void)
{
	i2c_master_bus_handle_t sensor_bus;
	TickType_t sequence_start = 0;
	bool calibrated = false;

	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_GPIO, 0);
	gpio_reset_pin(BUTTON_GPIO);
	gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_GPIO, GPIO_FLOATING);
	configure_motor_gpio();
	configure_sensor_xshut_gpio();

	pid_init(&heading_pid, PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, MOTOR_MAX_OUTPUT);
	telemetry_init();

	if (configure_sensor_bus(&sensor_bus)) {
		gpio_set_level(RIGHT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		bool right_ready = vl53l0x_init(&right_sensor, sensor_bus, RIGHT_SENSOR_ADDRESS);

		gpio_set_level(LEFT_SENSOR_XSHUT_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
		bool left_ready = vl53l0x_init(&left_sensor, sensor_bus, LEFT_SENSOR_ADDRESS);
		sensors_ready = right_ready && left_ready;
		ESP_LOGI(TAG, "ToF sensors: left %s, right %s",
			left_ready ? "ok" : "FAILED", right_ready ? "ok" : "FAILED");

		imu_ready = imu_init(&imu, sensor_bus, LSM6DS3_ADDR_LOW);
		if (!imu_ready) {
			ESP_LOGE(TAG, "LSM6DS3 not found at 0x%02X - check CS is tied high (I2C mode) "
				"and SA0 is tied low", LSM6DS3_ADDR_LOW);
		} else if (!imu_start_drdy(&imu, IMU_INT1_GPIO)) {
			ESP_LOGE(TAG, "INT1 setup failed on GPIO %d", IMU_INT1_GPIO);
			imu_ready = false;
		} else {
			ESP_LOGI(TAG, "LSM6DS3 ready, data-ready on GPIO %d", IMU_INT1_GPIO);
		}
	} else {
		ESP_LOGE(TAG, "I2C bus init failed");
	}

	if (!imu_ready) {
		ESP_LOGE(TAG, "no IMU - heading control disabled, robot will not run");
	}

	if (imu_ready) {
		xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
	}
	if (sensors_ready) {
		xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 4, NULL, 0);
	}

	while (1) {
		TickType_t now = xTaskGetTickCount();
		robot_state_t state = robot_state;

		if (state == ROBOT_IDLE && gpio_get_level(BUTTON_GPIO)) {
			if (imu_ready && sensors_ready) {
				robot_state = ROBOT_WAITING;
				sequence_start = now;
				calibrated = false;
				gpio_set_level(LED_GPIO, 0);
				stop_motors();
				pid_reset(&heading_pid);
				telemetry_begin_run(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT,
					DRIVE_SPEED, TURN_SPEED, TARGET_DISTANCE_MM);
			} else {
				ESP_LOGW(TAG, "start refused: imu %s, tof %s",
					imu_ready ? "ok" : "missing", sensors_ready ? "ok" : "missing");
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
		}

		/* The countdown is not dead time: the robot is stationary, which is
		 * exactly the condition gyro bias calibration needs. */
		if (state == ROBOT_WAITING && !calibrated) {
			calibrated = true;
			if (imu_calibrate_bias(&imu, 300)) {
				ESP_LOGI(TAG, "gyro bias %.3f deg/s", imu.yaw_bias_dps);
			} else {
				ESP_LOGW(TAG, "bias calibration failed - heading will drift");
			}
		}

		if (state == ROBOT_WAITING && now - sequence_start >= pdMS_TO_TICKS(LED_DELAY_MS)) {
			gpio_set_level(LED_GPIO, 1);
		}

		if (state == ROBOT_WAITING && now - sequence_start >= pdMS_TO_TICKS(MOTOR_START_DELAY_MS)) {
			imu_reset_heading(&imu);
			pid_reset(&heading_pid);
			robot_state = ROBOT_SEARCHING;
			ESP_LOGI(TAG, "run %" PRIu32 " started", telemetry_run_number());
		}

		/* A press while running stops the match and commits the log. */
		if ((state == ROBOT_SEARCHING || state == ROBOT_APPROACHING) &&
			gpio_get_level(BUTTON_GPIO)) {
			robot_state = ROBOT_STOPPED;
		}

		if (state == ROBOT_STOPPED) {
			stop_motors();
			gpio_set_level(LED_GPIO, 0);
			telemetry_flush();
			robot_state = ROBOT_IDLE;
			/* Crude debounce: long enough that the stopping press cannot be
			 * read again as a start. */
			vTaskDelay(pdMS_TO_TICKS(1500));
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
