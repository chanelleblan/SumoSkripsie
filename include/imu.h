#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

/* LSM6DS3 I2C address. SA0/SDO tied to GND gives 0x6A, tied to 3V3 gives 0x6B.
 * docs/HARDWARE.md specifies GND. */
#define LSM6DS3_ADDR_LOW 0x6A
#define LSM6DS3_ADDR_HIGH 0x6B

/* Mounting convention (docs/HARDWARE.md section 4): the sensor's Z axis points
 * vertically, so gyro Z is yaw rate and accel X is the robot's forward axis.
 * Written down once here so it is never re-derived at 2 a.m. Flip a sign if the
 * board ends up rotated on the chassis; positive yaw must mean counter-clockwise
 * seen from above, which is the same direction the ToF bearing calls "left". */
#define IMU_YAW_SIGN (+1.0f)
#define IMU_FORWARD_SIGN (+1.0f)
#define IMU_LATERAL_SIGN (+1.0f)

/* Both accel and gyro run at this ODR. The data-ready interrupt on INT1 is what
 * clocks the control loop, so this is also the PID sample rate. */
#define IMU_SAMPLE_RATE_HZ 104.0f

typedef struct {
	i2c_master_dev_handle_t device;
	float gyro_scale_dps;   /* LSB -> deg/s */
	float accel_scale_g;    /* LSB -> g */
	float yaw_bias_dps;     /* stationary gyro Z offset, removed on every read */
	float yaw_deg;          /* integrated heading, accumulates past +/-180 */
	int64_t last_sample_us;
	void *drdy_sem;         /* SemaphoreHandle_t, NULL until imu_start_drdy() */
} imu_t;

typedef struct {
	float yaw_deg;       /* integrated heading */
	float yaw_rate_dps;  /* bias-corrected gyro Z; feeds the PID derivative */
	float accel_fwd_g;   /* forward axis, for impact/stall detection */
	float accel_lat_g;
	float dt_s;          /* actual interval since the previous sample */
} imu_sample_t;

/* Resets the part, verifies WHO_AM_I and configures 104 Hz / +/-1000 dps /
 * +/-4 g. Returns false if the device does not answer or identifies wrong -
 * check that CS is tied high (I2C mode) before suspecting anything else. */
bool imu_init(imu_t *imu, i2c_master_bus_handle_t bus, uint8_t address);

/* Routes gyro data-ready to INT1 and installs an ISR that unblocks
 * imu_wait_drdy(). Gives the control loop a hardware-timed dt. */
bool imu_start_drdy(imu_t *imu, gpio_num_t int_pin);
bool imu_wait_drdy(imu_t *imu, uint32_t timeout_ms);

/* Averages `samples` stationary readings into yaw_bias_dps. The robot must not
 * move. Call it during the start countdown. Without this the heading drifts
 * several degrees per second and the integration is worthless. */
bool imu_calibrate_bias(imu_t *imu, int samples);

bool imu_read(imu_t *imu, imu_sample_t *sample);
void imu_reset_heading(imu_t *imu);

/* Wraps any angle into [-180, +180). Use it on heading *errors* so the
 * controller always turns the short way around. */
float imu_wrap180(float degrees);

#endif
