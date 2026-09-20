#include "imu.h"

#include <math.h>

#include "esp_attr.h" /* IRAM_ATTR on the data-ready ISR */
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define IMU_TIMEOUT_MS 100

#define REG_INT1_CTRL 0x0D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_STATUS 0x1E
#define REG_OUTX_L_G 0x22

/* 0x69 is the LSM6DS3, 0x6A the LSM6DS3TR-C / LSM6DS3-C. The TR-C's WHO_AM_I
 * value coincides with the other variant's bus address; the two are unrelated. */
#define WHO_AM_I_LSM6DS3 0x69
#define WHO_AM_I_LSM6DS3TRC 0x6A

/* ODR 104 Hz in the high nibble of CTRL1_XL / CTRL2_G. */
#define ODR_104_HZ 0x40

/* CTRL1_XL full-scale bits. Note the datasheet's ordering is not monotonic:
 * 00 = +/-2 g, 01 = +/-16 g, 10 = +/-4 g, 11 = +/-8 g. */
#define FS_XL_4G 0x08
#define ACCEL_SCALE_4G_MG 0.122f

/* CTRL2_G full-scale bits: 00 = 245, 01 = 500, 10 = 1000, 11 = 2000 dps.
 * 1000 dps leaves headroom for the spin an impact imparts; its 35 mdps/LSB is
 * already below the part's noise floor over this bandwidth, so the coarser
 * scale costs no usable resolution. */
#define FS_G_1000DPS 0x08
#define GYRO_SCALE_1000DPS_MDPS 35.0f

/* The first few conversions after configuration are garbage. */
#define IMU_SETTLING_SAMPLES 20

static bool write_register(imu_t *imu, uint8_t reg, uint8_t value)
{
	uint8_t data[] = {reg, value};
	return i2c_master_transmit(imu->device, data, sizeof(data), IMU_TIMEOUT_MS) == ESP_OK;
}

static bool read_registers(imu_t *imu, uint8_t reg, uint8_t *data, size_t length)
{
	return i2c_master_transmit_receive(imu->device, &reg, 1, data, length,
		IMU_TIMEOUT_MS) == ESP_OK;
}

static bool read_register(imu_t *imu, uint8_t reg, uint8_t *value)
{
	return read_registers(imu, reg, value, 1);
}

static void IRAM_ATTR drdy_isr(void *arg)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR((SemaphoreHandle_t)arg, &higher_priority_task_woken);
	if (higher_priority_task_woken) {
		portYIELD_FROM_ISR();
	}
}

float imu_wrap180(float degrees)
{
	degrees = fmodf(degrees + 180.0f, 360.0f);
	if (degrees < 0.0f) {
		degrees += 360.0f;
	}
	return degrees - 180.0f;
}

void imu_reset_heading(imu_t *imu)
{
	imu->yaw_deg = 0.0f;
	imu->last_sample_us = 0;
}

bool imu_init(imu_t *imu, i2c_master_bus_handle_t bus, uint8_t address)
{
	const i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address,
		.scl_speed_hz = 400000
	};

	imu->gyro_scale_dps = GYRO_SCALE_1000DPS_MDPS / 1000.0f;
	imu->accel_scale_g = ACCEL_SCALE_4G_MG / 1000.0f;
	imu->yaw_bias_dps = 0.0f;
	imu->yaw_deg = 0.0f;
	imu->last_sample_us = 0;
	imu->drdy_sem = NULL;

	if (i2c_master_bus_add_device(bus, &device_config, &imu->device) != ESP_OK) {
		return false;
	}

	uint8_t who_am_i;
	if (!read_register(imu, REG_WHO_AM_I, &who_am_i) ||
		(who_am_i != WHO_AM_I_LSM6DS3 && who_am_i != WHO_AM_I_LSM6DS3TRC)) {
		i2c_master_bus_rm_device(imu->device);
		return false;
	}

	/* Software reset, then wait for the bit to self-clear. */
	if (!write_register(imu, REG_CTRL3_C, 0x01)) {
		i2c_master_bus_rm_device(imu->device);
		return false;
	}
	for (int attempt = 0; attempt < 20; ++attempt) {
		uint8_t ctrl3;
		vTaskDelay(pdMS_TO_TICKS(5));
		if (!read_register(imu, REG_CTRL3_C, &ctrl3)) {
			i2c_master_bus_rm_device(imu->device);
			return false;
		}
		if ((ctrl3 & 0x01) == 0) {
			break;
		}
	}

	/* BDU so a burst read cannot straddle two conversions, IF_INC so the burst
	 * auto-increments the register pointer. */
	if (!write_register(imu, REG_CTRL3_C, 0x44) ||
		!write_register(imu, REG_CTRL1_XL, ODR_104_HZ | FS_XL_4G) ||
		!write_register(imu, REG_CTRL2_G, ODR_104_HZ | FS_G_1000DPS) ||
		!write_register(imu, REG_CTRL4_C, 0x00)) {
		i2c_master_bus_rm_device(imu->device);
		return false;
	}

	imu_sample_t discard;
	for (int i = 0; i < IMU_SETTLING_SAMPLES; ++i) {
		vTaskDelay(pdMS_TO_TICKS(10));
		imu_read(imu, &discard);
	}
	imu_reset_heading(imu);
	return true;
}

bool imu_start_drdy(imu_t *imu, gpio_num_t int_pin)
{
	SemaphoreHandle_t sem = xSemaphoreCreateBinary();
	if (sem == NULL) {
		return false;
	}

	const gpio_config_t int_config = {
		.pin_bit_mask = 1ULL << int_pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE
	};
	if (gpio_config(&int_config) != ESP_OK) {
		vSemaphoreDelete(sem);
		return false;
	}

	/* The service may already be installed by another module; that is not an
	 * error for us. */
	esp_err_t isr_service = gpio_install_isr_service(0);
	if (isr_service != ESP_OK && isr_service != ESP_ERR_INVALID_STATE) {
		vSemaphoreDelete(sem);
		return false;
	}
	if (gpio_isr_handler_add(int_pin, drdy_isr, sem) != ESP_OK) {
		vSemaphoreDelete(sem);
		return false;
	}

	/* Gyro data-ready drives INT1. Accel shares the ODR, so one line is enough. */
	if (!write_register(imu, REG_INT1_CTRL, 0x02)) {
		gpio_isr_handler_remove(int_pin);
		vSemaphoreDelete(sem);
		return false;
	}

	imu->drdy_sem = sem;
	return true;
}

bool imu_wait_drdy(imu_t *imu, uint32_t timeout_ms)
{
	if (imu->drdy_sem == NULL) {
		return false;
	}
	return xSemaphoreTake((SemaphoreHandle_t)imu->drdy_sem,
		pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool imu_read(imu_t *imu, imu_sample_t *sample)
{
	uint8_t raw[12];
	if (!read_registers(imu, REG_OUTX_L_G, raw, sizeof(raw))) {
		return false;
	}

	int16_t gyro_z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
	int16_t accel_x = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
	int16_t accel_y = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);

	int64_t now_us = esp_timer_get_time();
	float dt_s = 0.0f;
	if (imu->last_sample_us != 0) {
		dt_s = (float)(now_us - imu->last_sample_us) / 1000000.0f;
		/* A stall elsewhere must not inject a huge integration step. */
		if (dt_s > 0.2f) {
			dt_s = 0.2f;
		}
	}
	imu->last_sample_us = now_us;

	float yaw_rate = IMU_YAW_SIGN * ((float)gyro_z * imu->gyro_scale_dps) - imu->yaw_bias_dps;
	imu->yaw_deg += yaw_rate * dt_s;

	sample->yaw_deg = imu->yaw_deg;
	sample->yaw_rate_dps = yaw_rate;
	sample->accel_fwd_g = IMU_FORWARD_SIGN * (float)accel_x * imu->accel_scale_g;
	sample->accel_lat_g = IMU_LATERAL_SIGN * (float)accel_y * imu->accel_scale_g;
	sample->dt_s = dt_s;
	return true;
}

bool imu_calibrate_bias(imu_t *imu, int samples)
{
	double sum_dps = 0.0;
	int collected = 0;

	imu->yaw_bias_dps = 0.0f;
	for (int i = 0; i < samples; ++i) {
		imu_sample_t sample;
		/* Fall back to fixed-rate polling if INT1 is not wired. */
		if (imu->drdy_sem != NULL) {
			if (!imu_wait_drdy(imu, 50)) {
				continue;
			}
		} else {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if (imu_read(imu, &sample)) {
			sum_dps += sample.yaw_rate_dps;
			collected++;
		}
	}

	if (collected < samples / 2) {
		return false;
	}
	imu->yaw_bias_dps = (float)(sum_dps / collected);
	imu_reset_heading(imu);
	return true;
}
