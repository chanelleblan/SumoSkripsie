#include "vl53l0x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L0X_ADDRESS 0x29
#define VL53L0X_TIMEOUT_MS 100
#define SYSRANGE_START 0x00
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define SYSTEM_INTERRUPT_CLEAR 0x0B
#define RESULT_INTERRUPT_STATUS 0x13
#define RESULT_RANGE_STATUS 0x14

static bool write_register(vl53l0x_t *sensor, uint8_t reg, uint8_t value)
{
	uint8_t data[] = {reg, value};
	return i2c_master_transmit(sensor->device, data, sizeof(data), VL53L0X_TIMEOUT_MS) == ESP_OK;
}

static bool read_registers(vl53l0x_t *sensor, uint8_t reg, uint8_t *data, size_t length)
{
	return i2c_master_transmit_receive(sensor->device, &reg, 1, data, length,
		VL53L0X_TIMEOUT_MS) == ESP_OK;
}

static bool read_register(vl53l0x_t *sensor, uint8_t reg, uint8_t *value)
{
	return read_registers(sensor, reg, value, 1);
}

static bool write_registers(vl53l0x_t *sensor, const uint8_t *data, size_t length)
{
	return i2c_master_transmit(sensor->device, data, length, VL53L0X_TIMEOUT_MS) == ESP_OK;
}

static bool wait_for_interrupt(vl53l0x_t *sensor)
{
	uint8_t status;
	for (int attempt = 0; attempt < 100; ++attempt) {
		if (!read_register(sensor, RESULT_INTERRUPT_STATUS, &status)) {
			return false;
		}
		if (status & 0x07) {
			return true;
		}
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	return false;
}

static bool get_spad_info(vl53l0x_t *sensor, uint8_t *count, bool *aperture)
{
	uint8_t value;
	uint8_t spad_info;

	if (!write_register(sensor, 0x80, 0x01) || !write_register(sensor, 0xFF, 0x01) ||
		!write_register(sensor, 0x00, 0x00) || !write_register(sensor, 0xFF, 0x06) ||
		!read_register(sensor, 0x83, &value) || !write_register(sensor, 0x83, value | 0x04) ||
		!write_register(sensor, 0xFF, 0x07) || !write_register(sensor, 0x81, 0x01) ||
		!write_register(sensor, 0x80, 0x01) || !write_register(sensor, 0x94, 0x6B) ||
		!write_register(sensor, 0x83, 0x00)) {
		return false;
	}

	for (int attempt = 0; attempt < 100; ++attempt) {
		if (!read_register(sensor, 0x83, &value)) {
			return false;
		}
		if (value != 0) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(2));
		if (attempt == 99) {
			return false;
		}
	}

	if (!write_register(sensor, 0x83, 0x01) || !read_register(sensor, 0x92, &spad_info) ||
		!write_register(sensor, 0x81, 0x00) || !write_register(sensor, 0xFF, 0x06) ||
		!read_register(sensor, 0x83, &value) || !write_register(sensor, 0x83, value & (uint8_t)~0x04) ||
		!write_register(sensor, 0xFF, 0x01) || !write_register(sensor, 0x00, 0x01) ||
		!write_register(sensor, 0xFF, 0x00) || !write_register(sensor, 0x80, 0x00)) {
		return false;
	}

	*count = spad_info & 0x7F;
	*aperture = (spad_info & 0x80) != 0;
	return true;
}

static bool load_tuning_settings(vl53l0x_t *sensor)
{
	static const uint8_t settings[][2] = {
		{0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
		{0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
		{0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
		{0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
		{0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
		{0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
		{0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
		{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
		{0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
		{0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
		{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
		{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
		{0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
		{0xFF,0x00},{0x80,0x00}
	};

	for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
		if (!write_register(sensor, settings[i][0], settings[i][1])) {
			return false;
		}
	}
	return true;
}

static bool perform_reference_calibration(vl53l0x_t *sensor, uint8_t init_value)
{
	return write_register(sensor, SYSRANGE_START, 0x01 | init_value) &&
		wait_for_interrupt(sensor) &&
		write_register(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01) &&
		write_register(sensor, SYSRANGE_START, 0x00);
}

static bool configure_sensor(vl53l0x_t *sensor)
{
	uint8_t model_id;
	uint8_t stop_variable;
	uint8_t spad_count;
	uint8_t spad_map[6];
	bool aperture;

	if (!read_register(sensor, 0xC0, &model_id) || model_id != 0xEE ||
		!write_register(sensor, 0x88, 0x00) || !write_register(sensor, 0x80, 0x01) ||
		!write_register(sensor, 0xFF, 0x01) || !write_register(sensor, 0x00, 0x00) ||
		!read_register(sensor, 0x91, &stop_variable) || !write_register(sensor, 0x00, 0x01) ||
		!write_register(sensor, 0xFF, 0x00) || !write_register(sensor, 0x80, 0x00) ||
		!read_register(sensor, 0x60, &model_id) || !write_register(sensor, 0x60, model_id | 0x12) ||
		!write_register(sensor, 0x44, 0x00) || !write_register(sensor, 0x45, 0x20) ||
		!write_register(sensor, SYSTEM_SEQUENCE_CONFIG, 0xFF) ||
		!get_spad_info(sensor, &spad_count, &aperture) ||
		!read_registers(sensor, 0xB0, spad_map, sizeof(spad_map))) {
		return false;
	}

	uint8_t first_spad = aperture ? 12 : 0;
	uint8_t enabled = 0;
	for (uint8_t i = 0; i < 48; ++i) {
		if (i < first_spad || enabled == spad_count) {
			spad_map[i / 8] &= (uint8_t)~(1 << (i % 8));
		} else if (spad_map[i / 8] & (1 << (i % 8))) {
			enabled++;
		}
	}

	uint8_t spad_write[7] = {0xB0, spad_map[0], spad_map[1], spad_map[2],
		spad_map[3], spad_map[4], spad_map[5]};
	if (!write_registers(sensor, spad_write, sizeof(spad_write)) ||
		!load_tuning_settings(sensor) || !write_register(sensor, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) ||
		!write_register(sensor, 0x84, 0x00) || !write_register(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01) ||
		!write_register(sensor, SYSTEM_SEQUENCE_CONFIG, 0x01) ||
		!perform_reference_calibration(sensor, 0x40) ||
		!write_register(sensor, SYSTEM_SEQUENCE_CONFIG, 0x02) ||
		!perform_reference_calibration(sensor, 0x00) ||
		!write_register(sensor, SYSTEM_SEQUENCE_CONFIG, 0xE8) ||
		!write_register(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01)) {
		return false;
	}

	sensor->stop_variable = stop_variable;
	return true;
}

bool vl53l0x_init(vl53l0x_t *sensor, i2c_master_bus_handle_t bus, uint8_t address)
{
	const i2c_device_config_t default_device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = VL53L0X_ADDRESS,
		.scl_speed_hz = 400000
	};

	if (i2c_master_bus_add_device(bus, &default_device_config, &sensor->device) != ESP_OK) {
		return false;
	}
	if (!configure_sensor(sensor) || !write_register(sensor, 0x8A, address)) {
		i2c_master_bus_rm_device(sensor->device);
		return false;
	}

	i2c_master_bus_rm_device(sensor->device);
	const i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address,
		.scl_speed_hz = 400000
	};
	return i2c_master_bus_add_device(bus, &device_config, &sensor->device) == ESP_OK;
}

bool vl53l0x_read_range_mm(vl53l0x_t *sensor, uint16_t *range_mm)
{
	uint8_t range_data[12];

	if (!write_register(sensor, 0x80, 0x01) || !write_register(sensor, 0xFF, 0x01) ||
		!write_register(sensor, 0x00, 0x00) || !write_register(sensor, 0x91, sensor->stop_variable) ||
		!write_register(sensor, 0x00, 0x01) || !write_register(sensor, 0xFF, 0x00) ||
		!write_register(sensor, 0x80, 0x00) || !write_register(sensor, SYSRANGE_START, 0x01)) {
		return false;
	}

	for (int attempt = 0; attempt < 100; ++attempt) {
		uint8_t start;
		if (!read_register(sensor, SYSRANGE_START, &start)) {
			return false;
		}
		if ((start & 0x01) == 0) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(2));
		if (attempt == 99) {
			return false;
		}
	}

	if (!wait_for_interrupt(sensor) ||
		!read_registers(sensor, RESULT_RANGE_STATUS, range_data, sizeof(range_data)) ||
		!write_register(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01)) {
		return false;
	}

	*range_mm = ((uint16_t)range_data[10] << 8) | range_data[11];
	return true;
}
