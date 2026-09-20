#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"

typedef struct {
	i2c_master_dev_handle_t device;
	uint8_t stop_variable;
} vl53l0x_t;

bool vl53l0x_init(vl53l0x_t *sensor, i2c_master_bus_handle_t bus, uint8_t address);
bool vl53l0x_read_range_mm(vl53l0x_t *sensor, uint16_t *range_mm);

#endif