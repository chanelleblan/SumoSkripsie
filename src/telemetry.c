#include "telemetry.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_partition.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "imu.h" /* IMU_SAMPLE_RATE_HZ, recorded in the header */

#define TELEM_HEADER_OFFSET 0
#define TELEM_FOOTER_OFFSET 64
#define TELEM_RECORDS_OFFSET 128
#define FLASH_SECTOR_SIZE 4096

static const char *TAG = "telemetry";

static const esp_partition_t *partition;
static telem_record_t *buffer;
static uint32_t sample_count;
static uint32_t dropped_count;
static uint32_t run_number;
static int64_t run_start_us;

bool telemetry_init(void)
{
	partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
		ESP_PARTITION_SUBTYPE_ANY, "telemetry");
	if (partition == NULL) {
		ESP_LOGE(TAG, "no 'telemetry' partition - is CONFIG_PARTITION_TABLE_CUSTOM set?");
		return false;
	}

	size_t required = TELEM_RECORDS_OFFSET + (size_t)TELEM_CAPACITY * sizeof(telem_record_t);
	if (partition->size < required) {
		ESP_LOGE(TAG, "partition %" PRIu32 " B < required %u B", partition->size, (unsigned)required);
		return false;
	}

	buffer = malloc((size_t)TELEM_CAPACITY * sizeof(telem_record_t));
	if (buffer == NULL) {
		ESP_LOGE(TAG, "cannot allocate %u B buffer", (unsigned)(TELEM_CAPACITY * sizeof(telem_record_t)));
		return false;
	}

	ESP_LOGI(TAG, "ready: %u records, %u B RAM, partition at 0x%" PRIx32,
		(unsigned)TELEM_CAPACITY,
		(unsigned)(TELEM_CAPACITY * sizeof(telem_record_t)),
		partition->address);
	return true;
}

static uint32_t read_previous_run_number(void)
{
	telem_header_t previous;
	if (esp_partition_read(partition, TELEM_HEADER_OFFSET, &previous, sizeof(previous)) != ESP_OK) {
		return 0;
	}
	if (previous.magic != TELEM_MAGIC || previous.run_number == UINT32_MAX) {
		return 0;
	}
	return previous.run_number;
}

bool telemetry_begin_run(float kp, float ki, float kd, float integral_limit,
	int16_t drive_speed, int16_t turn_speed, uint16_t target_distance_mm)
{
	if (partition == NULL || buffer == NULL) {
		return false;
	}

	run_number = read_previous_run_number() + 1;
	sample_count = 0;
	dropped_count = 0;

	/* Erase only what this run can actually fill. Erasing the whole 2.4 MB
	 * partition would take several seconds and overrun the start countdown. */
	size_t used = TELEM_RECORDS_OFFSET + (size_t)TELEM_CAPACITY * sizeof(telem_record_t);
	size_t erase_size = (used + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE * FLASH_SECTOR_SIZE;

	int64_t erase_start = esp_timer_get_time();
	if (esp_partition_erase_range(partition, 0, erase_size) != ESP_OK) {
		ESP_LOGE(TAG, "erase failed");
		return false;
	}

	telem_header_t header = {
		.magic = TELEM_MAGIC,
		.version = TELEM_VERSION,
		.record_size = sizeof(telem_record_t),
		.run_number = run_number,
		.reserved0 = 0,
		.kp = kp,
		.ki = ki,
		.kd = kd,
		.integral_limit = integral_limit,
		.sample_rate_hz = IMU_SAMPLE_RATE_HZ,
		.drive_speed = drive_speed,
		.turn_speed = turn_speed,
		.target_distance_mm = target_distance_mm,
		.reserved1 = 0
	};
	snprintf(header.build, sizeof(header.build), "%s", __DATE__);

	if (esp_partition_write(partition, TELEM_HEADER_OFFSET, &header, sizeof(header)) != ESP_OK) {
		ESP_LOGE(TAG, "header write failed");
		return false;
	}

	run_start_us = esp_timer_get_time();
	ESP_LOGI(TAG, "run %" PRIu32 " armed (erase %" PRId64 " ms) kp=%.3f ki=%.3f kd=%.3f",
		run_number, (run_start_us - erase_start) / 1000, kp, ki, kd);
	return true;
}

void telemetry_log(const telem_record_t *record)
{
	if (buffer == NULL) {
		return;
	}
	if (sample_count >= TELEM_CAPACITY) {
		dropped_count++;
		return;
	}
	buffer[sample_count++] = *record;
}

bool telemetry_flush(void)
{
	if (partition == NULL || buffer == NULL) {
		return false;
	}
	if (sample_count == 0) {
		ESP_LOGW(TAG, "nothing to flush");
		return false;
	}

	size_t bytes = (size_t)sample_count * sizeof(telem_record_t);
	int64_t flush_start = esp_timer_get_time();

	if (esp_partition_write(partition, TELEM_RECORDS_OFFSET, buffer, bytes) != ESP_OK) {
		ESP_LOGE(TAG, "record write failed");
		return false;
	}

	telem_footer_t footer = {
		.magic = TELEM_FOOTER_MAGIC,
		.sample_count = sample_count,
		.dropped_count = dropped_count,
		.duration_ms = (uint32_t)((flush_start - run_start_us) / 1000)
	};
	memset(footer.reserved, 0, sizeof(footer.reserved));

	if (esp_partition_write(partition, TELEM_FOOTER_OFFSET, &footer, sizeof(footer)) != ESP_OK) {
		ESP_LOGE(TAG, "footer write failed");
		return false;
	}

	ESP_LOGI(TAG, "run %" PRIu32 " flushed: %" PRIu32 " samples (%u B) in %" PRId64 " ms, %" PRIu32 " dropped",
		run_number, sample_count, (unsigned)bytes,
		(esp_timer_get_time() - flush_start) / 1000, dropped_count);
	ESP_LOGI(TAG, "read it with: python tools/telemetry.py --port <PORT> --plot");
	return true;
}

uint32_t telemetry_sample_count(void)
{
	return sample_count;
}

uint32_t telemetry_run_number(void)
{
	return run_number;
}

bool telemetry_is_full(void)
{
	return sample_count >= TELEM_CAPACITY;
}
