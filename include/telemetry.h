#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Run logger.
 *
 * Samples accumulate in a RAM buffer during the run and are written to the raw
 * `telemetry` flash partition once the motors have stopped. The split is not
 * optional: SPI flash writes disable the ESP32's instruction cache and stall any
 * task not running from IRAM, so writing mid-run would inject millisecond-scale
 * jitter into the very control loop being measured.
 *
 * There is no filesystem. The records are fixed-size and written once, so
 * LittleFS would add a dependency and buy nothing. tools/telemetry.py reads the
 * partition straight off the chip with esptool and parses it with a numpy dtype.
 *
 * Flash layout:
 *   0x00  telem_header_t   written at run start (gains, config, run number)
 *   0x40  telem_footer_t   written at flush     (sample count, duration)
 *   0x80  telem_record_t[] the samples
 *
 * Header and footer occupy separate erased regions, so writing the footer after
 * the records is a legal flash write. Writing the header up front means an
 * interrupted run is still recoverable: the Python side falls back to trimming
 * on erased (0xFF) records when the footer is absent.
 */

#define TELEM_MAGIC 0x4D4C4554u        /* "TELM" little-endian */
#define TELEM_FOOTER_MAGIC 0x444E4554u /* "TEND" little-endian */
#define TELEM_VERSION 1

/* 3000 records is ~29 s at 104 Hz for 96 KB of heap - ample for a step response,
 * and it keeps the erase short enough to finish inside the start countdown. */
#define TELEM_CAPACITY 3000

/* Fixed-point scalings. Mirrored exactly in tools/telemetry.py; change one and
 * you must change the other. Sized to 32 bytes because esp_partition_write()
 * requires 4-byte alignment. */
#define TELEM_ANGLE_SCALE 10.0f  /* deci-degrees: +/-3276.8 deg, 0.1 deg steps */
#define TELEM_RATE_SCALE 10.0f   /* deci-deg/s */
#define TELEM_ACCEL_SCALE 1000.0f /* milli-g */
#define TELEM_TERM_SCALE 10.0f   /* deci-output-units */

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t version;
	uint16_t record_size;
	uint32_t run_number;
	uint32_t reserved0;
	float kp;
	float ki;
	float kd;
	float integral_limit;
	float sample_rate_hz;
	int16_t drive_speed;
	int16_t turn_speed;
	uint16_t target_distance_mm;
	uint16_t reserved1;
	char build[20];
} telem_header_t; /* 64 bytes */

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint32_t sample_count;
	uint32_t dropped_count;
	uint32_t duration_ms;
	uint8_t reserved[48];
} telem_footer_t; /* 64 bytes */

typedef struct __attribute__((packed)) {
	uint32_t t_us;
	int16_t yaw_ddeg;       /* actual heading */
	int16_t setpoint_ddeg;  /* desired heading */
	int16_t yaw_rate_ddps;
	int16_t accel_fwd_mg;
	int16_t accel_lat_mg;
	int16_t p_term;
	int16_t i_term;
	int16_t d_term;
	int16_t motor_a;
	int16_t motor_b;
	uint16_t range_left_mm;
	uint16_t range_right_mm;
	uint8_t state;
	uint8_t flags;
	uint16_t reserved;
} telem_record_t; /* 32 bytes */

#define TELEM_FLAG_TARGET_VISIBLE 0x01
#define TELEM_FLAG_CONTACT 0x02

/* Locates the partition and allocates the RAM buffer. */
bool telemetry_init(void);

/* Reads the previous run's number, erases only the region this run can fill,
 * and writes the header. Call while the robot is stationary - the erase takes a
 * few hundred milliseconds. */
bool telemetry_begin_run(float kp, float ki, float kd, float integral_limit,
	int16_t drive_speed, int16_t turn_speed, uint16_t target_distance_mm);

/* Appends one sample. Silently counts a drop once the buffer is full rather than
 * overwriting: for tuning it is the start of the run that matters. */
void telemetry_log(const telem_record_t *record);

/* Writes the buffered records and the footer. Motors must already be stopped. */
bool telemetry_flush(void);

uint32_t telemetry_sample_count(void);
uint32_t telemetry_run_number(void);
bool telemetry_is_full(void);

#endif
