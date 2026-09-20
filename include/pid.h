#ifndef PID_H
#define PID_H

#include <stdbool.h>

/* Heading controller.
 *
 * Two deliberate departures from the textbook form:
 *
 * 1. The derivative term uses the *measured rate* straight from the gyro rather
 *    than differentiating the heading. Differentiating a noisy signal amplifies
 *    the noise; the LSM6DS3 already measures the rate directly, so there is no
 *    reason to reconstruct it badly. This also removes derivative kick, because
 *    a step change in setpoint no longer appears in the D term at all.
 *
 * 2. Integration is conditional: the integrator is frozen whenever the output is
 *    already saturated and the error would push it further into the rail. This
 *    is cheaper and better behaved than clamping after the fact.
 */
typedef struct {
	float kp;
	float ki;
	float kd;
	float integral_limit;  /* absolute cap on the integral term's contribution */
	float output_limit;    /* absolute cap on the total output */
	float integral;
	/* Populated on every update so the telemetry log can record each term's
	 * contribution separately - when the robot oscillates you need to see which
	 * term caused it, not just that the output was wrong. */
	float p_term;
	float i_term;
	float d_term;
	float error;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd,
	float integral_limit, float output_limit);

/* Zeroes the integrator and the recorded terms. Call between runs, and whenever
 * the controller is re-engaged after being idle, so stale integral does not
 * kick the robot on the first sample. */
void pid_reset(pid_t *pid);

/* `error` must already be wrapped to [-180, +180) by the caller - see
 * imu_wrap180() - so the controller always turns the short way around.
 * `measured_rate` is the gyro's yaw rate in deg/s. */
float pid_update(pid_t *pid, float error, float measured_rate, float dt_s);

#endif
