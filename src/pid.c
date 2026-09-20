#include "pid.h"

static float clamp(float value, float limit)
{
	if (value > limit) {
		return limit;
	}
	if (value < -limit) {
		return -limit;
	}
	return value;
}

void pid_init(pid_t *pid, float kp, float ki, float kd,
	float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = integral_limit;
	pid->output_limit = output_limit;
	pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
	pid->integral = 0.0f;
	pid->p_term = 0.0f;
	pid->i_term = 0.0f;
	pid->d_term = 0.0f;
	pid->error = 0.0f;
}

float pid_update(pid_t *pid, float error, float measured_rate, float dt_s)
{
	if (dt_s <= 0.0f) {
		return clamp(pid->p_term + pid->i_term + pid->d_term, pid->output_limit);
	}

	pid->error = error;
	pid->p_term = pid->kp * error;

	/* D from the measured rate, negated: the term opposes motion, so it damps
	 * rather than drives. */
	pid->d_term = -pid->kd * measured_rate;

	/* Provisional integration, then undo it if it only deepens a saturation. */
	float candidate_integral = pid->integral + error * dt_s;
	float candidate_i_term = clamp(pid->ki * candidate_integral, pid->integral_limit);
	float unsaturated = pid->p_term + candidate_i_term + pid->d_term;

	bool winding_up = (unsaturated > pid->output_limit && error > 0.0f) ||
		(unsaturated < -pid->output_limit && error < 0.0f);
	if (!winding_up) {
		pid->integral = candidate_integral;
		pid->i_term = candidate_i_term;
		unsaturated = pid->p_term + pid->i_term + pid->d_term;
	} else {
		unsaturated = pid->p_term + pid->i_term + pid->d_term;
	}

	return clamp(unsaturated, pid->output_limit);
}
