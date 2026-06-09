#include "pid.h"

void pid_init(PID_Controller* pid,
              float kp, float ki, float kd,
              float out_min, float out_max) {
    pid->setpoint   = 0.0f;
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid_reset(pid);
}

void pid_reset(PID_Controller* pid) {
    pid->prev_feedback    = 0.0f;
    pid->integral         = 0.0f;
    pid->has_prev         = 0;
    pid->integral_clamped = 0;
}

float pid_update(PID_Controller* pid, float feedback, float dt) {
    /* First call: no history, so no meaningful derivative or integration.
      Seed prev_feedback so the next call sees a real derivative.
      Return a clamped P-only output rather than zero, otherwise the
      actuator pulses to zero on engage. */
    if (!pid->has_prev) {
        pid->prev_feedback = feedback;
        pid->has_prev      = 1;

        float p_only = pid->kp * (pid->setpoint - feedback);
        if (p_only > pid->output_max) p_only = pid->output_max;
        if (p_only < pid->output_min) p_only = pid->output_min;
        return p_only;
    }

    float error = pid->setpoint - feedback;

    // Tentative integral update
    float integral_new = pid->integral + error * dt;

    // Derivative on measurement
    float derivative = (feedback - pid->prev_feedback) / dt;

    float output = pid->kp * error
                 + pid->ki * integral_new
                 - pid->kd * derivative;

    pid->integral_clamped = 0;

    /* Anti-windup: if output saturates AND the error is in the direction
      that would push it further into saturation, discard the new
      integral. Recovery (error reversing sign) is still allowed, so
      the integrator can drain once the system swings back. */
    if (output > pid->output_max) {
        output = pid->output_max;
        if (error > 0.0f) {
            integral_new = pid->integral;
            pid->integral_clamped = 1;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        if (error < 0.0f) {
            integral_new = pid->integral;
            pid->integral_clamped = 1;
        }
    }

    pid->integral      = integral_new;
    pid->prev_feedback = feedback;

    return output;
}

