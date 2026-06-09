#ifndef PID_H
#define PID_H

#include <stdint.h>

#define EXPECTED_DT_MS  5.0f
#define DT_MIN_MS       (EXPECTED_DT_MS * 0.8f)
#define DT_MAX_MS       (EXPECTED_DT_MS * 1.2f)

typedef struct {
    float setpoint;
    float prev_feedback;
    float integral;
    float kp, ki, kd;
    float output_min;
    float output_max;
    uint32_t prev_time_ms;
    uint8_t integral_clamped;
} PID_Controller;

void pid_init(PID_Controller* pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

float pid_update(PID_Controller* pid, float feedback, uint32_t now_ms);

#endif

