#include "mixer.h"

static float clampf(float x, float lo, float hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
}

void mixer_compute(float thrust_cmd, float roll_cmd,
                   float pitch_cmd, float yaw_cmd, float F[4]) {
        // Each motor: thrust/4 ± roll ± pitch ± yaw.
        // Signs determined by motor position and spin direction.
        float t = thrust_cmd * 0.25f;

        F[0] = t - roll_cmd + pitch_cmd + yaw_cmd;  // M1 FR CCW
        F[1] = t + roll_cmd - pitch_cmd + yaw_cmd;  // M2 RL CCW
        F[2] = t + roll_cmd + pitch_cmd - yaw_cmd;  // M3 FL CW
        F[3] = t - roll_cmd - pitch_cmd - yaw_cmd;  // M4 RR CW

        for (int i = 0; i < 4; i++) {
                F[i] = clampf(F[i], 0.0f, F_MOTOR_MAX);
        }
}

