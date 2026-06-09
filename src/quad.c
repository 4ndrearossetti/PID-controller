#include <math.h>
#include "quad.h"

#define YAW_TORQUE_RATIO 0.03f
// #define MAX_TILT         0.45f   // 0.45rad ~ 25°

void quad_step(QuadState* s, const float F[4], float dt) {
        // Total thrust + body-frame torques
        float F_total   = F[0] + F[1] + F[2] + F[3];
        float tau_roll  = L_EFF * (F[1] + F[2] - F[0] - F[3]);
        float tau_pitch = L_EFF * (F[0] + F[2] - F[1] - F[3]);
        float tau_yaw   = YAW_TORQUE_RATIO * (F[0] + F[1] - F[2] - F[3]);

        // Body-to-world rotation of thrust vector
        float sr = sinf(s->roll),       cr = cosf(s->roll);
        float sp = sinf(s->pitch),      cp = cosf(s->pitch);
        float sy = sinf(s->yaw),        cy = cosf(s->yaw);

        float Fx_world = -F_total * (cy*sp*cr + sy*sr);
        float Fy_world = -F_total * (sy*sp*cr - cy*sr);
        float Fz_world = -F_total * (cp*cr);

        // Translational dynamics
        float ax = Fx_world / MASS;
        float ay = Fy_world / MASS;
        float az = Fz_world / MASS + GRAVITY;

        s->vx += ax * dt;
        s->vy += ay * dt;
        s->vz += az * dt;

        s->x += s->vx * dt;
        s->y += s->vy * dt;
        s->z += s->vz * dt;

        // Rotational dynamics
        float p_dot = tau_roll  / Ixx;
        float q_dot = tau_pitch / Iyy;
        float r_dot = tau_yaw   / Izz;

        s->p += p_dot * dt;
        s->q += q_dot * dt;
        s->r += r_dot * dt;

        // Attitude integration (small angle naive form)
        s->roll  += s->p * dt;
        s->pitch += s->q * dt;
        s->yaw   += s->r * dt;

        // Constraints
        if (s->z > 0.0f) { s->z  = 0.0f; s->vz = 0.0f; }
//        if (s->roll  >  MAX_TILT) s->roll  =  MAX_TILT;
//        if (s->roll  < -MAX_TILT) s->roll  = -MAX_TILT;
//        if (s->pitch >  MAX_TILT) s->pitch =  MAX_TILT;
//        if (s->pitch < -MAX_TILT) s->pitch = -MAX_TILT;
}

