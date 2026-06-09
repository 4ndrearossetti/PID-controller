#include "control.h"

ControlOutput control_update(Controllers* c, const QuadState* s,
                             const Setpoints* sp, float dt) {
        ControlOutput out;

        /* Altitude — single loop.
         The PID works in altitude space (positive up); the drone state is
         in NED z (positive down). One negation here is the only place that
         conversion lives. */
        c->altitude.setpoint = sp->altitude;
        out.thrust_cmd = pid_update(&c->altitude, -s->z, dt);

        /* Roll cascade.
         Outer angle PID: roll_setpoint - roll_measured --> desired body rate p.
         Inner rate PID:  desired_p - p_measured        --> torque-equivalent cmd.
         The mixer consumes roll_cmd as differential thrust in Newtons. */
        c->roll_angle.setpoint = sp->roll;
        float desired_p = pid_update(&c->roll_angle, s->roll, dt);
        c->roll_rate.setpoint = desired_p;
        out.roll_cmd = pid_update(&c->roll_rate, s->p, dt);

        // Pitch cascade — identical shape.
        c->pitch_angle.setpoint = sp->pitch;
        float desired_q = pid_update(&c->pitch_angle, s->pitch, dt);
        c->pitch_rate.setpoint = desired_q;
        out.pitch_cmd = pid_update(&c->pitch_rate, s->q, dt);

        /* Yaw — rate only, no outer loop.
         Setpoint is desired body rate r directly, no angle conversion. */
        c->yaw_rate.setpoint = sp->yaw_rate;
        out.yaw_cmd = pid_update(&c->yaw_rate, s->r, dt);

        return out;
}

void control_reset(Controllers* c) {
        pid_reset(&c->altitude);
        pid_reset(&c->roll_angle);
        pid_reset(&c->roll_rate);
        pid_reset(&c->pitch_angle);
        pid_reset(&c->pitch_rate);
        pid_reset(&c->yaw_rate);
}

