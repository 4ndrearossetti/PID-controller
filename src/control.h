#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"
#include "quad.h"

/*
  Setpoints — what the user (or test schedule) wants the drone to do.

  Altitude is in meters, positive up — the convention humans use.
  The controller internally converts to/from NED z when reading state.
  Roll and pitch are angles in radians. Yaw is a rate in rad/s
 */
typedef struct {
    float altitude;
    float roll;
    float pitch;
    float yaw_rate;
} Setpoints;

/*
  Control outputs — exactly what mixer_compute expects.

  All four in Newtons. thrust_cmd is the total collective thrust the
  mixer should produce (split four ways across motors). The other
  three are differential thrust commands that the mixer maps to
  per-motor offsets.
 */
typedef struct {
    float thrust_cmd;
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
} ControlOutput;

/*
  Cascade structure:
    altitude PID                        --> thrust_cmd
    roll_angle PID  --> roll_rate PID   --> roll_cmd
    pitch_angle PID --> pitch_rate PID  --> pitch_cmd
                        yaw_rate PID    --> yaw_cmd

  The Controllers struct holds the six PIDs but does not own their
  gains. Callers initialize each PID directly via pid_init() with the
  gains and limits appropriate for their airframe and tuning. This
  lives in main.c (or eventually a config file) — control.c knows
  how to wire the cascade, not what numbers to wire it with.
*/
typedef struct {
    PID_Controller altitude;
    PID_Controller roll_angle;
    PID_Controller roll_rate;
    PID_Controller pitch_angle;
    PID_Controller pitch_rate;
    PID_Controller yaw_rate;
} Controllers;

/*
  One control tick. Reads state and setpoints; produces commands for
  the mixer. Does not modify state. dt is in seconds; must be > 0.
 */
ControlOutput control_update(Controllers* c, const QuadState* s,
                             const Setpoints* sp, float dt);

/*
  Reset all internal PID state (integrators, history). Useful when
  re-engaging the controller after disabling it.
 */
void control_reset(Controllers* c);

#endif

