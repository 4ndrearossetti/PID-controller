#ifndef PID_H
#define PID_H

/*
  Generic single-axis PID controller with anti-windup.

  The controller is unit-agnostic: feedback and setpoint can be in any
  physical units. The output is in whatever units the gains map to.

  Derivative is computed on the measurement, not the error, to avoid
  derivative kick when the setpoint changes abruptly — important in
  cascaded setups where the outer loop's output (= inner setpoint)
  can step from tick to tick.

  Anti-windup uses conditional integration: when the output saturates,
  the integral is frozen in the direction that would worsen
  saturation. Updates that would reduce saturation are still allowed,
  so the controller can climb back out once the error reverses.

  Caller passes dt explicitly. The controller does not track time; this
  keeps it usable at any loop rate (e.g. a cascade with a fast inner
  loop and a slow outer loop) without baking timing assumptions in.
*/

typedef struct {
    // Inputs (set by caller)
    float setpoint;          // desired value
    float kp, ki, kd;        // gains
    float output_min;        // saturation lower bound
    float output_max;        // saturation upper bound

    // Internal state
    float prev_feedback;     // feedback from previous call
    float integral;          // accumulated error · dt
    int   has_prev;          // false until the first call completes
    int   integral_clamped;  // diagnostic: was integral frozen?
} PID_Controller;

/*
  Initialize a controller. Setpoint defaults to zero; assign
  pid->setpoint directly after init if you want a different value.
  No validation on gains or limits — caller's responsibility.
*/
void pid_init(PID_Controller* pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

/*
  Advance the controller one step.

    feedback : current measurement (same units as setpoint)
    dt       : elapsed time since previous call, in seconds; must be > 0

  Returns the control output, clamped to [output_min, output_max].

  On the first call (or first after pid_reset), the derivative term is
  suppressed by seeding prev_feedback = feedback. The output is a
  clamped P-only value, so the actuator doesn't pulse to zero before
  the controller engages.
*/
float pid_update(PID_Controller* pid, float feedback, float dt);

/*
  Clear integral and previous-feedback history. Use when re-engaging
  a controller after disabling it (e.g. switching flight modes), so the
  integrator doesn't carry stale state.
*/
void pid_reset(PID_Controller* pid);

#endif

