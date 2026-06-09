#ifndef MIXER_H
#define MIXER_H

#define F_MOTOR_MAX 2.5f  // N, per motor, 4:1 TWR airframe

/*
  Convert abstract control commands into per-motor thrusts.

  Inputs (all in Newtons):
    thrust_cmd  total thrust, must be >= 0; hover ≈ MASS * GRAVITY
    roll_cmd    + = left motors stronger  → positive roll torque
    pitch_cmd   + = front motors stronger → positive pitch torque
    yaw_cmd     + = CCW motors stronger   → positive yaw torque

  Output:
    motor_thrusts_N[4]   { M1=FR, M2=RL, M3=FL, M4=RR } in Newtons,
                         clamped to [0, F_MOTOR_MAX].
*/
void mixer_compute(float thrust_cmd, float roll_cmd,
                   float pitch_cmd, float yaw_cmd,
                   float motor_thrusts_N[4]);

#endif

