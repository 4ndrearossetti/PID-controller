#ifndef QUAD_H
#define QUAD_H

#include <stdint.h>

/* --- Physical constants --- */
#define GRAVITY         9.81f                           // m/s²
#define MASS            0.25f                           // kg
#define ARM_LENGTH      0.2f                            // meters (center to motor)
#define L_EFF           (ARM_LENGTH * 0.70710678f)      // arm * sin(45°)
#define Ixx             0.01f                           // kg·m²
#define Iyy             0.01f
#define Izz             0.02f

/* --- State --- */

/*
  Coordinate system: NED (North, East, Down)

    +x   forward (North)
    +y   left (East)
    +z   down

    roll    + = right roll (right side down)
    pitch   + = nose up
    yaw     + = CW from above

    p   + = right roll rate
    q   + = nose-up pitch rate
    r   + = CW yaw rate

  Hover: thrust balances gravity; altitude (up) is -z.
  Ground constraint: z > 0 is below ground.

  The sign chain through the entire control pipeline
  (PID → mixer → quad dynamics) must be consistent with
  this frame. If you change one link, you change them all.
*/
typedef struct {
        float x, y, z;              // position (m), NED
        float vx, vy, vz;           // velocity (m/s), NED
        float roll, pitch, yaw;     // attitude (rad)
        float p, q, r;              // body rates (rad/s)
} QuadState;

/* --- Dynamics --- */
/*
  Advance the state by dt seconds, given the four motor thrusts
  in Newtons.

  Motor index convention (X-frame, viewed from above):
    motor_thrusts_N[0]  M1  front-right  (CCW)
    motor_thrusts_N[1]  M2  rear-left    (CCW)
    motor_thrusts_N[2]  M3  front-left   (CW)
    motor_thrusts_N[3]  M4  rear-right   (CW)
*/
void quad_step(QuadState* s, const float motor_thrusts_N[4], float dt);

#endif

