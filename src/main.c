#include <stdio.h>
#include <unistd.h>
#include "quad.h"
#include "mixer.h"
#include "control.h"
#include "pid.h"

/* ────────────────────────────────────────────────────────────────── */
/*  Scenario selection                                                */
/*  Set exactly one to 1.                                             */
/* ────────────────────────────────────────────────────────────────── */
#define SCENARIO_ROLL_RATE_STEP  0
#define SCENARIO_ROLL_ANGLE_STEP 0
#define SCENARIO_PITCH_STEP      0
#define SCENARIO_YAW_RATE_STEP   0
#define SCENARIO_ALTITUDE_STEP   1
#define SCENARIO_FULL_FLIGHT     0

/* ────────────────────────────────────────────────────────────────── */
/*  Gains — edit these to tune.                                       */
/*  Inner (rate) loops first, then outer (angle), then altitude/yaw.  */
/* ────────────────────────────────────────────────────────────────── */

/* Altitude (single loop): meters → Newtons */
#define KP_ALT          0.0f
#define KI_ALT          0.0f
#define KD_ALT          0.0f

/* Roll angle outer (cascade): rad → rad/s */
#define KP_ROLL_ANG     0.0f
#define KI_ROLL_ANG     0.0f
#define KD_ROLL_ANG     0.0f

/* Roll rate inner (cascade): rad/s → N (differential) */
#define KP_ROLL_RATE    0.0f
#define KI_ROLL_RATE    0.0f
#define KD_ROLL_RATE    0.0f

/* Pitch angle outer: rad → rad/s */
#define KP_PITCH_ANG    0.0f
#define KI_PITCH_ANG    0.0f
#define KD_PITCH_ANG    0.0f

/* Pitch rate inner: rad/s → N (differential) */
#define KP_PITCH_RATE   0.0f
#define KI_PITCH_RATE   0.0f
#define KD_PITCH_RATE   0.0f

/* Yaw rate (single loop): rad/s → N (differential) */
#define KP_YAW_RATE     0.0f
#define KI_YAW_RATE     0.0f
#define KD_YAW_RATE     0.0f

/* ────────────────────────────────────────────────────────────────── */
/*  Output limits — set once from physics, don't tune.                */
/* ────────────────────────────────────────────────────────────────── */
#define ALT_OUT_MIN     0.0f
#define ALT_OUT_MAX     10.0f       /* 4 * F_MOTOR_MAX */

#define ANG_RATE_MAX    3.0f        /* rad/s, what outer asks of inner */

#define TORQUE_CMD_MAX  1.5f        /* N, roll/pitch differential limit */
#define YAW_CMD_MAX     2.0f        /* N, yaw differential limit */

/* ────────────────────────────────────────────────────────────────── */
/*  Simulation parameters                                             */
/* ────────────────────────────────────────────────────────────────── */
#define DT              0.005f       /* 200 Hz */
#define SIM_DURATION_S  10.0f
#define TOTAL_STEPS     ((int)(SIM_DURATION_S / DT))

/* Print to terminal every N steps (20 Hz at 200 Hz sim) */
#define PRINT_DECIMATE  10

/* ────────────────────────────────────────────────────────────────── */
/*  Scenarios — return desired setpoints for the current time.        */
/* ────────────────────────────────────────────────────────────────── */
static Setpoints get_setpoints(float t) {
    Setpoints sp = {0};

#if SCENARIO_ROLL_RATE_STEP
    /* Tuning the inner roll rate PID in isolation.
     * Bypass the outer cascade by writing roll_rate setpoint directly
     * via a hack: we'll patch this in main by overwriting the inner
     * setpoint after control_update. For now, the schedule is null —
     * see the main loop for the rate-injection logic. */
    (void)t;
#endif

#if SCENARIO_ROLL_ANGLE_STEP
    if (t > 1.0f) sp.altitude = 1.0f;
    if (t > 3.0f) sp.roll     = 0.2f;    /* step roll to ~11° */
    if (t > 5.0f) sp.roll     = -0.2f;
    if (t > 7.0f) sp.roll     = 0.0f;
#endif

#if SCENARIO_PITCH_STEP
    if (t > 1.0f) sp.altitude = 1.0f;
    if (t > 3.0f) sp.pitch    = 0.2f;
    if (t > 5.0f) sp.pitch    = -0.2f;
    if (t > 7.0f) sp.pitch    = 0.0f;
#endif

#if SCENARIO_YAW_RATE_STEP
    if (t > 1.0f) sp.altitude = 1.0f;
    if (t > 3.0f) sp.yaw_rate = 1.0f;    /* yaw right at 1 rad/s */
    if (t > 5.0f) sp.yaw_rate = -1.0f;
    if (t > 7.0f) sp.yaw_rate = 0.0f;
#endif

#if SCENARIO_ALTITUDE_STEP
    if (t > 1.0f) sp.altitude = 1.0f;
    if (t > 5.0f) sp.altitude = 2.0f;
    if (t > 8.0f) sp.altitude = 0.5f;
#endif

#if SCENARIO_FULL_FLIGHT
    if (t > 1.0f) sp.altitude = 1.0f;
    if (t > 3.0f) sp.roll     = 0.15f;
    if (t > 4.5f) sp.pitch    = 0.15f;
    if (t > 6.0f) sp.yaw_rate = 0.5f;
    if (t > 8.0f) { sp.roll = 0.0f; sp.pitch = 0.0f; sp.yaw_rate = 0.0f; }
#endif

    return sp;
}

/* ────────────────────────────────────────────────────────────────── */
/*  main                                                              */
/* ────────────────────────────────────────────────────────────────── */
int main(void) {
    QuadState s = {0};

    Controllers ctrl;
    pid_init(&ctrl.altitude,    KP_ALT,        KI_ALT,        KD_ALT,        ALT_OUT_MIN,     ALT_OUT_MAX);
    pid_init(&ctrl.roll_angle,  KP_ROLL_ANG,   KI_ROLL_ANG,   KD_ROLL_ANG,   -ANG_RATE_MAX,   ANG_RATE_MAX);
    pid_init(&ctrl.roll_rate,   KP_ROLL_RATE,  KI_ROLL_RATE,  KD_ROLL_RATE,  -TORQUE_CMD_MAX, TORQUE_CMD_MAX);
    pid_init(&ctrl.pitch_angle, KP_PITCH_ANG,  KI_PITCH_ANG,  KD_PITCH_ANG,  -ANG_RATE_MAX,   ANG_RATE_MAX);
    pid_init(&ctrl.pitch_rate,  KP_PITCH_RATE, KI_PITCH_RATE, KD_PITCH_RATE, -TORQUE_CMD_MAX, TORQUE_CMD_MAX);
    pid_init(&ctrl.yaw_rate,    KP_YAW_RATE,   KI_YAW_RATE,   KD_YAW_RATE,   -YAW_CMD_MAX,    YAW_CMD_MAX);

    FILE* log = fopen("flight_log.csv", "w");
    if (!log) {
        fprintf(stderr, "Error: could not open flight_log.csv for writing\n");
        return 1;
    }
    fprintf(log, "t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,"
                 "sp_alt,sp_roll,sp_pitch,sp_yaw_rate,"
                 "thrust_cmd,roll_cmd,pitch_cmd,yaw_cmd,"
                 "m1,m2,m3,m4\n");

    /* Terminal header. Aligned columns; altitude shown as -z (positive up). */
    printf("    t   |  alt  vz   |  roll  pitch  yaw  |   p     q     r    "
           "|  thr   roll_c  pitch_c yaw_c |  m1   m2   m3   m4\n");
    printf("--------+------------+--------------------+-------------------"
           "-+-------------------------------+----------------------\n");

    for (int step = 0; step < TOTAL_STEPS; step++) {
        float t = step * DT;

        Setpoints sp = get_setpoints(t);

#if SCENARIO_ROLL_RATE_STEP
        /* Bypass the outer cascade: inject the rate setpoint directly
         * into the inner PID. Outer (roll_angle) stays inert because
         * its gains are zero, but to make absolutely sure its output
         * doesn't leak into roll_rate.setpoint, we overwrite after. */
        float rate_sp = 0.0f;
        if (t > 1.0f) rate_sp = 2.0f;
        if (t > 3.0f) rate_sp = -2.0f;
        if (t > 5.0f) rate_sp = 0.0f;
        sp.altitude = 1.0f;  /* keep some thrust so it lifts off */
#endif

        ControlOutput cmd = control_update(&ctrl, &s, &sp, DT);

#if SCENARIO_ROLL_RATE_STEP
        /* Now override: re-run only the inner rate PID with our manual
         * setpoint and re-compute roll_cmd. Surgical, ugly, temporary. */
        ctrl.roll_rate.setpoint = rate_sp;
        cmd.roll_cmd = pid_update(&ctrl.roll_rate, s.p, DT);
#endif

        float F[4];
        mixer_compute(cmd.thrust_cmd, cmd.roll_cmd, cmd.pitch_cmd, cmd.yaw_cmd, F);

        quad_step(&s, F, DT);

        /* CSV — every tick, 24 columns. */
        fprintf(log,
                "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f\n",
                t, s.x, s.y, s.z, s.vx, s.vy, s.vz,
                s.roll, s.pitch, s.yaw, s.p, s.q, s.r,
                sp.altitude, sp.roll, sp.pitch, sp.yaw_rate,
                cmd.thrust_cmd, cmd.roll_cmd, cmd.pitch_cmd, cmd.yaw_cmd,
                F[0], F[1], F[2], F[3]);
        fflush(log);

        /* Terminal — decimated, key columns only. */
        if (step % PRINT_DECIMATE == 0) {
            printf("%6.2f  | %5.2f %5.2f | %5.2f %5.2f %5.2f | %5.2f %5.2f %5.2f "
                   "| %5.2f %6.2f %6.2f %5.2f | %4.2f %4.2f %4.2f %4.2f\n",
                   t, -s.z, -s.vz,
                   s.roll, s.pitch, s.yaw,
                   s.p, s.q, s.r,
                   cmd.thrust_cmd, cmd.roll_cmd, cmd.pitch_cmd, cmd.yaw_cmd,
                   F[0], F[1], F[2], F[3]);
        }

        usleep(5000);
    }

    fclose(log);
    printf("\nSim complete. Log: flight_log.csv\n");
    return 0;
}

