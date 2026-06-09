#include "../src/quad.h"
#include "../src/mixer.h"
#include "test_helpers.h"

/* All tests start above ground (z < 0 in NED) so the ground constraint
 * doesn't mask physics bugs. */
#define ALT_INIT  -10.0f

/* ── Hover ─────────────────────────────────────────────────────── */
static void test_hover(void) {
    QuadState s = {0};
    s.z = -1.0f;
    float F_h = (MASS * GRAVITY) / 4.0f;
    float F[4] = { F_h, F_h, F_h, F_h };

    for (int i = 0; i < 1000; i++) quad_step(&s, F, 0.005f);

    ASSERT_NEAR(s.z,     -1.0f, 1e-3f, "hover: z stays at -1m");
    ASSERT_NEAR(s.vz,     0.0f, 1e-3f, "hover: vz zero");
    ASSERT_NEAR(s.x,      0.0f, 1e-3f, "hover: no x drift");
    ASSERT_NEAR(s.y,      0.0f, 1e-3f, "hover: no y drift");
    ASSERT_NEAR(s.roll,   0.0f, 1e-6f, "hover: no roll");
    ASSERT_NEAR(s.pitch,  0.0f, 1e-6f, "hover: no pitch");
    TEST_PASS("test_hover");
}

/* ── Pure-axis torque tests (hand-crafted motor thrusts) ──────── */
static void test_pure_roll_torque(void) {
    /* M2, M3 stronger → positive τ_x → roll right */
    QuadState s = {0};
    s.z = ALT_INIT;
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    float F[4] = { F_h - bias, F_h + bias, F_h + bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.roll  > 0.0f, "pure roll: roll angle positive");
    ASSERT_TRUE(s.p     > 0.0f, "pure roll: body rate p positive");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-4f, "pure roll: no pitch coupling");
    ASSERT_NEAR(s.yaw,   0.0f, 1e-4f, "pure roll: no yaw coupling");
    TEST_PASS("test_pure_roll_torque");
}

static void test_pure_pitch_torque(void) {
    /* M1, M3 (front) stronger → positive τ_y → nose up in NED */
    QuadState s = {0};
    s.z = ALT_INIT;
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    float F[4] = { F_h + bias, F_h - bias, F_h + bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.pitch > 0.0f, "pure pitch: pitch angle positive");
    ASSERT_TRUE(s.q     > 0.0f, "pure pitch: body rate q positive");
    ASSERT_NEAR(s.roll, 0.0f, 1e-4f, "pure pitch: no roll coupling");
    ASSERT_NEAR(s.yaw,  0.0f, 1e-4f, "pure pitch: no yaw coupling");
    TEST_PASS("test_pure_pitch_torque");
}

static void test_pure_yaw_torque(void) {
    /* M1, M2 (CCW) stronger → positive τ_z → yaw right in NED */
    QuadState s = {0};
    s.z = ALT_INIT;
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    float F[4] = { F_h + bias, F_h + bias, F_h - bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.yaw > 0.0f, "pure yaw: yaw angle positive");
    ASSERT_TRUE(s.r   > 0.0f, "pure yaw: body rate r positive");
    ASSERT_NEAR(s.roll,  0.0f, 1e-4f, "pure yaw: no roll coupling");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-4f, "pure yaw: no pitch coupling");
    TEST_PASS("test_pure_yaw_torque");
}

/* Tilted thrust produces lateral motion.
 * In NED: nose-down (negative pitch) → forward (+x) acceleration. */
static void test_pitch_produces_forward_accel(void) {
    QuadState s = {0};
    s.z     = ALT_INIT;
    s.pitch = -0.1f;                                /* nose down */
    float F_h = (MASS * GRAVITY) / 4.0f;
    float F[4] = { F_h, F_h, F_h, F_h };

    for (int i = 0; i < 50; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.vx > 0.0f, "nose-down pitch produces +x velocity");
    TEST_PASS("test_pitch_produces_forward_accel");
}

/* ── Integration: mixer → quad ─────────────────────────────────
 * These drive the physics through the mixer with abstract commands,
 * verifying mixer and quad agree on sign conventions end-to-end. */

static void test_mixer_quad_roll(void) {
    QuadState s = {0};
    s.z = ALT_INIT;
    float F[4];
    float hover = MASS * GRAVITY;

    for (int i = 0; i < 200; i++) {
        mixer_compute(hover, 0.1f, 0.0f, 0.0f, F);   /* +roll cmd */
        quad_step(&s, F, 0.005f);
    }

    ASSERT_TRUE(s.roll  > 0.0f, "mixer→quad: +roll cmd produces +roll angle");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-2f, "mixer→quad: +roll cmd no pitch coupling");
    ASSERT_NEAR(s.yaw,   0.0f, 1e-2f, "mixer→quad: +roll cmd no yaw coupling");
    TEST_PASS("test_mixer_quad_roll");
}

static void test_mixer_quad_pitch(void) {
    QuadState s = {0};
    s.z = ALT_INIT;
    float F[4];
    float hover = MASS * GRAVITY;

    for (int i = 0; i < 200; i++) {
        mixer_compute(hover, 0.0f, 0.1f, 0.0f, F);   /* +pitch cmd */
        quad_step(&s, F, 0.005f);
    }

    ASSERT_TRUE(s.pitch > 0.0f, "mixer→quad: +pitch cmd produces +pitch angle");
    ASSERT_NEAR(s.roll, 0.0f, 1e-2f, "mixer→quad: +pitch cmd no roll coupling");
    ASSERT_NEAR(s.yaw,  0.0f, 1e-2f, "mixer→quad: +pitch cmd no yaw coupling");
    TEST_PASS("test_mixer_quad_pitch");
}

static void test_mixer_quad_yaw(void) {
    QuadState s = {0};
    s.z = ALT_INIT;
    float F[4];
    float hover = MASS * GRAVITY;

    for (int i = 0; i < 400; i++) {                 /* yaw is slow */
        mixer_compute(hover, 0.0f, 0.0f, 0.05f, F); /* +yaw cmd */
        quad_step(&s, F, 0.005f);
    }

    ASSERT_TRUE(s.yaw > 0.0f, "mixer→quad: +yaw cmd produces +yaw angle");
    ASSERT_NEAR(s.roll,  0.0f, 1e-2f, "mixer→quad: +yaw cmd no roll coupling");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-2f, "mixer→quad: +yaw cmd no pitch coupling");
    TEST_PASS("test_mixer_quad_yaw");
}

/* ── Driver ────────────────────────────────────────────────────── */
int main(void) {
    test_hover();
    test_pure_roll_torque();
    test_pure_pitch_torque();
    test_pure_yaw_torque();
    test_pitch_produces_forward_accel();

    test_mixer_quad_roll();
    test_mixer_quad_pitch();
    test_mixer_quad_yaw();

    printf("All quad tests passed.\n");
    return 0;
}

