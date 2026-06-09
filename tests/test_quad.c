#include "../src/quad.h"
#include "test_helpers.h"

static void test_hover(void) {
    QuadState s = {0};
    float F_hover_each = (MASS * GRAVITY) / 4.0f;
    float F[4] = {F_hover_each, F_hover_each, F_hover_each, F_hover_each};

    for (int i = 0; i < 1000; i++) quad_step(&s, F, 0.005f);

    ASSERT_NEAR(s.z,  0.0f, 1e-3f, "hover: z should stay near zero");
    ASSERT_NEAR(s.vz, 0.0f, 1e-3f, "hover: vz should be zero");
    ASSERT_NEAR(s.x,  0.0f, 1e-3f, "hover: no lateral drift");
    ASSERT_NEAR(s.roll,  0.0f, 1e-6f, "hover: no roll");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-6f, "hover: no pitch");
    TEST_PASS("test_hover");
}

static void test_pure_roll_torque(void) {
    // M2, M3 stronger → should produce positive roll torque, body rolls.
    QuadState s = {0};
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    float F[4] = { F_h - bias, F_h + bias, F_h + bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.roll  > 0.0f, "pure roll: roll angle should be positive");
    ASSERT_TRUE(s.p     > 0.0f, "pure roll: body rate p should be positive");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-4f, "pure roll: no pitch motion");
    ASSERT_NEAR(s.yaw,   0.0f, 1e-4f, "pure roll: no yaw motion");
    TEST_PASS("test_pure_roll_torque");
}

static void test_pure_pitch_torque(void) {
    QuadState s = {0};
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    // M1, M3 (front) stronger → positive pitch torque
    float F[4] = { F_h + bias, F_h - bias, F_h + bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.pitch > 0.0f, "pure pitch: pitch angle positive");
    ASSERT_TRUE(s.q     > 0.0f, "pure pitch: q positive");
    ASSERT_NEAR(s.roll, 0.0f, 1e-4f, "pure pitch: no roll");
    ASSERT_NEAR(s.yaw,  0.0f, 1e-4f, "pure pitch: no yaw");
    TEST_PASS("test_pure_pitch_torque");
}

static void test_pure_yaw_torque(void) {
    QuadState s = {0};
    float F_h = (MASS * GRAVITY) / 4.0f;
    float bias = 0.02f;
    // M1, M2 (CCW) stronger → positive yaw torque
    float F[4] = { F_h + bias, F_h + bias, F_h - bias, F_h - bias };

    for (int i = 0; i < 200; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.yaw > 0.0f, "pure yaw: yaw angle positive");
    ASSERT_TRUE(s.r   > 0.0f, "pure yaw: r positive");
    ASSERT_NEAR(s.roll,  0.0f, 1e-4f, "pure yaw: no roll");
    ASSERT_NEAR(s.pitch, 0.0f, 1e-4f, "pure yaw: no pitch");
    TEST_PASS("test_pure_yaw_torque");
}

static void test_pitch_produces_forward_accel(void) {
    // Set initial pitch directly, hover thrust, watch lateral motion.
    QuadState s = {0};
    s.pitch = 0.1f;
    float F_h = (MASS * GRAVITY) / 4.0f;
    float F[4] = {F_h, F_h, F_h, F_h};

    // Run a few steps; tilted thrust should produce +x acceleration.
    for (int i = 0; i < 50; i++) quad_step(&s, F, 0.005f);

    ASSERT_TRUE(s.vx > 0.0f, "tilted pitch should produce forward velocity");
    TEST_PASS("test_pitch_produces_forward_accel");
}

int main(void) {
    test_hover();
    test_pure_roll_torque();
    test_pure_pitch_torque();
    test_pure_yaw_torque();
    test_pitch_produces_forward_accel();
    printf("All quad tests passed.\n");
    return 0;
}

