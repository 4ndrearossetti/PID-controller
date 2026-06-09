#include "../src/mixer.h"
#include "../src/quad.h"
#include "test_helpers.h"

// Hover: equal thrust everywhere, no differentials.
static void test_hover_mix(void) {
    float F[4];
    float hover = MASS * GRAVITY;
    mixer_compute(hover, 0.0f, 0.0f, 0.0f, F);

    float expected = hover * 0.25f;
    ASSERT_NEAR(F[0], expected, 1e-6f, "hover: M1 = throttle/4");
    ASSERT_NEAR(F[1], expected, 1e-6f, "hover: M2 = throttle/4");
    ASSERT_NEAR(F[2], expected, 1e-6f, "hover: M3 = throttle/4");
    ASSERT_NEAR(F[3], expected, 1e-6f, "hover: M4 = throttle/4");
    TEST_PASS("test_hover_mix");
}

// Roll +: left motors (M2, M3) up; right motors (M1, M4) down.
static void test_roll_mix(void) {
    float F[4];
    mixer_compute(MASS * GRAVITY, 0.1f, 0.0f, 0.0f, F);

    ASSERT_TRUE(F[1] > F[0], "roll+: M2 > M1");
    ASSERT_TRUE(F[2] > F[3], "roll+: M3 > M4");
    ASSERT_NEAR(F[0] + F[3], F[1] + F[2] - 0.4f, 1e-5f,
                "roll+: total differential equals 4 * roll_cmd");
    TEST_PASS("test_roll_mix");
}

// Pitch +: front motors (M1, M3) up; rear (M2, M4) down.
static void test_pitch_mix(void) {
    float F[4];
    mixer_compute(MASS * GRAVITY, 0.0f, 0.1f, 0.0f, F);

    ASSERT_TRUE(F[0] > F[1], "pitch+: M1 > M2");
    ASSERT_TRUE(F[2] > F[3], "pitch+: M3 > M4");
    TEST_PASS("test_pitch_mix");
}

// Yaw +: CCW pair (M1, M2) up; CW pair (M3, M4) down.
static void test_yaw_mix(void) {
    float F[4];
    mixer_compute(MASS * GRAVITY, 0.0f, 0.0f, 0.1f, F);

    ASSERT_TRUE(F[0] > F[2], "yaw+: M1 > M3");
    ASSERT_TRUE(F[1] > F[3], "yaw+: M2 > M4");
    TEST_PASS("test_yaw_mix");
}

// Saturation: huge thrust request gets clamped to F_MOTOR_MAX.
static void test_clamp_high(void) {
    float F[4];
    mixer_compute(100.0f, 0.0f, 0.0f, 0.0f, F);
    for (int i = 0; i < 4; i++)
        ASSERT_NEAR(F[i], F_MOTOR_MAX, 1e-6f, "clamp high");
    TEST_PASS("test_clamp_high");
}

// Saturation: zero thrust with big roll → some motors want negative, clamped to 0.
static void test_clamp_low(void) {
    float F[4];
    mixer_compute(0.0f, 0.5f, 0.0f, 0.0f, F);
    for (int i = 0; i < 4; i++) ASSERT_TRUE(F[i] >= 0.0f, "no negative thrust");
    TEST_PASS("test_clamp_low");
}

int main(void) {
    test_hover_mix();
    test_roll_mix();
    test_pitch_mix();
    test_yaw_mix();
    test_clamp_high();
    test_clamp_low();
    printf("All mixer tests passed.\n");
    return 0;
}

