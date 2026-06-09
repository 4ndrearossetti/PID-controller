#include "../src/pid.h"
#include "test_helpers.h"

static const float DT = 0.005f;

/* Zero error → zero output, no integrator drift. */
static void test_zero_error(void) {
    PID_Controller pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
    pid.setpoint = 5.0f;

    float out = 0.0f;
    for (int i = 0; i < 100; i++) out = pid_update(&pid, 5.0f, DT);

    ASSERT_NEAR(out, 0.0f, 1e-5f, "zero error: output stays zero");
    ASSERT_NEAR(pid.integral, 0.0f, 1e-5f, "zero error: integral stays zero");
    TEST_PASS("test_zero_error");
}

/* P-only controller: output = kp · error after the priming call. */
static void test_p_only(void) {
    PID_Controller pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.setpoint = 10.0f;

    pid_update(&pid, 7.0f, DT);              /* priming call, returns kp·err */
    float out = pid_update(&pid, 7.0f, DT);  /* steady-state */

    ASSERT_NEAR(out, 2.0f * 3.0f, 1e-5f, "p-only: output = kp · error");
    TEST_PASS("test_p_only");
}

/* I-only controller: integral grows linearly with constant error. */
static void test_integral_accumulation(void) {
    PID_Controller pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, -1e6f, 1e6f);
    pid.setpoint = 1.0f;

    for (int i = 0; i < 100; i++) pid_update(&pid, 0.0f, DT);

    /* First call is the priming call: it doesn't update the integral.
     * Remaining 99 calls each add error·dt = 1·0.005. */
    float expected = 99.0f * DT;
    ASSERT_NEAR(pid.integral, expected, 1e-4f, "integral grows linearly");
    TEST_PASS("test_integral_accumulation");
}

/* D-on-measurement: setpoint steps do NOT produce a derivative kick. */
static void test_no_derivative_kick(void) {
    PID_Controller pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);

    pid.setpoint = 0.0f;
    pid_update(&pid, 0.0f, DT);     /* prime */
    pid_update(&pid, 0.0f, DT);     /* settled */

    pid.setpoint = 10.0f;            /* big setpoint step */
    float out = pid_update(&pid, 0.0f, DT);

    /* Feedback didn't change → derivative = 0 → output = 0.
     * D-on-error would have produced a huge spike here. */
    ASSERT_NEAR(out, 0.0f, 1e-5f, "setpoint step produces no D-kick");
    TEST_PASS("test_no_derivative_kick");
}

/* Output saturates to the upper limit on large positive error. */
static void test_output_clamp_high(void) {
    PID_Controller pid;
    pid_init(&pid, 1000.0f, 0.0f, 0.0f, -10.0f, 10.0f);
    pid.setpoint = 100.0f;

    pid_update(&pid, 0.0f, DT);
    float out = pid_update(&pid, 0.0f, DT);

    ASSERT_NEAR(out, 10.0f, 1e-5f, "output clamps to upper limit");
    TEST_PASS("test_output_clamp_high");
}

/* Output saturates to the lower limit on large negative error. */
static void test_output_clamp_low(void) {
    PID_Controller pid;
    pid_init(&pid, 1000.0f, 0.0f, 0.0f, -10.0f, 10.0f);
    pid.setpoint = -100.0f;

    pid_update(&pid, 0.0f, DT);
    float out = pid_update(&pid, 0.0f, DT);

    ASSERT_NEAR(out, -10.0f, 1e-5f, "output clamps to lower limit");
    TEST_PASS("test_output_clamp_low");
}

/* Anti-windup: integral freezes during sustained saturation. */
static void test_antiwindup_freeze(void) {
    PID_Controller pid;
    pid_init(&pid, 100.0f, 10.0f, 0.0f, -5.0f, 5.0f);
    pid.setpoint = 100.0f;

    for (int i = 0; i < 100; i++) pid_update(&pid, 0.0f, DT);
    float saturated_integral = pid.integral;

    /* 100 more steps still saturated; integral must not grow further. */
    for (int i = 0; i < 100; i++) pid_update(&pid, 0.0f, DT);

    ASSERT_NEAR(pid.integral, saturated_integral, 1e-5f,
                "integral frozen during sustained saturation");
    ASSERT_TRUE(pid.integral_clamped, "clamped flag set during saturation");
    TEST_PASS("test_antiwindup_freeze");
}

/* Anti-windup recovery: integral can move when the error reverses,
 * even if the output is still saturated. */
static void test_antiwindup_recovery(void) {
    PID_Controller pid;
    pid_init(&pid, 1.0f, 5.0f, 0.0f, -2.0f, 2.0f);
    pid.setpoint = 1.0f;

    /* Phase 1 — wind-up: error = 1, output starts at kp·1 = 1 (unsaturated).
     * Integral accumulates until output hits the +2 limit, then freezes. */
    for (int i = 0; i < 200; i++) pid_update(&pid, 0.0f, DT);
    float wound_up = pid.integral;
    ASSERT_TRUE(wound_up > 0.0f, "integral wound up before recovery");

    /* Phase 2 — small overshoot: feedback past setpoint, error reverses
     * sign. Output drops out of saturation, so the new (smaller) integral
     * is committed. Integral must decrease. */
    pid_update(&pid, 1.5f, DT);

    ASSERT_TRUE(pid.integral < wound_up,
                "integral decreases when error reverses");
    TEST_PASS("test_antiwindup_recovery");
}

/* pid_reset clears all internal state. */
static void test_reset(void) {
    PID_Controller pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
    pid.setpoint = 5.0f;

    for (int i = 0; i < 50; i++) pid_update(&pid, 0.0f, DT);
    ASSERT_TRUE(pid.integral != 0.0f, "integral non-zero before reset");

    pid_reset(&pid);
    ASSERT_NEAR(pid.integral, 0.0f, 1e-9f, "reset clears integral");
    ASSERT_TRUE(!pid.has_prev, "reset clears has_prev");
    TEST_PASS("test_reset");
}

int main(void) {
    test_zero_error();
    test_p_only();
    test_integral_accumulation();
    test_no_derivative_kick();
    test_output_clamp_high();
    test_output_clamp_low();
    test_antiwindup_freeze();
    test_antiwindup_recovery();
    test_reset();
    printf("All PID tests passed.\n");
    return 0;
}
