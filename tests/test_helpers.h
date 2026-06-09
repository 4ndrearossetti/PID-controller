#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define TEST_EPS 1e-4f

#define ASSERT_NEAR(actual, expected, eps, msg) do {                       \
    float _a = (actual), _e = (expected);                                  \
    if (fabsf(_a - _e) > (eps)) {                                          \
        fprintf(stderr, "FAIL %s:%d  %s\n"                                 \
                        "  expected %.6f, got %.6f (diff %.6f)\n",         \
                __FILE__, __LINE__, (msg), _e, _a, _a - _e);               \
        exit(1);                                                           \
    }                                                                      \
} while (0)

#define ASSERT_TRUE(cond, msg) do {                                        \
    if (!(cond)) {                                                         \
        fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));    \
        exit(1);                                                           \
    }                                                                      \
} while (0)

#define TEST_PASS(name) printf("PASS  %s\n", (name))

#endif
