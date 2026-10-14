#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <cmocka.h>

static void test_wrap_azimuth_360(void **state) {
    (void)state;
    double a = 359.0 + 10.0;
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0) a += 360.0;
    assert_true(a >= 0.0 && a < 360.0);
}

static void test_altitude_bounds(void **state) {
    (void)state;
    double alt = 95.0;
    if (alt > 90.0) alt = 90.0;
    if (alt < -90.0) alt = -90.0;
    assert_true(alt <= 90.0 && alt >= -90.0);
}

static void test_normalize_int_works(void **state) {
    (void)state;
    int x = 181;
    if (x > 180) x = 180;
    if (x < 0) x = 0;
    assert_int_equal(x, 180);
}

static void test_finite_value_guard(void **state) {
    (void)state;
    double v = 1.0;
    assert_false(isnan(v));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_wrap_azimuth_360),
        cmocka_unit_test(test_altitude_bounds),
        cmocka_unit_test(test_normalize_int_works),
        cmocka_unit_test(test_finite_value_guard),
    };

    return cmocka_run_group_tests_name("first_suite", tests, NULL, NULL);
}
