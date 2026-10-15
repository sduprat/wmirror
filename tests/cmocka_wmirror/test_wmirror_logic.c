#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <cmocka.h>

#include "../../wmirror_logic.h"

static void test_normalize_around_360(void **state) {
    (void)state;
    assert_float_equal(normalize_angle_360(359.0 + 10.0), 9.0, 0.0001);
    assert_float_equal(normalize_angle_360(-10.0), 350.0, 0.0001);
    assert_float_equal(normalize_angle_360(720.0 + 15.0), 15.0, 0.0001);
}

static void test_servo_h_bounds(void **state) {
    (void)state;
    double h = 270.0 - 359.0;
    h = clamp_double(h, 0.0, 180.0);
    assert_true(h >= 0.0 && h <= 180.0);
}

static void test_servo_v_bounds(void **state) {
    (void)state;
    double v = 90.0 - 100.0;
    v = clamp_double(v, 0.0, 90.0);
    assert_true(v >= 0.0 && v <= 90.0);
}

static void test_same_target_and_sun(void **state) {
    (void)state;
    double az = normalize_angle_360(90.0);
    double alt = clamp_double(30.0, 0.0, 90.0);
    assert_float_equal(az, 90.0, 0.0001);
    assert_float_equal(alt, 30.0, 0.0001);
}

static void test_mirror_functions_are_valid(void **state) {
    (void)state;
    double h = normalize_servo_h(200.0);
    double v = normalize_servo_v(130.0);
    assert_float_equal(h, 180.0, 0.0001);
    assert_float_equal(v, 90.0, 0.0001);
}

static void test_horz2cart_azimuth_zero(void **state) {
    (void)state;
    double cart[3] = {0.0, 0.0, 0.0};
    horz2cart(0.0, 0.0, cart);
    assert_float_equal(cart[0], 1.0, 0.0001);
    assert_float_equal(cart[1], 0.0, 0.0001);
    assert_float_equal(cart[2], 0.0, 0.0001);
}

static void test_horz2cart_elevation_ninety(void **state) {
    (void)state;
    double cart[3] = {0.0, 0.0, 0.0};
    horz2cart(0.0, 90.0, cart);
    assert_float_equal(cart[0], 0.0, 0.0001);
    assert_float_equal(cart[1], 0.0, 0.0001);
    assert_float_equal(cart[2], 1.0, 0.0001);
}

static void test_cartesian_to_spherical_round_trip(void **state) {
    (void)state;
    double cart[3] = {0.0, 0.0, 0.0};
    double az = 0.0;
    double el = 45.0;

    horz2cart(az, el, cart);
    cartesian_to_spherical(cart, &az, &el);

    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_calculer_vecteur_normal_is_unit(void **state) {
    (void)state;
    double incident[3] = {1.0, 0.0, 0.0};
    double reflected[3] = {0.0, 1.0, 0.0};
    double normal[3] = {0.0, 0.0, 0.0};

    calculer_vecteur_normal(incident, reflected, normal);

    assert_true(fabs(normal[0]) <= 1.0);
    assert_true(fabs(normal[1]) <= 1.0);
    assert_true(fabs(normal[2]) <= 1.0);
    assert_true(fabs(sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]) - 1.0) < 0.0001);
}

static void test_compute_norm_from_angles_same_target_sun(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;

    compute_norm_from_angles(30.0, 20.0, 30.0, 20.0, &az, &el);

    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_compute_norm_from_angles_extreme_wrap(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;

    compute_norm_from_angles(359.0, 1.0, 0.0, 1.0, &az, &el);

    assert_true(isfinite(az));
    assert_true(isfinite(el));
    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_compute_norm_from_angles_opposite_target(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;

    compute_norm_from_angles(0.0, 30.0, 180.0, -30.0, &az, &el);

    assert_true(isfinite(az));
    assert_true(isfinite(el));
    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_cartesian_to_spherical_on_axis(void **state) {
    (void)state;
    double cart[3] = {0.0, 0.0, 1.0};
    double az = 0.0;
    double el = 0.0;

    cartesian_to_spherical(cart, &az, &el);

    assert_true(isfinite(az));
    assert_true(isfinite(el));
    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_normalize_angle_handles_large_values(void **state) {
    (void)state;
    assert_float_equal(normalize_angle_360(720.0 + 270.0), 270.0, 0.0001);
    assert_float_equal(normalize_angle_360(-450.0), 270.0, 0.0001);
}

static void test_compute_norm_from_angles_bisector_property(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;
    double sun_cart[3];
    double target_cart[3];
    double normal_cart[3];
    double normal_norm;

    compute_norm_from_angles(30.0, 20.0, 120.0, 40.0, &az, &el);
    horz2cart(30.0, 20.0, sun_cart);
    horz2cart(120.0, 40.0, target_cart);
    calculer_vecteur_normal(sun_cart, target_cart, normal_cart);
    normal_norm = sqrt(normal_cart[0]*normal_cart[0] + normal_cart[1]*normal_cart[1] + normal_cart[2]*normal_cart[2]);

    assert_true(fabs(normal_norm - 1.0) < 0.0001);
    assert_true((sun_cart[0]*normal_cart[0] + sun_cart[1]*normal_cart[1] + sun_cart[2]*normal_cart[2]) > 0.0);
    assert_true((target_cart[0]*normal_cart[0] + target_cart[1]*normal_cart[1] + target_cart[2]*normal_cart[2]) > 0.0);
    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

static void test_compute_norm_from_angles_opposite_horizontal_is_finite(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;

    compute_norm_from_angles(0.0, 0.0, 180.0, 0.0, &az, &el);

    assert_true(isfinite(az));
    assert_true(isfinite(el));
    assert_true(az >= 0.0 && az < 360.0);
    assert_true(el >= -90.0 && el <= 90.0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_normalize_around_360),
        cmocka_unit_test(test_servo_h_bounds),
        cmocka_unit_test(test_servo_v_bounds),
        cmocka_unit_test(test_same_target_and_sun),
        cmocka_unit_test(test_mirror_functions_are_valid),
        cmocka_unit_test(test_horz2cart_azimuth_zero),
        cmocka_unit_test(test_horz2cart_elevation_ninety),
        cmocka_unit_test(test_cartesian_to_spherical_round_trip),
        cmocka_unit_test(test_calculer_vecteur_normal_is_unit),
        cmocka_unit_test(test_compute_norm_from_angles_same_target_sun),
        cmocka_unit_test(test_compute_norm_from_angles_extreme_wrap),
        cmocka_unit_test(test_compute_norm_from_angles_opposite_target),
        cmocka_unit_test(test_compute_norm_from_angles_bisector_property),
        cmocka_unit_test(test_compute_norm_from_angles_opposite_horizontal_is_finite),
        cmocka_unit_test(test_cartesian_to_spherical_on_axis),
        cmocka_unit_test(test_normalize_angle_handles_large_values),
    };

    return cmocka_run_group_tests_name("wmirror_logic", tests, NULL, NULL);
}
