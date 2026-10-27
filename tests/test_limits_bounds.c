#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <cmocka.h>

#include "../wmirror_logic.h"

#define EPSILON 0.0001

static void test_normalize_angle_360_exact_bounds(void **state) {
    (void)state;
    assert_float_equal(normalize_angle_360(0.0), 0.0, EPSILON);
    assert_float_equal(normalize_angle_360(360.0), 0.0, EPSILON);
    assert_float_equal(normalize_angle_360(-0.0), 0.0, EPSILON);
    assert_float_equal(normalize_angle_360(-360.0), 0.0, EPSILON);
}

static void test_normalize_angle_360_large_and_negative(void **state) {
    (void)state;
    assert_float_equal(normalize_angle_360(3600.0 + 45.0), 45.0, EPSILON);
    assert_float_equal(normalize_angle_360(-45.0), 315.0, EPSILON);
    assert_float_equal(normalize_angle_360(-3600.0 - 90.0), 270.0, EPSILON);
}

static void test_clamp_double_bounds(void **state) {
    (void)state;
    assert_float_equal(clamp_double(-10.0, 0.0, 100.0), 0.0, EPSILON);
    assert_float_equal(clamp_double(0.0, 0.0, 100.0), 0.0, EPSILON);
    assert_float_equal(clamp_double(50.0, 0.0, 100.0), 50.0, EPSILON);
    assert_float_equal(clamp_double(100.0, 0.0, 100.0), 100.0, EPSILON);
    assert_float_equal(clamp_double(150.0, 0.0, 100.0), 100.0, EPSILON);
}

static void test_clamp_int_bounds(void **state) {
    (void)state;
    assert_int_equal(clamp_int(-10, 0, 180), 0);
    assert_int_equal(clamp_int(0, 0, 180), 0);
    assert_int_equal(clamp_int(90, 0, 180), 90);
    assert_int_equal(clamp_int(180, 0, 180), 180);
    assert_int_equal(clamp_int(200, 0, 180), 180);
}

static void test_normalize_servo_limits(void **state) {
    (void)state;
    /* Servo H: [0, 180] */
    assert_float_equal(normalize_servo_h(-15.0), 0.0, EPSILON);
    assert_float_equal(normalize_servo_h(0.0), 0.0, EPSILON);
    assert_float_equal(normalize_servo_h(180.0), 180.0, EPSILON);
    assert_float_equal(normalize_servo_h(250.0), 180.0, EPSILON);

    /* Servo V: [0, 90] */
    assert_float_equal(normalize_servo_v(-10.0), 0.0, EPSILON);
    assert_float_equal(normalize_servo_v(0.0), 0.0, EPSILON);
    assert_float_equal(normalize_servo_v(90.0), 90.0, EPSILON);
    assert_float_equal(normalize_servo_v(120.0), 90.0, EPSILON);
}

static void test_horz2cart_cardinal_and_zenith(void **state) {
    (void)state;
    double cart[3];

    /* Azimuth 90, Elevation 0 -> (0, 1, 0) */
    horz2cart(90.0, 0.0, cart);
    assert_float_equal(cart[0], 0.0, EPSILON);
    assert_float_equal(cart[1], 1.0, EPSILON);
    assert_float_equal(cart[2], 0.0, EPSILON);

    /* Azimuth 180, Elevation 0 -> (-1, 0, 0) */
    horz2cart(180.0, 0.0, cart);
    assert_float_equal(cart[0], -1.0, EPSILON);
    assert_float_equal(cart[1], 0.0, EPSILON);
    assert_float_equal(cart[2], 0.0, EPSILON);

    /* Elevation -90 (Nadir) -> (0, 0, -1) */
    horz2cart(0.0, -90.0, cart);
    assert_float_equal(cart[0], 0.0, EPSILON);
    assert_float_equal(cart[1], 0.0, EPSILON);
    assert_float_equal(cart[2], -1.0, EPSILON);
}

static void test_cartesian_to_spherical_zero_and_small_vector(void **state) {
    (void)state;
    double az = 99.0;
    double el = 99.0;

    /* Zero vector */
    double zero_cart[3] = {0.0, 0.0, 0.0};
    cartesian_to_spherical(zero_cart, &az, &el);
    assert_float_equal(az, 0.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);

    /* Small vector below epsilon */
    double small_cart[3] = {1e-15, 1e-15, 1e-15};
    cartesian_to_spherical(small_cart, &az, &el);
    assert_float_equal(az, 0.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);
}

static void test_cartesian_to_spherical_nadir(void **state) {
    (void)state;
    double az = 0.0;
    double el = 0.0;
    double cart[3] = {0.0, 0.0, -5.0};

    cartesian_to_spherical(cart, &az, &el);
    assert_float_equal(el, -90.0, EPSILON);
}

static void test_calculer_vecteur_normal_coincident_and_opposite(void **state) {
    (void)state;
    double inc[3] = {1.0, 0.0, 0.0};
    double ref[3] = {1.0, 0.0, 0.0};
    double norm[3];

    /* Coincident vectors */
    calculer_vecteur_normal(inc, ref, norm);
    assert_float_equal(norm[0], 1.0, EPSILON);
    assert_float_equal(norm[1], 0.0, EPSILON);
    assert_float_equal(norm[2], 0.0, EPSILON);

    /* Opposite vectors -> zero normal output */
    double ref_opp[3] = {-1.0, 0.0, 0.0};
    calculer_vecteur_normal(inc, ref_opp, norm);
    assert_float_equal(norm[0], 0.0, EPSILON);
    assert_float_equal(norm[1], 0.0, EPSILON);
    assert_float_equal(norm[2], 0.0, EPSILON);
}

static void test_room_xy_to_az_el_quadrants_and_origin(void **state) {
    (void)state;
    double az, el;

    /* Origin (0,0,z) -> zenith 90 */
    room_xy_to_az_el(0.0, 0.0, 2.0, &az, &el);
    assert_float_equal(az, 0.0, EPSILON);
    assert_float_equal(el, 90.0, EPSILON);

    /* Quadrant 1 (+, +) */
    room_xy_to_az_el(1.0, 1.0, 0.0, &az, &el);
    assert_float_equal(az, 45.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);

    /* Quadrant 2 (-, +) */
    room_xy_to_az_el(-1.0, 1.0, 0.0, &az, &el);
    assert_float_equal(az, 135.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);

    /* Quadrant 3 (-, -) */
    room_xy_to_az_el(-1.0, -1.0, 0.0, &az, &el);
    assert_float_equal(az, 225.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);

    /* Quadrant 4 (+, -) */
    room_xy_to_az_el(1.0, -1.0, 0.0, &az, &el);
    assert_float_equal(az, 315.0, EPSILON);
    assert_float_equal(el, 0.0, EPSILON);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_normalize_angle_360_exact_bounds),
        cmocka_unit_test(test_normalize_angle_360_large_and_negative),
        cmocka_unit_test(test_clamp_double_bounds),
        cmocka_unit_test(test_clamp_int_bounds),
        cmocka_unit_test(test_normalize_servo_limits),
        cmocka_unit_test(test_horz2cart_cardinal_and_zenith),
        cmocka_unit_test(test_cartesian_to_spherical_zero_and_small_vector),
        cmocka_unit_test(test_cartesian_to_spherical_nadir),
        cmocka_unit_test(test_calculer_vecteur_normal_coincident_and_opposite),
        cmocka_unit_test(test_room_xy_to_az_el_quadrants_and_origin),
    };

    return cmocka_run_group_tests_name("limits_and_bounds", tests, NULL, NULL);
}
