#include "wmirror_logic.h"

#include <math.h>
#include <string.h>

static double wrap_to_360(double angle) {
    while (angle >= 360.0) {
        angle -= 360.0;
    }
    while (angle < 0.0) {
        angle += 360.0;
    }
    return angle;
}

double normalize_angle_360(double angle) {
    return wrap_to_360(angle);
}

double clamp_double(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

double normalize_servo_h(double value) {
    return clamp_double(value, 0.0, 180.0);
}

double normalize_servo_v(double value) {
    return clamp_double(value, 0.0, 90.0);
}

void horz2cart(double azimuth_deg, double elevation_deg, double cartesian[3]) {
    const double az = azimuth_deg * M_PI / 180.0;
    const double el = elevation_deg * M_PI / 180.0;
    const double cos_el = cos(el);

    cartesian[0] = cos_el * cos(az);
    cartesian[1] = cos_el * sin(az);
    cartesian[2] = sin(el);
}

void cartesian_to_spherical(const double cartesian[3], double *azimuth_deg, double *elevation_deg) {
    const double x = cartesian[0];
    const double y = cartesian[1];
    const double z = cartesian[2];
    const double r = sqrt(x * x + y * y + z * z);

    if (r == 0.0) {
        *azimuth_deg = 0.0;
        *elevation_deg = 0.0;
        return;
    }

    *azimuth_deg = atan2(y, x) * 180.0 / M_PI;
    *elevation_deg = asin(z / r) * 180.0 / M_PI;
    *azimuth_deg = normalize_angle_360(*azimuth_deg);
}

void calculer_vecteur_normal(const double incident[3], const double reflected[3], double normal[3]) {
    double avg[3];
    double length;

    avg[0] = (incident[0] + reflected[0]) / 2.0;
    avg[1] = (incident[1] + reflected[1]) / 2.0;
    avg[2] = (incident[2] + reflected[2]) / 2.0;

    length = sqrt(avg[0] * avg[0] + avg[1] * avg[1] + avg[2] * avg[2]);
    if (length == 0.0) {
        normal[0] = 0.0;
        normal[1] = 0.0;
        normal[2] = 0.0;
        return;
    }

    normal[0] = avg[0] / length;
    normal[1] = avg[1] / length;
    normal[2] = avg[2] / length;
}

void compute_norm_from_angles(double sun_azimuth_deg,
                             double sun_altitude_deg,
                             double target_azimuth_deg,
                             double target_altitude_deg,
                             double *normal_azimuth_deg,
                             double *normal_altitude_deg) {
    double sun_cart[3];
    double target_cart[3];
    double normal_cart[3];

    horz2cart(sun_azimuth_deg, sun_altitude_deg, sun_cart);
    horz2cart(target_azimuth_deg, target_altitude_deg, target_cart);
    calculer_vecteur_normal(sun_cart, target_cart, normal_cart);
    cartesian_to_spherical(normal_cart, normal_azimuth_deg, normal_altitude_deg);
}
