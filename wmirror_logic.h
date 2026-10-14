#ifndef WMIRROR_LOGIC_H
#define WMIRROR_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

double normalize_angle_360(double angle);
double clamp_double(double value, double min_value, double max_value);
int clamp_int(int value, int min_value, int max_value);

double normalize_servo_h(double value);
double normalize_servo_v(double value);

void horz2cart(double azimuth_deg, double elevation_deg, double cartesian[3]);
void cartesian_to_spherical(const double cartesian[3], double *azimuth_deg, double *elevation_deg);
void calculer_vecteur_normal(const double incident[3], const double reflected[3], double normal[3]);
void compute_norm_from_angles(double sun_azimuth_deg,
                             double sun_altitude_deg,
                             double target_azimuth_deg,
                             double target_altitude_deg,
                             double *normal_azimuth_deg,
                             double *normal_altitude_deg);

#ifdef __cplusplus
}
#endif

#endif
