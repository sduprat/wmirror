#ifndef WMIRROR_LOGIC_H
#define WMIRROR_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Normalizes an angle in degrees to the range [0.0, 360.0).
 *
 * @param angle Angle in degrees to normalize.
 * @return Normalized angle in degrees within [0.0, 360.0).
 */
double normalize_angle_360(double angle);

/**
 * @brief Clamps a double value between a minimum and maximum threshold.
 *
 * @param value The value to clamp.
 * @param min_value Minimum allowable threshold.
 * @param max_value Maximum allowable threshold.
 * @return Clamped double value within [min_value, max_value].
 */
double clamp_double(double value, double min_value, double max_value);

/**
 * @brief Clamps an integer value between a minimum and maximum threshold.
 *
 * @param value The value to clamp.
 * @param min_value Minimum allowable threshold.
 * @param max_value Maximum allowable threshold.
 * @return Clamped integer value within [min_value, max_value].
 */
int clamp_int(int value, int min_value, int max_value);

/**
 * @brief Normalizes and clamps the horizontal servo angle value to physical limits [0.0, 180.0].
 *
 * @param value Raw horizontal servo target angle in degrees.
 * @return Clamped horizontal servo angle in degrees within [0.0, 180.0].
 */
double normalize_servo_h(double value);

/**
 * @brief Normalizes and clamps the vertical servo angle value to physical limits [0.0, 90.0].
 *
 * @param value Raw vertical servo target angle in degrees.
 * @return Clamped vertical servo angle in degrees within [0.0, 90.0].
 */
double normalize_servo_v(double value);

/**
 * @brief Converts horizontal spherical coordinates (azimuth, elevation) to a 3D unit Cartesian vector.
 *
 * @param azimuth_deg Azimuth angle in degrees.
 * @param elevation_deg Elevation angle in degrees.
 * @param[out] cartesian Array of 3 doubles to store resulting (X, Y, Z) Cartesian coordinates.
 */
void horz2cart(double azimuth_deg, double elevation_deg, double cartesian[3]);

/**
 * @brief Converts 3D Cartesian coordinates to horizontal spherical coordinates (azimuth, elevation).
 *
 * @param cartesian Array of 3 doubles representing input (X, Y, Z) Cartesian coordinates.
 * @param[out] azimuth_deg Pointer to store calculated azimuth in degrees in range [0.0, 360.0).
 * @param[out] elevation_deg Pointer to store calculated elevation angle in degrees in range [-90.0, 90.0].
 */
void cartesian_to_spherical(const double cartesian[3], double *azimuth_deg, double *elevation_deg);

/**
 * @brief Calculates the unit normal reflection vector bisecting the incident and reflected vectors.
 *
 * @param incident Array of 3 doubles representing the incident ray vector.
 * @param reflected Array of 3 doubles representing the reflected ray vector.
 * @param[out] normal Array of 3 doubles to store the resulting normalized normal vector.
 */
void calculer_vecteur_normal(const double incident[3], const double reflected[3], double normal[3]);

/**
 * @brief Computes the required mirror surface normal vector angles given sun and target angles.
 *
 * @param sun_azimuth_deg Sun azimuth angle in degrees.
 * @param sun_altitude_deg Sun altitude/elevation angle in degrees.
 * @param target_azimuth_deg Target azimuth angle in degrees.
 * @param target_altitude_deg Target altitude/elevation angle in degrees.
 * @param[out] normal_azimuth_deg Pointer to store calculated normal vector azimuth in degrees.
 * @param[out] normal_altitude_deg Pointer to store calculated normal vector altitude in degrees.
 */
void compute_norm_from_angles(double sun_azimuth_deg,
                             double sun_altitude_deg,
                             double target_azimuth_deg,
                             double target_altitude_deg,
                             double *normal_azimuth_deg,
                             double *normal_altitude_deg);

/**
 * @brief Converts 3D room coordinates (X, Y, Z in meters) relative to mirror into azimuth and elevation angles.
 *
 * @param x_m Target X position in meters.
 * @param y_m Target Y position in meters.
 * @param z_m Target Z position in meters.
 * @param[out] azimuth_deg Pointer to store calculated azimuth angle in degrees in range [0.0, 360.0).
 * @param[out] elevation_deg Pointer to store calculated elevation angle in degrees in range [-90.0, 90.0].
 */
void room_xy_to_az_el(double x_m,
                     double y_m,
                     double z_m,
                     double *azimuth_deg,
                     double *elevation_deg);

#ifdef __cplusplus
}
#endif

#endif
