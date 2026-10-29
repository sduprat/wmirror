#ifndef WASM_MATH_H
#define WASM_MATH_H

#define M_PI 3.14159265358979323846

extern double sin(double x);
extern double cos(double x);
extern double atan2(double y, double x);
extern double asin(double x);

double fabs(double x);
double sqrt(double x);
double hypot(double x, double y);

#endif
