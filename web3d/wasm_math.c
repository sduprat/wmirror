#include <math.h>

double fabs(double x) {
    return __builtin_fabs(x);
}

double sqrt(double x) {
    return __builtin_sqrt(x);
}

double hypot(double x, double y) {
    return __builtin_sqrt(x * x + y * y);
}
