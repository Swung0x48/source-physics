// Transcendental math entry points used by the physics.
//
// The simulation must produce identical results on every platform it runs
// on (the game client and a headless server may run on different operating
// systems and CPUs).  IEEE arithmetic, sqrt, fabs, floor and friends are
// exactly rounded everywhere, but sin, cos, exp, log and the other
// transcendental functions are not: every C library implements them with its
// own polynomials and tables, and the results differ in the last bits.
//
// All such calls in the physics therefore go through the ivp_libm namespace.
// By default the functions forward to the C library.  When IVP_PORTABLE_BMMO
// is defined, the build must provide the ivp_libm_* symbols declared below
// (a correctly rounded or otherwise platform-independent implementation),
// and every platform then computes bit-identical results.
//
// The double-named functions deliberately take double even when called with
// a float argument, and the float-named functions take float; this mirrors
// the C library's behaviour and keeps the results of both configurations the
// same.

#ifndef IVU_LIBM_INCLUDED
#define IVU_LIBM_INCLUDED

#include <math.h>

#ifdef IVP_PORTABLE_BMMO
extern "C" {
double ivp_libm_sin(double);
double ivp_libm_cos(double);
double ivp_libm_tan(double);
double ivp_libm_asin(double);
double ivp_libm_acos(double);
double ivp_libm_atan(double);
double ivp_libm_atan2(double, double);
double ivp_libm_exp(double);
double ivp_libm_log(double);
double ivp_libm_pow(double, double);
float ivp_libm_sinf(float);
float ivp_libm_cosf(float);
float ivp_libm_tanf(float);
float ivp_libm_asinf(float);
float ivp_libm_acosf(float);
float ivp_libm_atanf(float);
float ivp_libm_atan2f(float, float);
float ivp_libm_expf(float);
float ivp_libm_logf(float);
float ivp_libm_powf(float, float);
}
#define IVP_LIBM_CALL(name) ivp_libm_##name
#else
#define IVP_LIBM_CALL(name) ::name
#endif

namespace ivp_libm {
    inline double sin(double x) { return IVP_LIBM_CALL(sin)(x); }
    inline double cos(double x) { return IVP_LIBM_CALL(cos)(x); }
    inline double tan(double x) { return IVP_LIBM_CALL(tan)(x); }
    inline double asin(double x) { return IVP_LIBM_CALL(asin)(x); }
    inline double acos(double x) { return IVP_LIBM_CALL(acos)(x); }
    inline double atan(double x) { return IVP_LIBM_CALL(atan)(x); }
    inline double atan2(double y, double x) { return IVP_LIBM_CALL(atan2)(y, x); }
    inline double exp(double x) { return IVP_LIBM_CALL(exp)(x); }
    inline double log(double x) { return IVP_LIBM_CALL(log)(x); }
    inline double pow(double x, double y) { return IVP_LIBM_CALL(pow)(x, y); }

    inline float sinf(float x) { return IVP_LIBM_CALL(sinf)(x); }
    inline float cosf(float x) { return IVP_LIBM_CALL(cosf)(x); }
    inline float tanf(float x) { return IVP_LIBM_CALL(tanf)(x); }
    inline float asinf(float x) { return IVP_LIBM_CALL(asinf)(x); }
    inline float acosf(float x) { return IVP_LIBM_CALL(acosf)(x); }
    inline float atanf(float x) { return IVP_LIBM_CALL(atanf)(x); }
    inline float atan2f(float y, float x) { return IVP_LIBM_CALL(atan2f)(y, x); }
    inline float expf(float x) { return IVP_LIBM_CALL(expf)(x); }
    inline float logf(float x) { return IVP_LIBM_CALL(logf)(x); }
    inline float powf(float x, float y) { return IVP_LIBM_CALL(powf)(x, y); }
}

#undef IVP_LIBM_CALL

#endif
