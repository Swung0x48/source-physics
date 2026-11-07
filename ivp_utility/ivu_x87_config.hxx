#ifndef IVP_U_X87_CONFIG_INCLUDED
#define IVP_U_X87_CONFIG_INCLUDED

// Extended-arithmetic selection (CMake: IVP_X87_ARITHMETIC and
// IVP_X87_ARITHMETIC_TRANSCENDENTAL, each OFF, SOFT or HARD).
//
// IVP_X87_ARITHMETIC: the restored x87 evaluation and write-back sequences.
//   OFF   ordinary floating-point expressions (no emulation)
//   SOFT  portable integer arithmetic, identical on every target
//   HARD  the host x87 unit; 32-bit x86 only
// IVP_X87_ARITHMETIC_TRANSCENDENTAL: IVP_Inline_Math's sine, cosine, arcsine,
// arccosine, arctangent and exponential.
//   OFF   the host C library
//   SOFT  the x87 instructions and C runtime sequences in software, identical
//         on every target
//   HARD  the host x87 instructions; 32-bit x86 only. Their last bit differs
//         between processor vendors, as the original build's results do.

#define IVP_X87_OFF 0
#define IVP_X87_SOFT 1
#define IVP_X87_HARD 2

#ifndef IVP_X87_ARITHMETIC
#if IVP_ENABLE_X87_EMULATION
#define IVP_X87_ARITHMETIC IVP_X87_SOFT
#else
#define IVP_X87_ARITHMETIC IVP_X87_OFF
#endif
#endif

#ifndef IVP_X87_ARITHMETIC_TRANSCENDENTAL
#define IVP_X87_ARITHMETIC_TRANSCENDENTAL IVP_X87_ARITHMETIC
#endif

// GCC and Clang on x86 and x86-64 (Clang on the MSVC targets with -mlong-double-80) give long double the x87
// extended format: HARD then evaluates as inline long double arithmetic and an extended value is a long double.
#if defined(__LDBL_MANT_DIG__) && __LDBL_MANT_DIG__ == 64 && (defined(__i386__) || defined(__x86_64__))
#define IVP_X87_LONG_DOUBLE 1
#else
#define IVP_X87_LONG_DOUBLE 0
#endif

#if (IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD) \
    && !(defined(_M_IX86) || defined(__i386__) || IVP_X87_LONG_DOUBLE)
#error "IVP_X87_ARITHMETIC / IVP_X87_ARITHMETIC_TRANSCENDENTAL HARD requires 32-bit x86 or an x87 long double"
#endif

// HARD arithmetic keeps extended values in long double registers.
#if IVP_X87_ARITHMETIC == IVP_X87_HARD && IVP_X87_LONG_DOUBLE
#define IVP_X87_HARD_LONG_DOUBLE 1
#else
#define IVP_X87_HARD_LONG_DOUBLE 0
#endif

#endif
