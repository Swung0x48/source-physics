#ifndef IVP_U_X87_INCLUDED
#define IVP_U_X87_INCLUDED

#include <stdint.h>
#include <string.h>
#include <ivu_x87_config.hxx>

class IVP_X87_Arithmetic;
struct IVP_X87_Control;

#if IVP_X87_HARD_LONG_DOUBLE
// An extended value in the host's x87 format (ivu_x87_config.hxx): the register itself, so HARD arithmetic
// compiles to plain x87 instructions. The bit accessors read the same 80 bits as the portable form.
class IVP_U_Float80
{
    long double value;

public:
    IVP_U_Float80() : value(0) {}
    static IVP_U_Float80 from_native(long double native)
    {
        IVP_U_Float80 result;
        result.value = native;
        return result;
    }
    long double native() const { return value; }
    static IVP_U_Float80 from_bits(uint16_t exponent, uint64_t fraction)
    {
        unsigned char bytes[sizeof(long double)] = {};
        memcpy(bytes, &fraction, 8);
        memcpy(bytes + 8, &exponent, 2);
        IVP_U_Float80 result;
        memcpy(&result.value, bytes, sizeof bytes);
        return result;
    }
    static IVP_U_Float80 from_bytes(const unsigned char *bytes);
    void to_bytes(unsigned char *bytes) const;
    uint64_t get_significand() const
    {
        uint64_t fraction;
        memcpy(&fraction, &value, 8);
        return fraction;
    }
    uint16_t get_sign_exponent() const
    {
        uint16_t exponent;
        memcpy(&exponent, reinterpret_cast<const unsigned char *>(&value) + 8, 2);
        return exponent;
    }
};
#else
// An extended value has no dependency on the host floating-point ABI.
class IVP_U_Float80
{
    uint64_t significand;
    uint16_t sign_exponent;

public:
    IVP_U_Float80() : significand(0), sign_exponent(0) {}
    static IVP_U_Float80 from_bits(uint16_t exponent, uint64_t fraction)
    {
        IVP_U_Float80 result;
        result.sign_exponent = exponent;
        result.significand = fraction;
        return result;
    }
    static IVP_U_Float80 from_bytes(const unsigned char *bytes);
    void to_bytes(unsigned char *bytes) const;
    uint64_t get_significand() const { return significand; }
    uint16_t get_sign_exponent() const { return sign_exponent; }
};
#endif

#include <ivu_x87_fast.hxx>
#if (IVP_X87_ARITHMETIC == IVP_X87_HARD || IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD)
#include <ivu_x87_native.hxx>
#endif

#if IVP_X87_HARD_LONG_DOUBLE
// HARD arithmetic with an x87 long double (ivu_x87_config.hxx): the executing FPU computes, inline. Constructing
// an object makes the FPU's precision and rounding the object's (IVP_X87_Control::set keeps them so already, so
// normally nothing changes); the destructor restores the word it found. Results equal the SOFT selection's bit
// for bit. Only the transcendental operations use the software model (their last bit differs between
// processor vendors), with a status initialized when they first need it.
inline unsigned short ivp_x87_host_control()
{
    unsigned short control;
    __asm__ __volatile__("fnstcw %0" : "=m"(control));
    return control;
}

inline void ivp_x87_set_host_control(unsigned short control)
{
    __asm__ __volatile__("fldcw %0" : : "m"(control) : "memory");
}

class IVP_X87_Arithmetic
{
    mutable union StatusStorage
    {
        uint64_t alignment;
        unsigned char bytes[16];
    } status_storage;
    mutable bool status_ready;
    // kept for the out-of-line members shared with the other selections
    bool fast_path;
    bool fast_inexact;
    bool native_path;
    unsigned short host_control;   // the FPU's word when this object was made
    unsigned short object_control; // host_control with this object's precision and rounding

    IVP_X87_Arithmetic(const IVP_X87_Arithmetic &);
    IVP_X87_Arithmetic &operator=(const IVP_X87_Arithmetic &);
    void init_status() const;
    void *get_status()
    {
        if(!status_ready) init_status();
        return status_storage.bytes;
    }
    const void *get_status() const
    {
        if(!status_ready) init_status();
        return status_storage.bytes;
    }
    void enter(unsigned bits)
    {
        status_ready = false;
        fast_path = bits == 0x0300;
        fast_inexact = false;
        native_path = true;
        host_control = ivp_x87_host_control();
        object_control = static_cast<unsigned short>((host_control & ~0x0f00u) | bits);
        if(object_control != host_control) ivp_x87_set_host_control(object_control);
    }
    void change(unsigned mask, unsigned bits)
    {
        object_control = static_cast<unsigned short>((object_control & ~mask) | bits);
        ivp_x87_set_host_control(object_control);
        status_ready = false;
        fast_path = (object_control & 0x0f00) == 0x0300;
    }

    IVP_U_Float80 load_general(float value);
    IVP_U_Float80 load_general(double value);
    float store_float_general(const IVP_U_Float80 &value);
    double store_double_general(const IVP_U_Float80 &value);
    IVP_U_Float80 add_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 subtract_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 multiply_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 divide_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);

public:
    enum Precision { PRECISION_24 = 24, PRECISION_53 = 53, PRECISION_64 = 64 };
    enum Rounding { ROUND_NEAREST = 0, ROUND_DOWN = 1, ROUND_UP = 2, ROUND_TO_ZERO = 3 };
    enum Relation { LESS = -1, EQUAL = 0, GREATER = 1, UNORDERED = 2 };
    enum Exception
    {
        EXCEPTION_INVALID_OPERATION = 0x01,
        EXCEPTION_DENORMAL_INPUT = 0x02,
        EXCEPTION_DIVISION_BY_ZERO = 0x04,
        EXCEPTION_OVERFLOW = 0x08,
        EXCEPTION_UNDERFLOW = 0x10,
        EXCEPTION_INEXACT_RESULT = 0x20
    };

    static unsigned control_bits(Precision precision, Rounding rounding)
    {
        return (precision == PRECISION_24 ? 0x0000u : precision == PRECISION_53 ? 0x0200u : 0x0300u)
            | (static_cast<unsigned>(rounding) << 10);
    }

    explicit IVP_X87_Arithmetic(Precision precision = PRECISION_64, Rounding rounding = ROUND_NEAREST)
    {
        enter(control_bits(precision, rounding));
    }
    explicit IVP_X87_Arithmetic(const IVP_X87_Control &control);
    ~IVP_X87_Arithmetic()
    {
        if(object_control != host_control) ivp_x87_set_host_control(host_control);
    }
    void set_precision(Precision precision) { change(0x0300, control_bits(precision, ROUND_NEAREST) & 0x0300); }
    void set_rounding(Rounding rounding) { change(0x0c00, static_cast<unsigned>(rounding) << 10); }
    // the executing FPU's flags (they are the thread's, not the object's)
    unsigned get_exception_flags() const
    {
        unsigned short status;
        __asm__ __volatile__("fnstsw %0" : "=m"(status));
        return status & 0x3f;
    }
    void clear_exception_flags() { __asm__ __volatile__("fnclex"); }

    IVP_U_Float80 load(float value) { return IVP_U_Float80::from_native(value); }
    IVP_U_Float80 load(double value) { return IVP_U_Float80::from_native(value); }
    IVP_U_Float80 load(int32_t value) { return IVP_U_Float80::from_native(value); }
    float store_float(const IVP_U_Float80 &value) { return ivp_x87_native_store_float(value); }
    double store_double(const IVP_U_Float80 &value) { return ivp_x87_native_store_double(value); }
    float return_float(const IVP_U_Float80 &value) { return store_float(value); }
    double return_double(const IVP_U_Float80 &value) { return store_double(value); }

    IVP_U_Float80 add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
        return IVP_U_Float80::from_native(first.native() + second.native());
    }
    IVP_U_Float80 subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
        return IVP_U_Float80::from_native(first.native() - second.native());
    }
    IVP_U_Float80 multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
        return IVP_U_Float80::from_native(first.native() * second.native());
    }
    IVP_U_Float80 divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
        return IVP_U_Float80::from_native(first.native() / second.native());
    }
    IVP_U_Float80 square_root(const IVP_U_Float80 &value) { return ivp_x87_native_square_root(value); }
    IVP_U_Float80 absolute(const IVP_U_Float80 &value)
    {
        long double operand = value.native();
        __asm__("fabs" : "+t"(operand));
        return IVP_U_Float80::from_native(operand);
    }
    IVP_U_Float80 negate(const IVP_U_Float80 &value)
    {
        long double operand = value.native();
        __asm__("fchs" : "+t"(operand));
        return IVP_U_Float80::from_native(operand);
    }
    Relation compare(const IVP_U_Float80 &first, const IVP_U_Float80 &second, bool quiet = false)
    {
        (void)quiet;
        const long double a = first.native(), b = second.native();
        return a < b ? LESS : a > b ? GREATER : a == b ? EQUAL : UNORDERED;
    }

    IVP_U_Float80 sine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 cosine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x);
    IVP_U_Float80 arcsine(const IVP_U_Float80 &value);
    IVP_U_Float80 arccosine(const IVP_U_Float80 &value);
    IVP_U_Float80 power2_minus1(const IVP_U_Float80 &value);
    IVP_U_Float80 log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y);
    IVP_U_Float80 round_integer(const IVP_U_Float80 &value);
    IVP_U_Float80 scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent);
    IVP_U_Float80 natural_logarithm_of_two() const;
    IVP_U_Float80 logarithm2_of_e() const;
    IVP_U_Float80 exponential(const IVP_U_Float80 &value);
};
#else
// Each numerical kernel has explicit precision, rounding and exception status.
// Its storage is fixed; constructing or operating on it never allocates.
class IVP_X87_Arithmetic
{
    union StatusStorage
    {
        uint64_t alignment;
        unsigned char bytes[16];
    } status_storage;
    // 64-bit precision with nearest-even rounding: normal cases of the
    // inline operations use ivu_x87_fast.hxx. Their inexact results are
    // merged into the exception flags.
    bool fast_path;
    bool fast_inexact;
    // HARD selections: fast_path while the executing FPU also uses 64-bit
    // precision and nearest rounding. Those operations then run on the host
    // x87 and raise its exceptions, not the object's.
    bool native_path;

    IVP_X87_Arithmetic(const IVP_X87_Arithmetic &);
    IVP_X87_Arithmetic &operator=(const IVP_X87_Arithmetic &);
    void *get_status() { return status_storage.bytes; }
    const void *get_status() const { return status_storage.bytes; }
    void update_fast_path();

    IVP_U_Float80 load_general(float value);
    IVP_U_Float80 load_general(double value);
    float store_float_general(const IVP_U_Float80 &value);
    double store_double_general(const IVP_U_Float80 &value);
    IVP_U_Float80 add_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 subtract_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 multiply_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);
    IVP_U_Float80 divide_general(const IVP_U_Float80 &first, const IVP_U_Float80 &second);

public:
    enum Precision { PRECISION_24 = 24, PRECISION_53 = 53, PRECISION_64 = 64 };
    enum Rounding { ROUND_NEAREST = 0, ROUND_DOWN = 1, ROUND_UP = 2, ROUND_TO_ZERO = 3 };
    enum Relation { LESS = -1, EQUAL = 0, GREATER = 1, UNORDERED = 2 };
    enum Exception
    {
        EXCEPTION_INVALID_OPERATION = 0x01,
        EXCEPTION_DENORMAL_INPUT = 0x02,
        EXCEPTION_DIVISION_BY_ZERO = 0x04,
        EXCEPTION_OVERFLOW = 0x08,
        EXCEPTION_UNDERFLOW = 0x10,
        EXCEPTION_INEXACT_RESULT = 0x20
    };

    explicit IVP_X87_Arithmetic(Precision precision = PRECISION_64,
        Rounding rounding = ROUND_NEAREST);
    explicit IVP_X87_Arithmetic(const IVP_X87_Control &control);
    void set_precision(Precision precision);
    void set_rounding(Rounding rounding);
    unsigned get_exception_flags() const;
    void clear_exception_flags();

    IVP_U_Float80 load(float value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_load_float(value);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_load_float(value, &result)) return result;
        return load_general(value);
    }
    IVP_U_Float80 load(double value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_load_double(value);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_load_double(value, &result)) return result;
        return load_general(value);
    }
    IVP_U_Float80 load(int32_t value);
    float store_float(const IVP_U_Float80 &value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_store_float(value);
#endif
        float result;
        if(fast_path && ivp_x87_fast_store_float(value, &result, &fast_inexact)) return result;
        return store_float_general(value);
    }
    double store_double(const IVP_U_Float80 &value)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_store_double(value);
#endif
        double result;
        if(fast_path && ivp_x87_fast_store_double(value, &result, &fast_inexact)) return result;
        return store_double_general(value);
    }
    // Scalar API returns use the host ABI. On a native x87 ABI this retains
    // the extended result; internal portable call chains use IVP_U_Float80.
    float return_float(const IVP_U_Float80 &value);
    double return_double(const IVP_U_Float80 &value);

    IVP_U_Float80 add(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_add(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_add(first, second, 0, &result, &fast_inexact)) return result;
        return add_general(first, second);
    }
    IVP_U_Float80 subtract(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_subtract(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_add(first, second, 1, &result, &fast_inexact)) return result;
        return subtract_general(first, second);
    }
    IVP_U_Float80 multiply(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_multiply(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_multiply(first, second, &result, &fast_inexact)) return result;
        return multiply_general(first, second);
    }
    IVP_U_Float80 divide(const IVP_U_Float80 &first, const IVP_U_Float80 &second)
    {
#if IVP_X87_ARITHMETIC == IVP_X87_HARD
        if(native_path) return ivp_x87_native_divide(first, second);
#endif
        IVP_U_Float80 result;
        if(fast_path && ivp_x87_fast_divide(first, second, &result, &fast_inexact)) return result;
        return divide_general(first, second);
    }
    IVP_U_Float80 square_root(const IVP_U_Float80 &value);
    IVP_U_Float80 absolute(const IVP_U_Float80 &value);
    IVP_U_Float80 negate(const IVP_U_Float80 &value);
    Relation compare(const IVP_U_Float80 &first, const IVP_U_Float80 &second,
        bool quiet = false);

    IVP_U_Float80 sine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 cosine(const IVP_U_Float80 &value, bool *out_of_range = 0);
    IVP_U_Float80 arctangent(const IVP_U_Float80 &y, const IVP_U_Float80 &x);
    // The C runtime's x87 asin and acos: for |x| < 1, an arctangent of x and
    // the square root of (1 + x)(1 - x) at the current precision with nearest
    // rounding; stored constants for x = +-1; the default NaN outside the
    // domain. Unless the control is 53-bit nearest, the runtime returns the
    // result rounded to binary64.
    IVP_U_Float80 arcsine(const IVP_U_Float80 &value);
    IVP_U_Float80 arccosine(const IVP_U_Float80 &value);
    IVP_U_Float80 power2_minus1(const IVP_U_Float80 &value);
    IVP_U_Float80 log2_multiply(const IVP_U_Float80 &x, const IVP_U_Float80 &y);
    IVP_U_Float80 round_integer(const IVP_U_Float80 &value);
    IVP_U_Float80 scale(const IVP_U_Float80 &value, const IVP_U_Float80 &exponent);
    IVP_U_Float80 natural_logarithm_of_two() const;
    IVP_U_Float80 logarithm2_of_e() const;
    IVP_U_Float80 exponential(const IVP_U_Float80 &value);
};
#endif

// Precision and rounding belong to the executing thread, as in an x87 control
// word. An explicit setting also works on processors without a native x87 FPU.
struct IVP_X87_Control
{
    IVP_X87_Arithmetic::Precision precision;
    IVP_X87_Arithmetic::Rounding rounding;

    static IVP_X87_Control current();
    static void set(const IVP_X87_Control &control);
    static void reset();
};


#if IVP_X87_HARD_LONG_DOUBLE
// The thread's selection, read inline (ivu_x87.cxx defines the variables).
extern thread_local bool ivp_x87_control_is_explicit;
extern thread_local IVP_X87_Control ivp_x87_control;

inline IVP_X87_Control IVP_X87_Control::current()
{
    if(ivp_x87_control_is_explicit) return ivp_x87_control;
    IVP_X87_Control result = {IVP_X87_Arithmetic::PRECISION_64, IVP_X87_Arithmetic::ROUND_NEAREST};
    return result;
}

inline IVP_X87_Arithmetic::IVP_X87_Arithmetic(const IVP_X87_Control &control)
{
    enter(control_bits(control.precision, control.rounding));
}
#endif

#endif
