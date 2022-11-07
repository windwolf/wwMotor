#ifndef __WWCONTROL_FOC_BASE_HPP__
#define __WWCONTROL_FOC_BASE_HPP__

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"
#include "arm_math.h"

#ifdef __cplusplus
}
#endif

namespace wwMotor
{
using namespace std;

using Scalar = float;

template <typename T> struct Vector2
{
    T v1;
    T v2;

    Vector2() : v1(0), v2(0)
    {
    }
    Vector2(T v1, T v2) : v1(v1), v2(v2)
    {
    }

    Vector2<T> operator+(const Vector2<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 + other.v1;
        result.v2 = v2 + other.v2;
        return result;
    };
    Vector2<T> operator+(const T other) const
    {
        Vector2<T> result;
        result.v1 = v1 + other;
        result.v2 = v2 + other;
        return result;
    };
    void operator+=(const Vector2<T> &other)
    {
        v1 += other.v1;
        v2 += other.v2;
    };
    void operator+=(const T other)
    {
        v1 += other;
        v2 += other;
    };

    Vector2<T> operator-(const Vector2<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 - other.v1;
        result.v2 = v2 - other.v2;
        return result;
    };
    Vector2<T> operator-(const T &other) const
    {
        Vector2<T> result;
        result.v1 = v1 - other;
        result.v2 = v2 - other;
        return result;
    };
    void operator-=(const Vector2<T> &other)
    {
        v1 -= other.v1;
        v2 -= other.v2;
    };
    void operator-=(const T other)
    {
        v1 -= other;
        v2 -= other;
    };

    Vector2<T> operator*(const Vector2<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 * other.v1;
        result.v2 = v2 * other.v2;
        return result;
    };
    Vector2<T> operator*(const T other) const
    {
        Vector2<T> result;
        result.v1 = v1 * other;
        result.v2 = v2 * other;
        return result;
    };
    void operator*=(const Vector2<T> &other)
    {
        v1 *= other.v1;
        v2 *= other.v2;
    };
    void operator*=(const T other)
    {
        v1 *= other;
        v2 *= other;
    };

    Vector2<T> operator/(const Vector2<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 / other.v1;
        result.v2 = v2 / other.v2;
        return result;
    };
    Vector2<T> operator/(const T other) const
    {
        Vector2<T> result;
        result.v1 = v1 / other;
        result.v2 = v2 / other;
        return result;
    };
    void operator/=(const Vector2<T> &other)
    {
        v1 /= other.v1;
        v2 /= other.v2;
    };
    void operator/=(const T other)
    {
        v1 /= other;
        v2 /= other;
    };

    // Vector2<T> operator%(const T other) const
    // {
    //     Vector2<T> result;
    //     result.v1 = Math::mod(v1, other);
    //     result.v2 = Math::mod(v2, other);
    //     return result;
    // };
    // void operator%=(const T other)
    // {
    //     v1 = Math::mod(v1, other);
    //     v2 = Math::mod(v2, other);
    // };
};

using Vector2f = Vector2<Scalar>;
using Vector2i = Vector2<uint32_t>;

template <typename T> struct Vector3
{
    T v1;
    T v2;
    T v3;

    Vector3() : v1(0), v2(0), v3(0){};
    Vector3(T v1, T v2, T v3) : v1(v1), v3(v3){};

    Vector3<T> operator+(const Vector3<T> &other) const
    {
        Vector3<T> result;
        result.v1 = v1 + other.v1;
        result.v2 = v2 + other.v2;
        result.v3 = v3 + other.v3;
        return result;
    };
    Vector3<T> operator+(const T other) const
    {
        Vector3<T> result;
        result.v1 = v1 + other;
        result.v2 = v2 + other;
        result.v3 = v3 + other;
        return result;
    };
    void operator+=(const Vector3<T> &other)
    {
        v1 += other.v1;
        v2 += other.v2;
        v3 += other.v3;
    };
    void operator+=(const T other)
    {
        v1 += other;
        v2 += other;
        v3 += other;
    };

    Vector3<T> operator-(const Vector3<T> &other) const
    {
        Vector3<T> result;
        result.v1 = v1 - other.v1;
        result.v2 = v2 - other.v2;
        result.v3 = v3 - other.v3;
        return result;
    };
    Vector3<T> operator-(const T &other) const
    {
        Vector3<T> result;
        result.v1 = v1 - other;
        result.v2 = v2 - other;
        result.v3 = v3 - other;
        return result;
    };
    void operator-=(const Vector3<T> &other)
    {
        v1 -= other.v1;
        v2 -= other.v2;
        v3 -= other.v3;
    };
    void operator-=(const T other)
    {
        v1 -= other;
        v2 -= other;
        v3 -= other;
    };

    Vector3<T> operator*(const Vector3<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 * other.v1;
        result.v2 = v2 * other.v2;
        result.v3 = v3 * other.v3;
        return result;
    };
    Vector3<T> operator*(const T other) const
    {
        Vector3<T> result;
        result.v1 = v1 * other;
        result.v2 = v2 * other;
        result.v3 = v3 * other;
        return result;
    };
    void operator*=(const Vector3<T> &other)
    {
        v1 *= other.v1;
        v2 *= other.v2;
        v3 *= other.v3;
    };
    void operator*=(const T other)
    {
        v1 *= other;
        v2 *= other;
        v3 *= other;
    };

    Vector3<T> operator/(const Vector3<T> &other) const
    {
        Vector3<T> result;
        result.v1 = v1 / other.v1;
        result.v2 = v2 / other.v2;
        result.v3 = v3 / other.v3;
        return result;
    };
    Vector3<T> operator/(const T other) const
    {
        Vector3<T> result;
        result.v1 = v1 / other;
        result.v2 = v2 / other;
        result.v3 = v3 / other;
        return result;
    };
    void operator/=(const Vector3<T> &other)
    {
        v1 /= other.v1;
        v2 /= other.v2;
        v3 /= other.v3;
    };
    void operator/=(const T other)
    {
        v1 /= other;
        v2 /= other;
        v3 /= other;
    };
};

using Vector3f = Vector3<Scalar>;
using Vector3i = Vector3<uint32_t>;

template <typename T> struct Vector4
{
    T v1;
    T v2;
    T v3;
    T v4;

    Vector4() : v1(0), v2(0), v3(0), v4(0){};
    Vector4(T v1, T v2, T v3, T v4) : v1(v1), v3(v3), v4(0){};

    Vector4<T> operator+(const Vector4<T> &other) const
    {
        Vector4<T> result;
        result.v1 = v1 + other.v1;
        result.v2 = v2 + other.v2;
        result.v3 = v3 + other.v3;
        result.v4 = v4 + other.v4;
        return result;
    };
    Vector4<T> operator+(const T other) const
    {
        Vector4<T> result;
        result.v1 = v1 + other;
        result.v2 = v2 + other;
        result.v3 = v3 + other;
        result.v4 = v4 + other;
        return result;
    };
    void operator+=(const Vector4<T> &other)
    {
        v1 += other.v1;
        v2 += other.v2;
        v3 += other.v3;
        v4 += other.v4;
    };
    void operator+=(const T other)
    {
        v1 += other;
        v2 += other;
        v3 += other;
        v4 += other;
    };

    Vector4<T> operator-(const Vector4<T> &other) const
    {
        Vector4<T> result;
        result.v1 = v1 - other.v1;
        result.v2 = v2 - other.v2;
        result.v3 = v3 - other.v3;
        result.v4 = v4 - other.v4;
        return result;
    };
    Vector4<T> operator-(const T &other) const
    {
        Vector4<T> result;
        result.v1 = v1 - other;
        result.v2 = v2 - other;
        result.v3 = v3 - other;
        result.v4 = v4 - other;
        return result;
    };
    void operator-=(const Vector4<T> &other)
    {
        v1 -= other.v1;
        v2 -= other.v2;
        v3 -= other.v3;
        v4 -= other.v4;
    };
    void operator-=(const T other)
    {
        v1 -= other;
        v2 -= other;
        v3 -= other;
        v4 -= other;
    };

    Vector4<T> operator*(const Vector4<T> &other) const
    {
        Vector2<T> result;
        result.v1 = v1 * other.v1;
        result.v2 = v2 * other.v2;
        result.v3 = v3 * other.v3;
        result.v4 = v4 * other.v4;
        return result;
    };
    Vector4<T> operator*(const T other) const
    {
        Vector4<T> result;
        result.v1 = v1 * other;
        result.v2 = v2 * other;
        result.v3 = v3 * other;
        result.v4 = v4 * other;
        return result;
    };
    void operator*=(const Vector4<T> &other)
    {
        v1 *= other.v1;
        v2 *= other.v2;
        v3 *= other.v3;
        v4 *= other.v4;
    };
    void operator*=(const T other)
    {
        v1 *= other;
        v2 *= other;
        v3 *= other;
        v4 *= other;
    };

    Vector4<T> operator/(const Vector4<T> &other) const
    {
        Vector4<T> result;
        result.v1 = v1 / other.v1;
        result.v2 = v2 / other.v2;
        result.v3 = v3 / other.v3;
        result.v4 = v4 / other.v4;
        return result;
    };
    Vector4<T> operator/(const T other) const
    {
        Vector4<T> result;
        result.v1 = v1 / other;
        result.v2 = v2 / other;
        result.v3 = v3 / other;
        result.v4 = v4 / other;
        return result;
    };
    void operator/=(const Vector4<T> &other)
    {
        v1 /= other.v1;
        v2 /= other.v2;
        v3 /= other.v3;
        v4 /= other.v4;
    };
    void operator/=(const T other)
    {
        v1 /= other;
        v2 /= other;
        v3 /= other;
        v4 /= other;
    };
};

using Vector4f = Vector4<Scalar>;
using Vector4i = Vector4<uint32_t>;

#define _PI 3.14159265358979323846f
#define _2PI 6.28318530717958647692f
#define _PI_2 1.57079632679489661923f
#define _PI_3 1.04719755119659774615f
#define _2PI_3 2.09439510239319549231f
#define _4PI_3 4.18879020478639098462f
#define _5PI_3 5.23598775598298873078f
#define _SQRT3 1.73205080756887729352f
#define _SQRT3_2 0.86602540378443864676f
#define _1_SQRT3 0.57735026918962576450f

struct Math
{
    static Scalar atan2(Scalar y, Scalar x)
    {
        Scalar result;
        arm_atan2_f32(y, x, &result);
        return result;
    }
    static void sincos(Scalar theta, Scalar *sin, Scalar *cos)
    {
        arm_sin_cos_f32(theta, sin, cos);
    }

    static Scalar sin(Scalar theta)
    {
        return arm_sin_f32(theta);
    }

    static Scalar cos(Scalar theta)
    {
        return arm_cos_f32(theta);
    }

    static Scalar sqrt(Scalar x)
    {
        Scalar result;
        arm_sqrt_f32(x, &result);
        return result;
    }

    static Scalar mod(Scalar x, Scalar y)
    {
        Scalar result = std::fmod(x, y);
        return result >= 0 ? result : (result + y);
    }

    static Scalar sign(Scalar x)
    {
        return x >= 0 ? 1 : -1;
    }

    static Vector2f sign(Vector2f x)
    {
        Vector2f result;
        if (x.v1 > 0)
            result.v1 = 1.0f;
        else if (x.v1 < 0)

            result.v1 = -1.0f;
        else
            result.v1 = 0.0f;

        if (x.v2 > 0)
            result.v2 = 1.0f;
        else if (x.v2 < 0)
            result.v2 = -1.0f;
        else
            result.v2 = 0.0f;
        return result;
    }

    static Scalar circle_normalize(Scalar theta)
    {

        Scalar result = std::fmod(theta, _2PI);
        return result >= 0 ? result : (result + _2PI);
    }

    static Scalar floor(Scalar x)
    {
        return std::floor(x);
    }
};

struct Motor
{
  public:
    enum class FluxSetMode
    {
        Flux,
        BackEMFConstant,
    };

    struct Config
    {
        uint8_t PolePair;
        float Rs;
        float Ld;
        float Lq;
        FluxSetMode fluxMode;
        float Flux;

        /**
         * @brief Back EMF constant. Vs/rad
         *
         */
        float Ke;

        /**
         * @brief Nm/rad/s
         *
         */
        float Damping;

        /**
         * @brief Nm
         *
         */
        float Friction;

        /**
         * @brief Kgm^2
         *
         */
        float Inertia;

        /**
         * @brief rpm/V
         *
         */
        float KV;

        /**
         * @brief V
         *
         */
        float RatedVoltage;

        /**
         * @brief A
         *
         */
        float RatedCurrent;

        /**
         * @brief rpm
         *
         */
        float RatedSpeed;
    };

    struct Variable
    {

        Vector3f i_abc;
        Vector3f u_abc;
        Vector2f i_ab;
        Vector2f u_ab;
        Vector2f i_dq;
        Vector2f u_dq;
        Scalar pos_e;
        Scalar pos_m;
        Scalar speed_e;
        Scalar speed_m;
        uint8_t section;
    };

  public:
    // Config or parameters
    uint8_t polePair;
    float rs;
    float ld;
    float lq;
    float flux;

    float interia;
    float friction;

    // Limits
    float speed_limit;
    float u_bus_max;
    float i_bus_limit;
    float i_phase_limit;

    // Dynamic variables
    // float u_bus;
    // float u_bus_sq;

    // void init() {
    //     // Init dynamic variables
    //     u_bus = 0;
    //     u_bus_sq = 0;
    // };

    // /**
    //  * @brief Raw data fetched from sensor.
    //  *
    //  */
    // Variable mea;
    // /**
    //  * @brief Estimated data.
    //  * In sensorless mode, this is the data estimated by observer.
    //  * In sensor mode, this is the data fetched from sensor then process neccessary calculation.
    //  */
    // Variable est;
    // /**
    //  * @brief Reference data. This is the data used for control, and executor.
    //  *
    //  */
    // Variable cmd;
};

} // namespace wwMotor

#endif // __WWCONTROL_FOC_BASE_HPP__
