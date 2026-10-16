#ifndef MATHC_INCLUDE_MATHC_H
#define MATHC_INCLUDE_MATHC_H

#include <cmath>
#include <cstdint>
#include <ostream>

namespace Math
{

    constexpr float PI = 3.14159265358979323846f;
    constexpr float DEG2RAD = PI / 180.0f;
    constexpr float RAD2DEG = 180.0f / PI;
    constexpr float EPSILON = 1e-6f;

    struct Vec2
    {
        union
        {
            struct
            {
                float x, y;
            };
            float v[2];
        };

        Vec2() : x(0.0f), y(0.0f) {}
        Vec2(float x, float y) : x(x), y(y) {}
        explicit Vec2(float scalar) : x(scalar), y(scalar) {}

        float &operator[](int index) { return v[index]; }
        const float &operator[](int index) const { return v[index]; }

        Vec2 operator+(const Vec2 &o) const { return Vec2(x + o.x, y + o.y); }
        Vec2 operator-(const Vec2 &o) const { return Vec2(x - o.x, y - o.y); }
        Vec2 operator*(const Vec2 &o) const { return Vec2(x * o.x, y * o.y); }
        Vec2 operator/(const Vec2 &o) const { return Vec2(x / o.x, y / o.y); }

        Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
        Vec2 operator/(float s) const
        {
            float inv = 1.0f / s;
            return Vec2(x * inv, y * inv);
        }

        Vec2 &operator+=(const Vec2 &o) { x += o.x; y += o.y; return *this; }
        Vec2 &operator-=(const Vec2 &o) { x -= o.x; y -= o.y; return *this; }
        Vec2 &operator*=(const Vec2 &o) { x *= o.x; y *= o.y; return *this; }
        Vec2 &operator/=(const Vec2 &o) { x /= o.x; y /= o.y; return *this; }
        Vec2 &operator*=(float s) { x *= s; y *= s; return *this; }
        Vec2 &operator/=(float s)
        {
            float inv = 1.0f / s;
            x *= inv; y *= inv;
            return *this;
        }

        Vec2 operator-() const { return Vec2(-x, -y); }

        bool operator==(const Vec2 &o) const
        {
            return std::fabs(x - o.x) < EPSILON && std::fabs(y - o.y) < EPSILON;
        }
        bool operator!=(const Vec2 &o) const { return !(*this == o); }

        float LengthSquared() const { return x * x + y * y; }
        float Length() const { return std::sqrt(LengthSquared()); }

        Vec2 Normalized() const
        {
            float inv = 1.0f / std::sqrt(LengthSquared());
            return Vec2(x * inv, y * inv);
        }

        Vec2 NormalizedSafe() const
        {
            float rawLenSq = LengthSquared();
            float lenSq = rawLenSq > EPSILON * EPSILON ? rawLenSq : EPSILON * EPSILON;
            float inv = 1.0f / std::sqrt(lenSq);
            return Vec2(x * inv, y * inv);
        }

        void Normalize() { *this = Normalized(); }
        void NormalizeSafe() { *this = NormalizedSafe(); }

        float Dot(const Vec2 &o) const { return x * o.x + y * o.y; }
        float Cross(const Vec2 &o) const { return x * o.y - y * o.x; }

        float Angle() const { return std::atan2(y, x); }
        float AngleDeg() const { return Angle() * RAD2DEG; }

        Vec2 Rotate(float angleRad) const
        {
            float s = std::sin(angleRad);
            float c = std::cos(angleRad);
            return Vec2(x * c - y * s, x * s + y * c);
        }
        Vec2 RotateDeg(float angleDeg) const { return Rotate(angleDeg * DEG2RAD); }

        static Vec2 FromAngle(float angleRad) { return Vec2(std::cos(angleRad), std::sin(angleRad)); }
        static Vec2 FromAngleDeg(float angleDeg) { return FromAngle(angleDeg * DEG2RAD); }
        static float AngleBetween(const Vec2 &a, const Vec2 &b) { return std::atan2(a.Cross(b), a.Dot(b)); }
        static float AngleBetweenDeg(const Vec2 &a, const Vec2 &b) { return AngleBetween(a, b) * RAD2DEG; }
        static float Dot(const Vec2 &a, const Vec2 &b) { return a.Dot(b); }
        static float Cross(const Vec2 &a, const Vec2 &b) { return a.Cross(b); }
        static float Distance(const Vec2 &a, const Vec2 &b) { return (a - b).Length(); }
        static float DistanceSquared(const Vec2 &a, const Vec2 &b) { return (a - b).LengthSquared(); }
        static Vec2 Lerp(const Vec2 &a, const Vec2 &b, float t) { return a + (b - a) * t; }
        static Vec2 Min(const Vec2 &a, const Vec2 &b) { return Vec2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y); }
        static Vec2 Max(const Vec2 &a, const Vec2 &b) { return Vec2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y); }
        static Vec2 Clamp(const Vec2 &v, const Vec2 &lo, const Vec2 &hi) { return Min(Max(v, lo), hi); }
        static const Vec2 Zero;
        static const Vec2 One;
        static const Vec2 UnitX;
        static const Vec2 UnitY;
    };

    inline Vec2 operator*(float scalar, const Vec2 &v) { return v * scalar; }
    std::ostream &operator<<(std::ostream &os, const Vec2 &v);

    struct Vec3
    {
        union
        {
            struct
            {
                float x, y, z;
            };
            float v[3];
        };

        Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
        explicit Vec3(float scalar) : x(scalar), y(scalar), z(scalar) {}
        Vec3(const Vec2 &xy, float z) : x(xy.x), y(xy.y), z(z) {}

        float &operator[](int index) { return v[index]; }
        const float &operator[](int index) const { return v[index]; }

        Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
        Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
        Vec3 operator*(const Vec3 &o) const { return Vec3(x * o.x, y * o.y, z * o.z); }
        Vec3 operator/(const Vec3 &o) const { return Vec3(x / o.x, y / o.y, z / o.z); }

        Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
        Vec3 operator/(float s) const
        {
            float inv = 1.0f / s;
            return Vec3(x * inv, y * inv, z * inv);
        }

        Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
        Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
        Vec3 &operator*=(const Vec3 &o) { x *= o.x; y *= o.y; z *= o.z; return *this; }
        Vec3 &operator/=(const Vec3 &o) { x /= o.x; y /= o.y; z /= o.z; return *this; }
        Vec3 &operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
        Vec3 &operator/=(float s)
        {
            float inv = 1.0f / s;
            x *= inv; y *= inv; z *= inv;
            return *this;
        }

        Vec3 operator-() const { return Vec3(-x, -y, -z); }

        bool operator==(const Vec3 &o) const
        {
            return std::fabs(x - o.x) < EPSILON && std::fabs(y - o.y) < EPSILON && std::fabs(z - o.z) < EPSILON;
        }
        bool operator!=(const Vec3 &o) const { return !(*this == o); }

        float LengthSquared() const { return x * x + y * y + z * z; }
        float Length() const { return std::sqrt(LengthSquared()); }

        Vec3 Normalized() const
        {
            float inv = 1.0f / std::sqrt(LengthSquared());
            return Vec3(x * inv, y * inv, z * inv);
        }

        Vec3 NormalizedSafe() const
        {
            float rawLenSq = LengthSquared();
            float lenSq = rawLenSq > EPSILON * EPSILON ? rawLenSq : EPSILON * EPSILON;
            float inv = 1.0f / std::sqrt(lenSq);
            return Vec3(x * inv, y * inv, z * inv);
        }

        void Normalize() { *this = Normalized(); }
        void NormalizeSafe() { *this = NormalizedSafe(); }

        float Dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
        Vec3 Cross(const Vec3 &o) const
        {
            return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
        }

        Vec2 xy() const { return Vec2(x, y); }

        Vec3 RotateX(float angleRad) const
        {
            float s = std::sin(angleRad);
            float c = std::cos(angleRad);
            return Vec3(x, y * c - z * s, y * s + z * c);
        }
        Vec3 RotateY(float angleRad) const
        {
            float s = std::sin(angleRad);
            float c = std::cos(angleRad);
            return Vec3(x * c + z * s, y, -x * s + z * c);
        }
        Vec3 RotateZ(float angleRad) const
        {
            float s = std::sin(angleRad);
            float c = std::cos(angleRad);
            return Vec3(x * c - y * s, x * s + y * c, z);
        }
        Vec3 RotateXDeg(float angleDeg) const { return RotateX(angleDeg * DEG2RAD); }
        Vec3 RotateYDeg(float angleDeg) const { return RotateY(angleDeg * DEG2RAD); }
        Vec3 RotateZDeg(float angleDeg) const { return RotateZ(angleDeg * DEG2RAD); }

        static float AngleBetween(const Vec3 &a, const Vec3 &b) { return std::atan2(a.Cross(b).Length(), a.Dot(b)); }
        static float AngleBetweenDeg(const Vec3 &a, const Vec3 &b) { return AngleBetween(a, b) * RAD2DEG; }
        static float Dot(const Vec3 &a, const Vec3 &b) { return a.Dot(b); }
        static Vec3 Cross(const Vec3 &a, const Vec3 &b) { return a.Cross(b); }
        static float Distance(const Vec3 &a, const Vec3 &b) { return (a - b).Length(); }
        static float DistanceSquared(const Vec3 &a, const Vec3 &b) { return (a - b).LengthSquared(); }
        static Vec3 Lerp(const Vec3 &a, const Vec3 &b, float t) { return a + (b - a) * t; }
        static Vec3 Min(const Vec3 &a, const Vec3 &b)
        {
            return Vec3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
        }
        static Vec3 Max(const Vec3 &a, const Vec3 &b)
        {
            return Vec3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
        }
        static Vec3 Clamp(const Vec3 &v, const Vec3 &lo, const Vec3 &hi) { return Min(Max(v, lo), hi); }
        static const Vec3 Zero;
        static const Vec3 One;
        static const Vec3 UnitX;
        static const Vec3 UnitY;
        static const Vec3 UnitZ;
    };

    inline Vec3 operator*(float scalar, const Vec3 &v) { return v * scalar; }
    std::ostream &operator<<(std::ostream &os, const Vec3 &v);

    struct alignas(16) Vec4
    {
        union
        {
            struct
            {
                float x, y, z, w;
            };
            float v[4];
        };

        Vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
        Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
        explicit Vec4(float scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}
        Vec4(const Vec3 &xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
        Vec4(const Vec2 &xy, float z, float w) : x(xy.x), y(xy.y), z(z), w(w) {}

        float &operator[](int index) { return v[index]; }
        const float &operator[](int index) const { return v[index]; }

        Vec4 operator+(const Vec4 &o) const { return Vec4(x + o.x, y + o.y, z + o.z, w + o.w); }
        Vec4 operator-(const Vec4 &o) const { return Vec4(x - o.x, y - o.y, z - o.z, w - o.w); }
        Vec4 operator*(const Vec4 &o) const { return Vec4(x * o.x, y * o.y, z * o.z, w * o.w); }
        Vec4 operator/(const Vec4 &o) const { return Vec4(x / o.x, y / o.y, z / o.z, w / o.w); }
        Vec4 operator*(float s) const { return Vec4(x * s, y * s, z * s, w * s); }
        Vec4 operator/(float s) const
        {
            float inv = 1.0f / s;
            return Vec4(x * inv, y * inv, z * inv, w * inv);
        }

        Vec4 &operator+=(const Vec4 &o) { return *this = *this + o; }
        Vec4 &operator-=(const Vec4 &o) { return *this = *this - o; }
        Vec4 &operator*=(const Vec4 &o) { return *this = *this * o; }
        Vec4 &operator/=(const Vec4 &o) { return *this = *this / o; }
        Vec4 &operator*=(float s) { return *this = *this * s; }
        Vec4 &operator/=(float s) { return *this = *this / s; }

        Vec4 operator-() const { return Vec4(-x, -y, -z, -w); }

        bool operator==(const Vec4 &o) const
        {
            return std::fabs(x - o.x) < EPSILON && std::fabs(y - o.y) < EPSILON &&
                   std::fabs(z - o.z) < EPSILON && std::fabs(w - o.w) < EPSILON;
        }
        bool operator!=(const Vec4 &o) const { return !(*this == o); }

        float LengthSquared() const { return x * x + y * y + z * z + w * w; }
        float Length() const { return std::sqrt(LengthSquared()); }

        Vec4 Normalized() const
        {
            float inv = 1.0f / std::sqrt(LengthSquared());
            return *this * inv;
        }

        Vec4 NormalizedSafe() const
        {
            float rawLenSq = LengthSquared();
            float lenSq = rawLenSq > EPSILON * EPSILON ? rawLenSq : EPSILON * EPSILON;
            float inv = 1.0f / std::sqrt(lenSq);
            return *this * inv;
        }

        void Normalize() { *this = Normalized(); }
        void NormalizeSafe() { *this = NormalizedSafe(); }

        float Dot(const Vec4 &o) const { return x * o.x + y * o.y + z * o.z + w * o.w; }

        Vec2 xy() const { return Vec2(x, y); }
        Vec3 xyz() const { return Vec3(x, y, z); }

        static float AngleBetween(const Vec4 &a, const Vec4 &b)
        {
            float d = a.Dot(b) / (a.Length() * b.Length());
            d = d < -1.0f ? -1.0f : (d > 1.0f ? 1.0f : d);
            return std::acos(d);
        }
        static float AngleBetweenDeg(const Vec4 &a, const Vec4 &b) { return AngleBetween(a, b) * RAD2DEG; }
        static float Dot(const Vec4 &a, const Vec4 &b) { return a.Dot(b); }
        static float Distance(const Vec4 &a, const Vec4 &b) { return (a - b).Length(); }
        static float DistanceSquared(const Vec4 &a, const Vec4 &b) { return (a - b).LengthSquared(); }
        static Vec4 Lerp(const Vec4 &a, const Vec4 &b, float t) { return a + (b - a) * t; }
        static Vec4 Min(const Vec4 &a, const Vec4 &b)
        {
            return Vec4(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y,
                        a.z < b.z ? a.z : b.z, a.w < b.w ? a.w : b.w);
        }
        static Vec4 Max(const Vec4 &a, const Vec4 &b)
        {
            return Vec4(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y,
                        a.z > b.z ? a.z : b.z, a.w > b.w ? a.w : b.w);
        }
        static Vec4 Clamp(const Vec4 &v, const Vec4 &lo, const Vec4 &hi) { return Min(Max(v, lo), hi); }
        static const Vec4 Zero;
        static const Vec4 One;
        static const Vec4 UnitX;
        static const Vec4 UnitY;
        static const Vec4 UnitZ;
        static const Vec4 UnitW;
    };

    inline Vec4 operator*(float scalar, const Vec4 &v) { return v * scalar; }
    std::ostream &operator<<(std::ostream &os, const Vec4 &v);

    struct Mat2
    {
        Vec2 col0, col1;

        Mat2();
        explicit Mat2(float scalar);
        Mat2(float m00, float m01, float m10, float m11);
        Mat2(const Vec2 &col0, const Vec2 &col1);

        Vec2 &operator[](int col);
        const Vec2 &operator[](int col) const;
        float *Data();
        const float *Data() const;

        Mat2 operator+(const Mat2 &other) const;
        Mat2 operator-(const Mat2 &other) const;
        Mat2 operator*(const Mat2 &other) const;
        Mat2 operator*(float scalar) const;
        Vec2 operator*(const Vec2 &v) const;

        bool operator==(const Mat2 &other) const;
        bool operator!=(const Mat2 &other) const;

        Mat2 Transposed() const;
        float Determinant() const;
        Mat2 Inverse() const;
        Mat2 InverseSafe() const;

        Vec2 GetScale() const;
        float GetRotation() const;
        float GetSkew() const;

        static Mat2 Identity();
        static Mat2 Scale(float sx, float sy);
        static Mat2 Scale(const Vec2 &s);
        static Mat2 Rotation(float angleRad);
        static Mat2 SkewX(float angleRad);
        static Mat2 SkewY(float angleRad);

        static Mat2 FromScaleRotationSkew(const Vec2 &scale, float rotationRad, float skewRad);
    };

    Mat2 operator*(float scalar, const Mat2 &m);
    std::ostream &operator<<(std::ostream &os, const Mat2 &m);

#ifndef MATHC_NO_EXTRA
    struct Transform2D
    {
        Mat2 basis;
        Vec2 origin;

        Transform2D();
        Transform2D(const Mat2 &basis, const Vec2 &origin);

        Vec2 TransformPoint(const Vec2 &p) const;
        Vec2 TransformVector(const Vec2 &v) const;

        void TransformPoints(const Vec2 *in, Vec2 *out, int count) const;

        Transform2D operator*(const Transform2D &other) const;

        bool operator==(const Transform2D &other) const;
        bool operator!=(const Transform2D &other) const;

        Transform2D Inverse() const;
        Transform2D InverseSafe() const;

        Transform2D RigidInverse() const;

        static Transform2D Identity();
        static Transform2D Translation(const Vec2 &t);
        static Transform2D Rotation(float angleRad);
        static Transform2D ScaleBy(const Vec2 &s);
        static Transform2D Rigid(const Vec2 &translation, float rotationRad);

        static Transform2D RotationAround(const Vec2 &pivot, float angleRad);
        static Transform2D ScaleAround(const Vec2 &pivot, const Vec2 &scale);

        static Transform2D FromTRS(const Vec2 &translation, float rotationRad, const Vec2 &scale,
                                    float skewXRad, float skewYRad, const Vec2 &pivot = Vec2::Zero);
    };

    std::ostream &operator<<(std::ostream &os, const Transform2D &t);
#endif 

    struct Mat3
    {
        Vec3 col0, col1, col2;

        Mat3();
        explicit Mat3(float scalar);
        Mat3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22);
        Mat3(const Vec3 &col0, const Vec3 &col1, const Vec3 &col2);
#ifndef MATHC_NO_EXTRA

        explicit Mat3(const Transform2D &t);
#endif

        Vec3 &operator[](int col);
        const Vec3 &operator[](int col) const;
        float *Data();
        const float *Data() const;

        Mat3 operator+(const Mat3 &other) const;
        Mat3 operator-(const Mat3 &other) const;
        Mat3 operator*(const Mat3 &other) const;
        Mat3 operator*(float scalar) const;
        Vec3 operator*(const Vec3 &v) const;

        bool operator==(const Mat3 &other) const;
        bool operator!=(const Mat3 &other) const;

        Mat3 Transposed() const;
        float Determinant() const;
        Mat3 Inverse() const;
        Mat3 InverseSafe() const;

        Vec2 TransformPoint2D(const Vec2 &p) const;

        Vec2 TransformVector2D(const Vec2 &v) const;

        static Mat3 Identity();
        static Mat3 Scale(const Vec3 &s);
        static Mat3 RotationX(float angleRad);
        static Mat3 RotationY(float angleRad);
        static Mat3 RotationZ(float angleRad);
    };

    Mat3 operator*(float scalar, const Mat3 &m);
    std::ostream &operator<<(std::ostream &os, const Mat3 &m);

    struct alignas(16) Mat4
    {
        Vec4 col0, col1, col2, col3;

        Mat4();
        explicit Mat4(float scalar);
        Mat4(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13,
             float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33);
        Mat4(const Vec4 &col0, const Vec4 &col1, const Vec4 &col2, const Vec4 &col3);

        explicit Mat4(const Mat3 &upperLeft);
        Mat4(const Mat3 &upperLeft, const Vec3 &translation);

        Vec4 &operator[](int col);
        const Vec4 &operator[](int col) const;
        float *Data();
        const float *Data() const;

        Mat4 operator+(const Mat4 &other) const;
        Mat4 operator-(const Mat4 &other) const;
        Mat4 operator*(const Mat4 &other) const;
        Mat4 operator*(float scalar) const;
        Vec4 operator*(const Vec4 &v) const;

        bool operator==(const Mat4 &other) const;
        bool operator!=(const Mat4 &other) const;

        Mat4 Transposed() const;
        float Determinant() const;
        Mat4 Inverse() const;

        Mat4 InverseAffine() const;

        Mat3 UpperLeft3x3() const;
        Vec3 GetTranslation() const;

        Vec3 TransformPoint(const Vec3 &p) const;

        Vec3 TransformPointPerspective(const Vec3 &p) const;

        Vec3 TransformVector(const Vec3 &v) const;

        static Mat4 Identity();
        static Mat4 Translation(const Vec3 &t);
        static Mat4 Scale(const Vec3 &s);
        static Mat4 RotationX(float angleRad);
        static Mat4 RotationY(float angleRad);
        static Mat4 RotationZ(float angleRad);

        static Mat4 LookAt(const Vec3 &eye, const Vec3 &center, const Vec3 &up);

        static Mat4 Perspective(float fovyRad, float aspect, float near, float far);
        static Mat4 Ortho(float left, float right, float bottom, float top, float near, float far);
    };

    Mat4 operator*(float scalar, const Mat4 &m);
    std::ostream &operator<<(std::ostream &os, const Mat4 &m);

    struct alignas(16) Quaternion
    {
        union
        {
            struct
            {
                float x, y, z, w;
            };
            float v[4];
        };

        Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
        Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

        float &operator[](int index) { return v[index]; }
        const float &operator[](int index) const { return v[index]; }

        Quaternion operator+(const Quaternion &o) const { return Quaternion(x + o.x, y + o.y, z + o.z, w + o.w); }
        Quaternion operator-(const Quaternion &o) const { return Quaternion(x - o.x, y - o.y, z - o.z, w - o.w); }
        Quaternion operator*(float s) const { return Quaternion(x * s, y * s, z * s, w * s); }

        Quaternion operator*(const Quaternion &o) const
        {
            return Quaternion(
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w,
                w * o.w - x * o.x - y * o.y - z * o.z);
        }

        Vec3 operator*(const Vec3 &vec) const
        {
            Vec3 qv(x, y, z);
            Vec3 t = qv.Cross(vec) * 2.0f;
            return vec + t * w + qv.Cross(t);
        }

        Quaternion operator-() const { return Quaternion(-x, -y, -z, -w); }

        bool operator==(const Quaternion &o) const
        {
            return std::fabs(x - o.x) < EPSILON && std::fabs(y - o.y) < EPSILON &&
                   std::fabs(z - o.z) < EPSILON && std::fabs(w - o.w) < EPSILON;
        }
        bool operator!=(const Quaternion &o) const { return !(*this == o); }

        float LengthSquared() const { return x * x + y * y + z * z + w * w; }
        float Length() const { return std::sqrt(LengthSquared()); }

        Quaternion Normalized() const
        {
            float inv = 1.0f / std::sqrt(LengthSquared());
            return *this * inv;
        }
        Quaternion NormalizedSafe() const
        {
            float rawLenSq = LengthSquared();
            float lenSq = rawLenSq > EPSILON * EPSILON ? rawLenSq : EPSILON * EPSILON;
            float inv = 1.0f / std::sqrt(lenSq);
            return *this * inv;
        }
        void Normalize() { *this = Normalized(); }
        void NormalizeSafe() { *this = NormalizedSafe(); }

        Quaternion Conjugate() const { return Quaternion(-x, -y, -z, w); }
        Quaternion Inverse() const
        {
            float invLenSq = 1.0f / LengthSquared();
            return Quaternion(-x * invLenSq, -y * invLenSq, -z * invLenSq, w * invLenSq);
        }
        float Dot(const Quaternion &o) const { return x * o.x + y * o.y + z * o.z + w * o.w; }

        Mat3 ToMat3() const;
        Mat4 ToMat4() const;

        static Quaternion Identity() { return Quaternion(); }
        static Quaternion FromAxisAngle(const Vec3 &axis, float angleRad)
        {
            float half = angleRad * 0.5f;
            float s = std::sin(half);
            return Quaternion(axis.x * s, axis.y * s, axis.z * s, std::cos(half));
        }

        static Quaternion FromEulerAngles(float pitchX, float yawY, float rollZ);
        static Quaternion FromMat3(const Mat3 &m);

        static Quaternion FromTo(const Vec3 &from, const Vec3 &to);
        static Quaternion LookRotation(const Vec3 &forward, const Vec3 &up);

        static float Dot(const Quaternion &a, const Quaternion &b) { return a.Dot(b); }

        static Quaternion Lerp(const Quaternion &a, const Quaternion &b, float t)
        {
            Quaternion bb = a.Dot(b) < 0.0f ? -b : b;
            return (a * (1.0f - t) + bb * t).Normalized();
        }

        static Quaternion Slerp(const Quaternion &a, const Quaternion &b, float t);
    };

    inline Quaternion operator*(float scalar, const Quaternion &q) { return q * scalar; }
    std::ostream &operator<<(std::ostream &os, const Quaternion &q);

#ifndef MATHC_NO_EXTRA
    struct Ray
    {
        Vec3 origin;
        Vec3 direction;

        Ray();
        Ray(const Vec3 &origin, const Vec3 &direction);

        Vec3 PointAt(float t) const;

        static Ray FromPoints(const Vec3 &a, const Vec3 &b);
    };

    std::ostream &operator<<(std::ostream &os, const Ray &r);

    struct Plane
    {
        Vec3 normal;
        float d;

        Plane();
        Plane(const Vec3 &normal, float d);
        Plane(const Vec3 &normal, const Vec3 &pointOnPlane);
        Plane(const Vec3 &a, const Vec3 &b, const Vec3 &c);

        float DistanceToPoint(const Vec3 &point) const;
        Vec3 ClosestPoint(const Vec3 &point) const;
        Plane Normalized() const;

        bool IntersectRay(const Ray &ray, float &outT) const;

        static Plane FromPoints(const Vec3 &a, const Vec3 &b, const Vec3 &c);
    };

    std::ostream &operator<<(std::ostream &os, const Plane &p);

    struct Box
    {
        Vec3 min, max;

        Box();
        Box(const Vec3 &min, const Vec3 &max);

        Vec3 Center() const;
        Vec3 Extents() const; 
        Vec3 Size() const;    

        bool Contains(const Vec3 &point) const;
        bool Intersects(const Box &other) const;
        Box Union(const Box &other) const;
        Box Expanded(const Vec3 &point) const;
        Box Expanded(float amount) const;

        bool IntersectRay(const Ray &ray, float &outTMin, float &outTMax) const;

        Box Transformed(const Mat4 &m) const;

        void GetCorners(Vec3 outCorners[8]) const;

        static Box FromCenterExtents(const Vec3 &center, const Vec3 &extents);
        static Box FromPoints(const Vec3 *points, int count);
        static const Box Empty;
    };

    std::ostream &operator<<(std::ostream &os, const Box &b);

    struct Frustum
    {
        Plane planes[6];

        enum PlaneIndex { Left = 0, Right = 1, Bottom = 2, Top = 3, Near = 4, Far = 5 };

        Frustum();

        static Frustum FromViewProjection(const Mat4 &viewProj);

        bool ContainsPoint(const Vec3 &point) const;
        bool IntersectsSphere(const Vec3 &center, float radius) const;
        bool IntersectsBox(const Box &box) const;
    };
#endif 

} 

#endif 