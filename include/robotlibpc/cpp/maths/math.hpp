#ifndef _ROBOTLIB_MATH_HPP
#define _ROBOTLIB_MATH_HPP

#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f
#define F32_2_PI 6.28318530717958f

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232

template <typename t>
inline t map(t x, t in_min, t in_max, t out_min, t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename t>
inline t map(t x, t out_min, t out_max)
{
  return map(x, 0, 1, out_min, out_max);
}

template <typename t>
t clamp(t x, t outmin, t outmax)
{
  if (x > outmax)
    return outmax;
  else if (x < outmin)
    return outmin;
  return x;
}

template <typename t>
inline t trim(t x, t a, t b, t c)
{
  if (x > a && x <= b)
    return (x - a) / (b - a);
  else if (x > b && x <= c)
    return (c - x) / (c - b);
  return 0;
}

template <typename t>
inline t invTrim(t x, t a, t b, t c)
{
  t out1, out2;
  out1 = (b - a) * x + a;
  out2 = c - (c - b) * x;
  return (out1 + out2) / 2;
}

inline float angleChange(const float curr, const float prev)
{
  float change = curr - prev;
  if (change > F32_PI)
  {
    change -= F32_2_PI;
  }
  else if (change < -F32_PI)
  {
    change += F32_2_PI;
  }
  return change;
}

inline float angleClamp(float angle)
{
  if (angle > F32_PI)
  {
    angle -= F32_2_PI;
  }
  else if (angle < (-F32_PI))
  {
    angle += F32_2_PI;
  }
  return angle;
}

inline float rad2Deg(float rad)
{
  return (rad * 180.0f / F32_PI);
}

inline float deg2Rad(float rad)
{
  return (rad * F32_PI / 180.0f);
}

template <typename T = float>
struct Vector2
{
  T x;
  T y;
};

template <typename T = float>
struct Vector3
{
  T x;
  T y;
  T z;
};

template <typename T = float>
struct Vector4
{
  T w;
  T x;
  T y;
  T z;
};

#endif // _ROBOTLIB_MATH_HPP
