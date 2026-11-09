#ifndef _STATE_MSG_HPP
#define _STATE_MSG_HPP

#include <math.h>

struct Twist
{
    float vx = 0.0f;
    float vy = 0.0f;
    float w = 0.0f;

    Twist(float _vx, float _vy, float _w)
        : vx(_vx), vy(_vy), w(_w) {}

    Twist() = default;

    static Twist from_v_theta_omega(const float v, const float theta, const float omega)
    {
        return Twist(v * std::cos(theta), v * std::sin(theta), omega);
    }

    float get_theta() const
    {
        return atan2(vy, vx);
    }

    void set_from_v_theta_omega(const float v, const float theta, const float omega)
    {
        vx = v * std::cos(theta);
        vy = v * std::sin(theta);
        w = omega;
    }
};

struct Position
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

struct RobotState
{
    Position pose;
    Twist twist;
};

struct Euler
{
    double yaw;
    double pitch;
    double roll;
};

struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

Quaternion toQuaternion(const Euler &ypr);

Euler toEuler(const Quaternion &q);

#endif // _STATE_MSG_HPP