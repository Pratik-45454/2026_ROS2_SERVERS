#include "state_msg.hpp"

Quaternion toQuaternion(const Euler &ypr)
{
    Quaternion q;

    double cr = cos(ypr.roll * 0.5);
    double sr = sin(ypr.roll * 0.5);
    double cp = cos(ypr.pitch * 0.5);
    double sp = sin(ypr.pitch * 0.5);
    double cy = cos(ypr.yaw * 0.5);
    double sy = sin(ypr.yaw * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

Euler toEuler(const Quaternion &q)
{
    Euler ypr;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    ypr.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    ypr.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ypr.yaw = atan2(siny_cosp, cosy_cosp);

    return ypr;
}