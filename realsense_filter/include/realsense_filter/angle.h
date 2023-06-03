#include <math.h>

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct EulerAngles
{
    float roll;
    float pitch;
    float yaw;
};

EulerAngles quaternionToEulerAngles(const Quaternion &q)
{
    EulerAngles angles;

    float sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0)
        angles.pitch = std::copysign(M_PI / 2.0, sinp);
    else
        angles.pitch = std::asin(sinp);

    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

float toRAD(float deg)
{
  return deg * M_PI / 180;
}

float toDEG(float rad)
{
  return rad * 180 / M_PI;
}