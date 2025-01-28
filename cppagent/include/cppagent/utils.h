#ifndef UTILS_H
#define UTILS_H

#include "geometry_msgs/msg/quaternion.hpp"

struct EulerAngles
{
    double roll;
    double pitch;
    double yaw;
};

EulerAngles quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion);

#endif // UTILS_H