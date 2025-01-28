#include "cppagent/utils.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


EulerAngles quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return {roll, pitch, yaw};
}