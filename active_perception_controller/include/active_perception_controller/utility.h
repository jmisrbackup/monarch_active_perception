#ifndef UTILITY_H
#define UTILITY_H

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovariance.h>


/** This class implements functions to compute utilities based on information gain.

*/
class Utility
{
public:
    static double computeInfoGain(sensor_msgs::PointCloud person_particles, geometry_msgs::PoseWithCovariance robot_pose);
};

#endif
