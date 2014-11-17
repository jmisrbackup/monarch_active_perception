#include <active_perception_controller/utility.h>


/**  This method computed the expected entropy gain for a future robot pose.
    Given the current belief over the person position, it is computed the entropy gain
    by moving the robot to a future pose and taking a measurement there.
  \param person_particles Current belief over the person position
  \param robot_pose Future robot pose to evaluate
  \return entropy_gain Expected entropy gain
*/
static double Utility::computeEntropyGain(sensor_msgs::PointCloud person_particles, geometry_msgs::PoseWithCovariance robot_pose)
{
    double entropy_gain;

    return entropy_gain;
}
