#include <active_perception_controller/utility.h>
#include <active_perception_controller/person_particle_filter.h>
#include <active_perception_controller/rfid_sensor_model.h>


/**  This method computes the expected information gain for a future robot pose based on entropy gain.
    Given the current belief over the person position, it is computed the entropy gain
    by moving the robot to a future pose and taking a measurement there.
  \param person_particles Current belief over the person position
  \param robot_pose Future robot pose to evaluate
  \return entropy_gain Expected entropy gain
*/
double Utility::computeInfoGain(sensor_msgs::PointCloud person_particles, geometry_msgs::PoseWithCovariance robot_pose)
{
    double entropy_gain;

    /* EXpected_H' = H'(z=yes)*p(z=yes) + H'(z=no)*p(z=no)
      Init person_particle_filter with PointCloud = S

      p(z=yes) = Sum_{s_i in S}(p(s_i)*p(z=yes|s_i))

      p(z=no) = 1 - p(z=yes)

      Update S with z = yes -> S'
      Compute entropy S' = H'(z=yes)
      Update S with z = no -> S'
      Compute entropy S' = H'(z=no)


      */
    return entropy_gain;
}
