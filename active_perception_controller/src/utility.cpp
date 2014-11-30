#include <active_perception_controller/utility.h>
#include <active_perception_controller/person_particle_filter.h>


/**  This method computes the expected information gain for a future robot pose based on entropy gain.
    Given the current belief over the person position, it is computed the entropy gain
    by moving the robot to a future pose and taking a measurement there.
  \param person_particles Current belief over the person position
  \param robot_pose Future robot pose to evaluate
  \param sensor_model Sensor model to update belief
  \return entropy_gain Expected entropy gain
*/
double Utility::computeInfoGain(sensor_msgs::PointCloud person_particles, geometry_msgs::PoseWithCovariance robot_pose, RfidSensorModel *sensor_model)
{
    // Init person_particle_filter with PointCloud = S
    int n_particles = person_particles.points.size();

    PersonParticleFilter particle_filter(n_particles);
    particle_filter.initFromParticles(person_particles);
    particle_filter.setSensorModel(sensor_model);

    /* p(z=yes) = Sum_{s_i in S}(p(s_i)*p(z=yes|s_i))
       p(z=no) = 1 - p(z=yes)
    */

    double prob_det = 0.0, prob_ndet;
    RfidSensorData rfid_obs;
    rfid_obs.rfid_ = true;
    rfid_obs.pose_ = robot_pose;

    for(int i = 0; i < n_particles; i++)
    {
        Particle *part_ptr = particle_filter.getParticle(i);
        prob_det += part_ptr->weight_*sensor_model->applySensorModel(rfid_obs, part_ptr);
    }

    prob_ndet = 1.0 - prob_det;

    /* Update S with z = yes -> S'
       Compute entropy S' = H'(z=yes)
    */
    double entropy_det, entropy_ndet;
    particle_filter.update(rfid_obs);
    entropy_det = particle_filter.entropyParticles();

    /* Update S with z = no -> S'
       Compute entropy S' = H'(z=no)
    */
    particle_filter.initFromParticles(person_particles);
    rfid_obs.rfid_ = false;
    particle_filter.update(rfid_obs);
    entropy_ndet = particle_filter.entropyParticles();

    /* Expected_H' = H'(z=yes)*p(z=yes) + H'(z=no)*p(z=no) */

    return entropy_det*prob_det + entropy_ndet*prob_ndet;
}
