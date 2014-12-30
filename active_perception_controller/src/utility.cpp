#include <active_perception_controller/utility.h>
#include <active_perception_controller/person_particle_filter.h>
#include <ros/serialization.h>

Utility::Utility(std::string prob_image_path,
                 float resolution) :
sensor_model_(new RfidSensorModel(prob_image_path, resolution))
{}

void Utility::setPersonParticles(const std::string& serialized_particles)
{
    /*This is taken from the ROS python_bindings_tutorial*/
    size_t serial_size = serialized_particles.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    for (size_t i = 0; i < serial_size; ++i)
    {
      buffer[i] = serialized_particles[i];
    }
    ros::serialization::IStream stream(buffer.get(), serial_size);
    ros::serialization::Serializer<sensor_msgs::PointCloud>::read(stream, person_particles_);
    cout <<"read " << person_particles_.points.size() << " particles" << endl;

}

/**  This function computes the expected information gain for a future robot pose based on entropy gain.
    Given the current belief over the person position, it is computed the entropy gain
    by moving the robot to a future pose and taking a measurement there.
  \param person_particles Current belief over the person position
  \param robot_pose Future robot pose to evaluate
  \param sensor_model Sensor model to update belief
  \return entropy_gain Expected entropy gain
*/
double Utility::computeInfoGain(float px,
                                float py)
{
    geometry_msgs::PoseWithCovariance robot_pose;
    robot_pose.pose.position.x = px;
    robot_pose.pose.position.y = py;
    // Init person_particle_filter with PointCloud = S
    int n_particles = person_particles_.points.size();

    PersonParticleFilter particle_filter(n_particles);
    particle_filter.initFromParticles(person_particles_);
    particle_filter.setSensorModel(sensor_model_.get());

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
        prob_det += part_ptr->weight_*sensor_model_->applySensorModel(rfid_obs, part_ptr);
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
    particle_filter.initFromParticles(person_particles_);
    rfid_obs.rfid_ = false;
    particle_filter.update(rfid_obs);
    entropy_ndet = particle_filter.entropyParticles();

    /* Expected_H' = H'(z=yes)*p(z=yes) + H'(z=no)*p(z=no) */

    return entropy_det*prob_det + entropy_ndet*prob_ndet;
}

