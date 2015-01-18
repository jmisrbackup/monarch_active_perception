#include <active_perception_controller/utility.h>
#include <active_perception_controller/person_particle_filter.h>
#include <tf/transform_datatypes.h>
#include <ros/serialization.h>

/**
 * \brief Constructor
 * \param prob_image_path Image with sensor model
 * \param resolution Image resolution for sensor model
 * \param sigma_pose Standard deviation for person movement
 */
Utility::Utility(std::string prob_image_path,
                 float resolution, double sigma_pose) :
sensor_model_(new RfidSensorModel(prob_image_path, resolution))
{
    sigma_pose_ = sigma_pose;
}

/** \brief Setting values for the particles
 * \param serialized_particles New particles serialized
 */
void Utility::setPersonParticles(const std::string& serialized_particles)
{
    sensor_msgs::PointCloud pc;
    /*This is taken from the ROS python_bindings_tutorial*/
    size_t serial_size = serialized_particles.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    for (size_t i = 0; i < serial_size; ++i)
    {
        buffer[i] = serialized_particles[i];
    }
    ros::serialization::IStream stream(buffer.get(), serial_size);
    ros::serialization::Serializer<sensor_msgs::PointCloud>::read(stream, pc);

    size_t num_particles = pc.points.size();

    if(person_particles_.size() != num_particles)
    {
        person_particles_.resize(num_particles, 0);
    }

    // Look for the channel with weights
    int weights_channel = 0;
    while(pc.channels[weights_channel].name != "weights")
    {
        weights_channel++;
        if(weights_channel >= pc.channels.size())
        {
            ROS_ERROR("PersonParticleFilter: Particle set must have a 'weights' channel");
            return;
        }
    }

    // Copy particles from particle set
    for(int i = 0; i < num_particles; i++)
    {
        if(person_particles_[i] != 0)
            free(person_particles_[i]);

        person_particles_[i] = (Particle*) (new PersonParticle());
        PersonParticle * part_ptr = (PersonParticle *)(person_particles_[i]);
        part_ptr->pose_[0] = pc.points[i].x;
        part_ptr->pose_[1] = pc.points[i].y;
        part_ptr->weight_ = pc.channels[weights_channel].values[i];
    }
}

/**  This function computes the expected information gain for a future robot pose based on entropy gain.
    Given the current belief over the person position, it is computed the entropy gain
    by moving the robot to a future pose and taking a measurement there.
  \param px Future robot pose to evaluate
  \param py Future robot pose to evaluate
  \param yaw Future robot yaw to evaluate
  \param prev_weights Particle weights for current position
  \param updated_weights Particle weights after updating in future position. Expected values are computed
  \return entropy_gain Expected entropy gain
*/
double Utility::computeInfoGain(float px,
                                float py,
                                float yaw,
                                vector<double>& prev_weights,
                                vector<double>& updated_weights)
{
    geometry_msgs::PoseWithCovariance robot_pose;
    robot_pose.pose.position.x = px;
    robot_pose.pose.position.y = py;
    robot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    prev_weights.resize(person_particles_.size(),0);
    updated_weights.resize(person_particles_.size(),0);
    vector<double> det_weights(person_particles_.size(),0);
    vector<double> ndet_weights(person_particles_.size(),0);

    double prob_det = 0.0, prob_ndet;
    RfidSensorData rfid_obs;
    rfid_obs.rfid_ = true;
    rfid_obs.pose_ = robot_pose;

    /* p(z=yes) = Sum_{s_i in S}(p(s_i)*p(z=yes|s_i))
       p(z=no) = 1 - p(z=yes)
    */

    for(int i = 0; i < person_particles_.size(); i++)
    {
        Particle *part_ptr = person_particles_[i];
        prob_det += prev_weights[i]*sensor_model_->applySensorModel(rfid_obs, part_ptr);
    }

    prob_ndet = 1.0 - prob_det;

    /* Update S with z = yes -> S'
       Compute entropy S' = H'(z=yes)
    */
    double entropy_det, entropy_ndet;

    PersonParticleFilter::update(*(sensor_model_.get()),
                                 person_particles_,
                                 rfid_obs,
                                 prev_weights,
                                 det_weights);

    entropy_det = PersonParticleFilter::entropyParticles(*(sensor_model_.get()),
                                                         person_particles_,
                                                         rfid_obs,
                                                         prev_weights,
                                                         det_weights);

    // Alternative method
    // entropy_det = PersonParticleFilter::entropyGMM(det_weights, sigma_pose_);


    /* Update S with z = no -> S'
       Compute entropy S' = H'(z=no)
    */
    rfid_obs.rfid_ = false;

    PersonParticleFilter::update(*(sensor_model_.get()),
                                 person_particles_,
                                 rfid_obs,
                                 prev_weights,
                                 ndet_weights);

    entropy_ndet = PersonParticleFilter::entropyParticles(*(sensor_model_.get()),
                                                          person_particles_,
                                                          rfid_obs,
                                                          prev_weights,
                                                          ndet_weights);

    // Alternative method
    // entropy_ndet = PersonParticleFilter::entropyGMM(ndet_weights, sigma_pose_);


    /* Expected_H' = H'(z=yes)*p(z=yes) + H'(z=no)*p(z=no) */
    ROS_INFO_STREAM("pdet: " << prob_det << " entropy det " << entropy_det << " prob_ndet: " << prob_ndet << " entropy ndet " << entropy_ndet);

    for(size_t i = 0; i < updated_weights.size(); i++)
        updated_weights[i] = prob_det*det_weights[i] + prob_ndet*ndet_weights[i];

    return entropy_det*prob_det + entropy_ndet*prob_ndet;
}
