#include "person_particle_filter.h"
#include <cstdlib>

/**
  Constructor
  */
PersonParticle::PersonParticle()
{
    pose_.push_back(0.0);
    pose_.push_back(0.0);
    weight_ = 0.0;
}

/** Constructor
   \param n_particles Number of particles
   \param map Occupancy map
   \param sigma_pose Standard deviation of person movement noise
   \param d_threshold Distance threshold to detect RFID tag
   \param prob_positive_det Detection probability within range
   \param prob_false_det Detection probability out of range
  */
PersonParticleFilter::PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose, double d_threshold, double prob_positive_det, double prob_false_det):ParticleFilter(map)
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }

    ran_generator_ = gsl_rng_alloc(gsl_rng_taus);
    sigma_pose_ = sigma_pose;
    d_threshold_ = d_threshold;
    prob_positive_det_ = prob_positive_det;
    prob_false_det_ = prob_false_det;
}

/** Destructor
  */
PersonParticleFilter::~PersonParticleFilter()
{
    gsl_rng_free(ran_generator_);
}

/** Draw particles from a uniform distribution
  */
void PersonParticleFilter::initUniform()
{
    // Draw only particles from free space
    int num_particles = particles_.size();

    for(int i = 0; i < num_particles; i++)
    {
        unsigned int rand_index = gsl_rng_uniform(ran_generator_)* free_space_ind_.size();
        std::pair<int,int> free_point = free_space_ind_[rand_index];

        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);

        part_ptr->pose_[0] = map_->info.origin.position.x + map_->info.resolution * free_point.first;
        part_ptr->pose_[1] = map_->info.origin.position.y + map_->info.resolution * (map_->info.height - free_point.second);
    }
}

/** Predict particles
  \param timestep Prediction step duration in seconds
  */
void PersonParticleFilter::predict(double timeStep)
{
    int num_particles = particles_.size();

    for(int i = 0; i < num_particles; i++)
    {
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->pose_[0] += gsl_ran_gaussian(ran_generator_, sigma_pose_);
        part_ptr->pose_[1] += gsl_ran_gaussian(ran_generator_, sigma_pose_);
    }
}

/** Update particles with new RFID measure
  \param rfid_mes RFID measure
  \param robot_x Current robot pose
  \param robot_y Current robot pose
  \param robot_x_cov Covariance
  \param robot_y_cov Covariance
  */
void PersonParticleFilter::update(bool &rfid_mes, double &robot_x, double &robot_y, double &robot_x_cov, double &robot_y_cov)
{
    double total_weight = 0.0;

    // Update weights
    for(int i = 0; i < particles_.size(); i++)
    {
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->weight_ = computeObsProb(rfid_mes, robot_x, robot_y, robot_x_cov, robot_y_cov, part_ptr->pose_[0], part_ptr->pose_[1]);
        total_weight += part_ptr->weight_;
    }

    // Normalize weights
    for(int i = 0; i < particles_.size(); i++)
    {
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->weight_ = part_ptr->weight_/total_weight;
    }
}

/** Compute the probability of obtaining an RFID measure given a reader and emitter
  \param rfid_mes RFID measure
  \param x_r Reader position
  \param y_r Reader position
  \param x_r_cov Covariance in robot position
  \param y_r_cov Covariance in robot position
  \param x_e Emitter position
  \param y_e Emitter position
  */
double PersonParticleFilter::computeObsProb(bool &rfid_mes, double x_r, double y_r, double x_r_cov, double y_r_cov, double x_e, double y_e)
{
    double obs_prob;
    double distance = sqrt(((x_r-x_e)*(x_r-x_e)) + ((y_r-y_e)*(y_r-y_e)));

    if(distance <= d_threshold_)
    {
        if(rfid_mes)
            obs_prob = prob_positive_det_;
        else
            obs_prob = 1.0 - prob_positive_det_;
    }
    else
    {
        if(rfid_mes)
            obs_prob = prob_false_det_;
        else
            obs_prob = 1.0 - prob_false_det_;
    }
    return obs_prob;
}



