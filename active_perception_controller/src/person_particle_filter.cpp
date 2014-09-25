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
  */
PersonParticleFilter::PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose):ParticleFilter(map)
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }

    ran_generator_ = gsl_rng_alloc(gsl_rng_taus);
    sigma_pose_ = sigma_pose;
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
  \param mes RFID measure
  \param robot_x Current robot pose
  \param robot_y Current robot pose
  */
void PersonParticleFilter::update(bool &rfid_mes, double &robot_x, double &robot_y)
{

}

/** Update particles with new RFID measure
  \param mes RFID measure
  \param robot_cloud Current robot pose cloud
  */
void PersonParticleFilter::update(bool &rfid_mes, geometry_msgs::PoseArray &robot_cloud)
{

}



