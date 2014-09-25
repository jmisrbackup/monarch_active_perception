#include "particle_filter.h"

/** Default constructor
  */
ParticleFilter::
ParticleFilter
()
{}

ParticleFilter::
ParticleFilter
(nav_msgs::OccupancyGrid const *map)
{
    map_ = map;

    // Compute free space in map
    free_space_ind_.resize(0);
    for(int i = 0; i < map_->info.width; i++)
    {
        for(int j = 0; j < map_->info.height; j++)
        {
            if(map_->data[i*map_->info.width + j] == 0)
                free_space_ind_.push_back(std::make_pair(i,j));
        }
    }
}

/** Destructor
  */
ParticleFilter::~ParticleFilter()
{
}

/** Get number of particles
  */
int ParticleFilter::getNumParticles()
{
    return particles_.size();
}

/** Get a particle
  \param particle_id Identifier of the particle
  */
Particle* ParticleFilter::getParticle(int particle_id)
{
    try
    {
        return particles_.at(particle_id);
    }
    catch(const std::out_of_range& oor)
    {
        ROS_ERROR_STREAM("Particle index out of range!");
        return 0;
    }
}
