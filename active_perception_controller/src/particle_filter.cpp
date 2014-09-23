#include "particle_filter.h"

/** Default constructor
  */
ParticleFilter::
ParticleFilter
()
{}

ParticleFilter::
ParticleFilter
(const nav_msgs::OccupancyGridConstPtr& map)
{
    map_ = map;
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
