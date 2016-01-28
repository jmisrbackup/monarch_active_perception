#include <active_perception_controller/particle_filter.h>

/** Default constructor
  */
ParticleFilter::
ParticleFilter
()
{
    map_ = NULL;
    ran_generator_ = gsl_rng_alloc(gsl_rng_taus);
}

ParticleFilter::
ParticleFilter
(const nav_msgs::OccupancyGrid *map)
{
    map_ = map;

    // Compute free space in map
    free_space_ind_.resize(0);
    for(int i = 0; i < map_->info.width; i++)
    {
        for(int j = 0; j < map_->info.height; j++)
        {
            if(map_->data[j*map_->info.width + i] == 0)
                free_space_ind_.push_back(std::make_pair(i,j));
        }
    }

    ran_generator_ = gsl_rng_alloc(gsl_rng_taus);
}

/** Destructor
  */
ParticleFilter::~ParticleFilter()
{
    gsl_rng_free(ran_generator_);
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

/**  Method to resample particles according to the current distribution. The method is based on
  the low-variance technique. The method determines which particles from the original set will
  be used after resampling.
  \return resampled_set Vector with the particles index to resample
*/
vector<int> ParticleFilter::calcResampledSet()
{
    vector<int> resampled_set;
    double c, u, r;
    int num_samples = particles_.size();

    double total_weight=0.0;
    for (int i=0; i<num_samples;i++)
	total_weight=total_weight+particles_[i]->weight_;
   

    if(num_samples > 0)
    {
        r = gsl_rng_uniform(ran_generator_)*(total_weight/num_samples);

        int m, i;

        i = 0;
        c = particles_[0]->weight_;
        for (int m = 0; m < num_samples; m++)
        {
          u = r + (m) * (total_weight / num_samples);
          while (u > c)
          {
            i = i + 1;
            c = c + particles_[i]->weight_;
          }
          resampled_set.push_back(i);
        }
    }

    return resampled_set;
}

/** Set an occupancy map for the filter
  \param map New map
  */
void ParticleFilter::setMap(const nav_msgs::OccupancyGrid *map)
{
    map_ = map;
}
