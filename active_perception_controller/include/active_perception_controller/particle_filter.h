#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gsl/gsl_rng.h>
#include <vector>

using namespace std;

class SensorData;

/** \brief Information for a single particle

*/
class Particle
{
public:
    // Weight for this pose
    double weight_;
};

/**
  Class to implement a particle filter that estimate a person position
  */
class ParticleFilter
{
public:
    ParticleFilter();
    ParticleFilter(nav_msgs::OccupancyGrid const* map);
    ~ParticleFilter();

    int getNumParticles();
    void setNumParticles();
    Particle* getParticle(int particle_id);
    virtual void initUniform() = 0;
    virtual void predict(double timeStep) = 0;
    virtual void update(SensorData &obs_data){};
    virtual void resample() = 0;
    vector<int> calcResampledSet();
    void setMap(const nav_msgs::OccupancyGridConstPtr& map);

protected:
    vector<Particle*> particles_;           ///< particle set.
    nav_msgs::OccupancyGrid const *map_;
    vector<pair<int,int> > free_space_ind_; ///< Map indices with free space
    gsl_rng *ran_generator_;                ///< Random number generator
};

#endif
