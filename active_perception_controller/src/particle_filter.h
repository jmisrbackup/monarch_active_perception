#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"
#include <vector>

using namespace std;

/** \brief Information for a single particle

*/
class Particle
{
public:
    // Pose represented by this sample
    vector<double> pose;
    // Weight for this pose
    double weight;
    Particle();
};

/**
  Class to implement a particle filter that estimate a person position
  */
class ParticleFilter
{
public:
    ParticleFilter();
    ParticleFilter(int n_particles);
    ~ParticleFilter();

    int getNumParticles();
    vector<double> getParticlePose(int particle_id);
    void initUniform();
    void predict(double timeStep);
    void update(bool &rfid_mes, double &robot_x, double &robot_y);
    void update(bool &rfid_mes, geometry_msgs::PoseArray &robot_cloud);

protected:

    int num_particles;
    vector<Particle> particles;     ///< particle set.
};


#endif
