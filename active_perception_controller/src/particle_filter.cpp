#include "particle_filter.h"

/**
  Constructor
  */
Particle::Particle()
{
    pose[0] = 0.0;
    pose[1] = 0.0;
    weight = 0.0;
}

/** Default constructor
  */
ParticleFilter::ParticleFilter()
{
    num_particles = 0;
}

/** Constructor
  */
ParticleFilter::ParticleFilter(int n_particles)
{
    num_particles = n_particles;

    for(int i = 0; i < num_particles; i++)
    {
        particles[i].weight = 0.0;
        particles[i].pose[0] = 0.0;
        particles[i].pose[1] = 0.0;
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
    return num_particles;
}

/** Get pose of a particle
  \param particle_id Identifier of the particle
  */
vector<double> ParticleFilter::getParticlePose(int particle_id)
{
    return particles[particle_id].pose;
}

/** Draw particles from a uniform distribution
  */
void ParticleFilter::initUniform()
{

}

/** Predict particles
  \param timestep Prediction step duration in seconds
  */
void ParticleFilter::predict(double timeStep)
{

}

/** Update particles with new RFID measure
  \param mes RFID measure
  \param robot_x Current robot pose
  \param robot_y Current robot pose
  */
void ParticleFilter::update(bool &rfid_mes, double &robot_x, double &robot_y)
{

}

/** Update particles with new RFID measure
  \param mes RFID measure
  \param robot_cloud Current robot pose cloud
  */
void ParticleFilter::update(bool &rfid_mes, geometry_msgs::PoseArray &robot_cloud)
{

}



