#include "person_particle_filter.h"

/**
  Constructor
  */
PersonParticle::PersonParticle()
{
    pose_.push_back(0.0);
    pose_.push_back(0.0);
    weight_ = 0.0;
}

/** Default constructor
  */
PersonParticleFilter::PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid& map):ParticleFilter(map)
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }
}

/** Draw particles from a uniform distribution
  */
void PersonParticleFilter::initUniform()
{

}

/** Predict particles
  \param timestep Prediction step duration in seconds
  */
void PersonParticleFilter::predict(double timeStep)
{

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



