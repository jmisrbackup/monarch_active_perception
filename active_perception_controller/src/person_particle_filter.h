#ifndef PERSON_PARTICLE_FILTER_H
#define PERSON_PARTICLE_FILTER_H

#include "geometry_msgs/PoseArray.h"
#include "particle_filter.h"

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <vector>

using namespace std;

/** \brief Information for a single particle

*/
class PersonParticle : public Particle
{
public:
    // Pose represented by this sample
    vector<double> pose_;
    PersonParticle();
};

/**
  Class to implement a particle filter that estimate a person position
  */
class PersonParticleFilter : public ParticleFilter
{
public:
    PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose, double d_threshold, double prob_positive_det, double prob_false_det);
    ~PersonParticleFilter();

    void initUniform();
    void predict(double timeStep);
    void update(bool &rfid_mes, double &robot_x, double &robot_y, double &robot_x_cov, double &robot_y_cov);

protected:
    gsl_rng *ran_generator_;        ///< Random number generator
    double sigma_pose_;             ///< Starndard deviation for person movement
    double d_threshold_;            ///< Distance threshold to detect RFID tag
    double prob_positive_det_;      ///< Detection probability within range
    double prob_false_det_;         ///< Detection probability out of range

    double computeObsProb(bool &rfid_mes, double x_r, double y_r, double x_r_cov, double y_r_cov, double x_e, double y_e);
};


#endif
