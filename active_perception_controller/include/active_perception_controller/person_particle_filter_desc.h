#ifndef PERSON_PARTICLE_FILTER_H
#define PERSON_PARTICLE_FILTER_H

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <active_perception_controller/particle_filter.h>
#include <active_perception_controller/rfid_sensor_model.h>

#include <gsl/gsl_randist.h>
#include <vector>
#include <string>

using namespace std;

/** \brief Information for a single particle

*/
class PersonParticle : public Particle
{
public:
    // Pose represented by this sample
    vector<double> pose_;
    PersonParticle();
    PersonParticle(const PersonParticle &person_particle);
};

/**
  Class to implement a particle filter that estimate a person position
  */
class PersonParticleFilter : public ParticleFilter
{
public:
    PersonParticleFilter(int n_particles);
    PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose, double rfid_map_res, string rfid_prob_map);
    ~PersonParticleFilter();

    void initUniform();
    void predict(double timeStep);
    void update(SensorData &obs_data);
    static void update(RfidSensorModel &rfid_model,
                       vector<Particle*> &particles,
                       SensorData &obs,
                       const vector<double>& prev_weights,
                       vector<double>& updated_weights,
                       const vector<size_t>& use_particle_idx);
    void resample();
    void setSensorModel(RfidSensorModel *model);
    double entropyParticles();
    static double entropyParticles(RfidSensorModel &rfid_model,
                                   vector<Particle*> &particles,
                                   SensorData &new_obs,
                                   const vector<double>& prev_weights,
                                   const vector<double>& current_weights);
    double entropyGMM();
    static double entropyGMM(const vector<double>& current_weights,
                             double sigma_pose);

    void initFromParticles(const sensor_msgs::PointCloud &particle_set);
    void join_sets(const sensor_msgs::PointCloud &particle_set);
    void divide_1to1(double ow, double nw, double own_pos[2], double neighbor_pos[2]);
    void divide_cv(double la, double oa, double ra, double left_pos[2], double right_pos[2]);

protected:
    double sigma_pose_;                 ///< Starndard deviation for person movement
    RfidSensorModel *rfid_model_;       ///< Probability model for RFID observations
    bool local_sensor_;                 ///< True if the sensor model is created locally
    bool external_sensor_;              ///< True if an external sensor model is loaded
    vector<double> prev_weights_;       ///< Weights of previous step
    SensorData *last_obs_;              ///< Data from last observation updated
    bool prev_step_info_;               ///< True if there is information from previous weights
};


#endif
