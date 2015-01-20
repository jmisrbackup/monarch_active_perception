#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>

#include <active_perception_controller/person_particle_filter.h>
#include <active_perception_controller/sensor_model.h>

#define FILTER_ON_PREDICTION 0

/**
  Constructor
  */
PersonParticle::PersonParticle()
{
    pose_.resize(2,0.0);
    weight_ = 0.0;
}

/** Copy constructor
  */
PersonParticle::PersonParticle(const PersonParticle &person_particle)
{
    pose_.clear();
    pose_ = person_particle.pose_;
    weight_ = person_particle.weight_;
}

/** Constructor
   \param n_particles Number of particles
  */
PersonParticleFilter::PersonParticleFilter(int n_particles):ParticleFilter()
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }
    local_sensor_ = false;
    external_sensor_ = false;
    prev_step_info_ = false;
    prev_weights_.resize(n_particles,0.0);
    last_obs_ = new RfidSensorData();
}

/** Constructor
   \param n_particles Number of particles
   \param map Occupancy map
   \param sigma_pose Standard deviation of person movement noise
   \param rfid_map_res Resolution of RFID maps
   \param rfid_prob_map Probability map for positive RFID
  */
PersonParticleFilter::PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose, double rfid_map_res, string rfid_prob_map):ParticleFilter(map)
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }

    sigma_pose_ = sigma_pose;

    rfid_model_ = new RfidSensorModel(rfid_prob_map, rfid_map_res);
    local_sensor_ = true;
    external_sensor_ = false;
    prev_step_info_ = false;
    prev_weights_.resize(n_particles,0.0);
    last_obs_ = new RfidSensorData();
}

/** Destructor
  */
PersonParticleFilter::~PersonParticleFilter()
{
    for(int i = 0; i < particles_.size(); i++)
    {
        delete ((PersonParticle *)(particles_[i]));
    }

    if(local_sensor_)
        delete rfid_model_;

    delete last_obs_;
}

/** Draw particles from a uniform distribution
  */
void PersonParticleFilter::initUniform()
{
    if(map_ != NULL)
    {
        // Draw only particles from free space
        int num_particles = particles_.size();

        for(int i = 0; i < num_particles; i++)
        {
            unsigned int rand_index = gsl_rng_uniform(ran_generator_)* free_space_ind_.size();
            std::pair<int,int> free_point = free_space_ind_[rand_index];

            PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);

            part_ptr->pose_[0] = map_->info.origin.position.x + map_->info.resolution * free_point.first;
            part_ptr->pose_[1] = - (map_->info.origin.position.y + map_->info.resolution * (map_->info.height - free_point.second));
            part_ptr->weight_ = 1.0/num_particles;
        }
    }
    else
    {
        ROS_ERROR("Particle filter cannot be initialized as a uniform without a map");
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

        double dx = gsl_ran_gaussian(ran_generator_, sigma_pose_);
        double dy = gsl_ran_gaussian(ran_generator_, sigma_pose_);
#ifdef FILTER_ON_PREDICTION
        if(map_ != NULL)
        {
            size_t map_x = floor((part_ptr->pose_[0] + dx - map_->info.origin.position.x)/map_->info.resolution);
            size_t map_y = floor((part_ptr->pose_[1] + dy + map_->info.origin.position.y)/map_->info.resolution + map_->info.height);
            if(map_->data[map_y*map_->info.width + map_x] != 0) //occupied cell
            {
                dx = 0;
                dy = 0;
            }
        }
        else
        {
            ROS_WARN("Prediction cannot be filtered out without a map");
        }
#endif

        part_ptr->pose_[0] += dx;
        part_ptr->pose_[1] += dy;
    }
}

/** Update particles with new observation
  \param obs_data Observation
  */
void PersonParticleFilter::update(SensorData &obs_data)
{
    double total_weight = 0.0;

    if(local_sensor_ || external_sensor_)
    {
        // Update weights. We assume all observations are from RFID sensor
        for(int i = 0; i < particles_.size(); i++)
        {
            // Save previous information
            prev_weights_[i] = particles_[i]->weight_;

            particles_[i]->weight_ = particles_[i]->weight_ * rfid_model_->applySensorModel(obs_data, particles_[i]);
            total_weight += particles_[i]->weight_;
        }

        if(total_weight > 0)
        {
        // Normalize weights
            for(int i = 0; i < particles_.size(); i++)
            {
               particles_[i]->weight_ = particles_[i]->weight_/total_weight;
            }
        }
        else
        {
            for(int i = 0; i < particles_.size(); i++)
            {
                ROS_WARN("PersonParticleFilter:: All particles have 0 weight. Re-normalizing.");
                particles_[i]->weight_ = 1.0/particles_.size();
            }
        }
        RfidSensorData *curr_obs = (RfidSensorData *)&obs_data;
        RfidSensorData *prev_obs = (RfidSensorData *)last_obs_;
        *prev_obs = *curr_obs;

        prev_step_info_ = true;
    }
    else
        ROS_ERROR("Filter has no sensor model to update");
}

/** \brief Static version of the update function
\param rfid_model Sensor model
\param particles Positions of current particles
\param obs_data Observation to update
\param prev_weights Particle weights before updating
\param updated_weights Particle weights after updating
*/
void PersonParticleFilter::update(RfidSensorModel &rfid_model,
                                  vector<Particle*> &particles,
                                  SensorData &obs_data,
                                  const vector<double>& prev_weights,
                                  vector<double>& updated_weights)
{
    double total_weight = 0.0;

    // Update weights. We assume all observations are from RFID sensor
    for(int i = 0; i < particles.size(); i++)
    {
        updated_weights[i] = prev_weights[i]*rfid_model.applySensorModel(obs_data, particles[i]);
        total_weight += updated_weights[i];
    }

    if(total_weight > 0)
    {
    // Normalize weights
        for(int i = 0; i < updated_weights.size(); i++)
        {
            updated_weights[i] = updated_weights[i]/total_weight;
        }
    }
    else
    {
        for(int i = 0; i < particles.size(); i++)
        {
            ROS_WARN("PersonParticleFilter:: All particles have 0 weight. Re-normalizing.");
            updated_weights[i] = 1.0/particles.size();
        }
    }
}

/** Resample the current set of particles according to the probability distribution
  */
void PersonParticleFilter::resample()
{
    vector<int> resampled_set = calcResampledSet();
    vector<PersonParticle*> resampled_particles;

    // Create new set
    for(int i = 0; i < resampled_set.size(); i++)
    {
        resampled_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[resampled_set[i]]))));
    }

    // Free old set
    for(int i = 0; i < particles_.size(); i++)
    {
        delete ((PersonParticle *)(particles_[i]));
    }
    particles_.clear();

    // Replace with new set
    for(int i = 0; i < resampled_particles.size(); i++)
    {
        particles_.push_back((Particle *)(resampled_particles[i]));
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->weight_ = 1.0/resampled_particles.size();
    }

    prev_step_info_ = false;
}

/**
  Set external sensor model for particle filter
  \param model New model
  */
void PersonParticleFilter::setSensorModel(RfidSensorModel *model)
{
    if(local_sensor_)
    {
        delete rfid_model_;
        local_sensor_ = false;
    }

    external_sensor_ = true;
    rfid_model_ = model;
}

/** \brief Compute entropy of probability distribution as an approximation from the particles.
    Eq. 14 from paper "Particle Filter Based Entropy".
    By Boers, Driessen, Bagchi and Mandal.
  */
double PersonParticleFilter::entropyParticles()
{
    double entropy = 0.0;

    if(prev_step_info_)
    {
        if(local_sensor_ || external_sensor_)
        {
            double obs_prob;
            double first_term = 0.0;
            double second_term = 0.0;

            /*
              entropy = log{ Sum_{s_i in S}( p(s_i(k-1)) * p(z(k))|s_i(k)) )} -
              Sum{s_i in S}( log{ p(z(k)|s_i(k)) } * p(s_i(k)) )

              When p(s_i(k) | Z(k)) = 0 for all i, the entropy converges to 0
              (see the original derivation and remember that lim_{x->0} x log(x) = 0)
              */

            for(int i = 0; i < particles_.size(); i++)
            {
                obs_prob = rfid_model_->applySensorModel(*last_obs_, particles_[i]);
                if(obs_prob > 0)
                {
                    first_term += obs_prob*prev_weights_[i];
                    second_term += log(obs_prob)*particles_[i]->weight_;
                }
            }

            if(first_term == 0)
                entropy = 0;
            else
                entropy = log(first_term) - second_term;
        }
        else
            ROS_ERROR("Entropy cannot be computed without a sensor model");
    }
    else
        ROS_ERROR("Entropy cannot be computed without information from previous step");

    return entropy;
}

/** \brief Static version of the entropy calculation function (for performance reasons)
\param rfid_model Sensor model to update
\param particles Current particles' positions
\param obs Last observation
\param prev_weights Particle weights before updating
\param current_weights Particle weights after updating
*/
double
PersonParticleFilter::
entropyParticles(RfidSensorModel &rfid_model,
                 vector<Particle*> &particles,
                 SensorData &obs,
                 const vector<double>& prev_weights,
                 const vector<double>& current_weights)
{
    double entropy = 0.0;

    double obs_prob;
    double first_term = 0.0;
    double second_term = 0.0;

    /*
      entropy = log{ Sum_{s_i in S}( p(s_i(k-1)) * p(z(k))|s_i(k)) )} -
      Sum{s_i in S}( log{ p(z(k)|s_i(k)) } * p(s_i(k)) )

      When p(s_i(k) | Z(k)) = 0 for all i, the entropy converges to 0
      (see the original derivation and remember that lim_{x->0} x log(x) = 0)
      */

    for(int i = 0; i < particles.size(); i++)
    {
        obs_prob = rfid_model.applySensorModel(obs, particles[i]);
        if(obs_prob > 0)
        {
            first_term += obs_prob*prev_weights[i];
            second_term += log(obs_prob)*current_weights[i];
        }
    }

    if(first_term == 0)
        entropy = 0;
    else
    {
        entropy = log(first_term) - second_term;
        ROS_WARN_STREAM("first: " << first_term);
        ROS_WARN_STREAM("second: " << second_term);
    }
    return entropy;
}

/** \brief Compute entropy of probability distribution as an approximation.
 The approximation is based on a bound for the entropy of a Gaussian Mixture Model.
 The set of particles can be approximated as a GMM.
 Eq. 12 from paper "Active Sensing for Range-Only Mapping using Multiple Hypothesis".
 By L. Merino, F. Caballero and A. Ollero.
  */
double PersonParticleFilter::entropyGMM()
{   
    double w;
    double entropy = 0.0;

    if(sigma_pose_ > 0.0)
    {
        for(int i = 0; i < particles_.size(); i++)
        {
            w = particles_[i]->weight_;
            entropy += w*(-log(w) + 0.5*log(pow(2*M_PI*exp(1),2)*pow(sigma_pose_,4)));
        }
    }
    else
        ROS_ERROR("Entropy cannot be computed without prediction model");

    return entropy;
}

/** \brief Static version of the entropy calculation function (for performance reasons)
\param current_weights Current particle weights
\param sigma_pose Standard deviation for person movement
  */
double PersonParticleFilter::entropyGMM(const vector<double>& current_weights, double sigma_pose)
{
    double w;
    double entropy = 0.0;

    if(sigma_pose > 0.0)
    {
        for(int i = 0; i < current_weights.size(); i++)
        {
            w = current_weights[i];
            entropy += w*(-log(w) + 0.5*log(pow(2*M_PI*exp(1),2)*pow(sigma_pose,4)));
        }
    }
    else
        ROS_ERROR("Entropy cannot be computed without prediction model");

    return entropy;
}


/** Initialize filter with a specific set of particles
  \param particle_set Particle set to initialize
  */
void PersonParticleFilter::initFromParticles(const sensor_msgs::PointCloud &particle_set)
{
    int num_particles = particle_set.points.size();

    if(particles_.size() != num_particles)
    {
        particles_.resize(num_particles, 0);
    }

    // Look for the channel with weights
    int weights_channel = 0;
    while(particle_set.channels[weights_channel].name != "weights")
    {
        weights_channel++;
        if(weights_channel >= particle_set.channels.size())
        {
            ROS_ERROR("PersonParticleFilter: Particle set must have a 'weights' channel");
            return;
        }
    }

    // Copy particles from particle set
    for(int i = 0; i < num_particles; i++)
    {
        if(particles_[i] == 0) particles_[i] = (Particle*) (new PersonParticle());
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->pose_[0] = particle_set.points[i].x;
        part_ptr->pose_[1] = particle_set.points[i].y;
        part_ptr->weight_ = particle_set.channels[weights_channel].values[i];
    }
}


