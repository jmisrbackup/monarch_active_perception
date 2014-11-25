#include <active_perception_controller/person_particle_filter.h>
#include <active_perception_controller/sensor_model.h>

#include <cstdlib>

#define FILTER_ON_PREDICTION 0

/**
  Constructor
  */
PersonParticle::PersonParticle()
{
    pose_.resize(2,0);
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
   \param map Occupancy map
   \param sigma_pose Standard deviation of person movement noise
   \param rfid_map_res Resolution of RFID maps
   \param rfid_prob_pos Probability map for positive RFID
   \param rfid_prob_neg Probability map for negative RFID
  */
PersonParticleFilter::PersonParticleFilter(int n_particles, nav_msgs::OccupancyGrid const* map, double sigma_pose, double rfid_map_res, string rfid_prob_pos, string rfid_prob_neg):ParticleFilter(map)
{
    for(int i = 0; i < n_particles; i++)
    {
        particles_.push_back(new PersonParticle());
    }

    sigma_pose_ = sigma_pose;

    rfid_model_ = new RfidSensorModel(rfid_prob_pos, rfid_prob_neg, rfid_map_res);
}

/** Destructor
  */
PersonParticleFilter::~PersonParticleFilter()
{
    for(int i = 0; i < particles_.size(); i++)
    {
        delete ((PersonParticle *)(particles_[i]));
    }

    delete rfid_model_;
}

/** Draw particles from a uniform distribution
  */
void PersonParticleFilter::initUniform()
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
		size_t map_x = floor((part_ptr->pose_[0] + dx - map_->info.origin.position.x)/map_->info.resolution);
		size_t map_y = floor((part_ptr->pose_[1] + dy + map_->info.origin.position.y)/map_->info.resolution + map_->info.height);
		if(map_->data[map_y*map_->info.width + map_x] != 0) //occupied cell
		{
			dx = 0;
			dy = 0;
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

    // Update weights. We assume all observations are from RFID sensor
    for(int i = 0; i < particles_.size(); i++)
    {
        particles_[i]->weight_ = rfid_model_->applySensorModel(obs_data, particles_[i]);
        total_weight += particles_[i]->weight_;
    }

    // Normalize weights
    for(int i = 0; i < particles_.size(); i++)
    {
       particles_[i]->weight_ = particles_[i]->weight_/total_weight;
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
    double total_weight = 0.0;
    for(int i = 0; i < resampled_particles.size(); i++)
    {
        particles_.push_back((Particle *)(resampled_particles[i]));
        total_weight += resampled_particles[i]->weight_;
    }

    // Normalize
    for(int i = 0; i < particles_.size(); i++)
    {
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
        part_ptr->weight_ = part_ptr->weight_/total_weight;
    }
}

/** Initialize filter with a specific set of particles
  \param particle_set Particle set to initialize
  */
void PersonParticleFilter::initFromParticles(sensor_msgs::PointCloud &particle_set)
{
    int num_particles = particle_set.points.size();

    if(particles_.size() != num_particles)
    {
        particles_.resize(num_particles);
    }

    // Look for the channel with weights
    int weights_channel = 0;
    while(particle_set.channels[weights_channel].name.c_str() != "weights")
    {
        weights_channel++;
    }

    // Copy particles from particle set
    for(int i = 0; i < num_particles; i++)
    {
        PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);

        part_ptr->pose_[0] = particle_set.points[i].x;
        part_ptr->pose_[1] = particle_set.points[i].y;
        part_ptr->weight_ = particle_set.channels[weights_channel].values[i];
    }
}

/** Compute the probability of obtaining an RFID measure given a reader and emitter
  \param rfid_mes RFID measure
  \param x_r Reader position
  \param y_r Reader position
  \param x_r_cov Covariance in robot position
  \param y_r_cov Covariance in robot position
  \param x_e Emitter position
  \param y_e Emitter position
  */
/*
double PersonParticleFilter::computeObsProb(bool &rfid_mes, double x_r, double y_r, double x_r_cov, double y_r_cov, double x_e, double y_e)
{

#ifndef FILTER_ON_PREDICTION
	size_t map_x = floor((x_e - map_->info.origin.position.x)/map_->info.resolution);
	size_t map_y = floor((y_e + map_->info.origin.position.y)/map_->info.resolution + map_->info.height);

	if(map_->data[map_y*map_->info.width + map_x] != 0) //occupied cell
	{
		return 0.0;
	}
#endif

    double obs_prob;
    double distance = sqrt(((x_r-x_e)*(x_r-x_e)) + ((y_r-y_e)*(y_r-y_e)));

    if(distance <= d_threshold_)
    {
        if(rfid_mes)
            obs_prob = prob_positive_det_;
        else
            obs_prob = 1.0 - prob_positive_det_;
    }
    else
    {
        if(rfid_mes)
            obs_prob = prob_false_det_;
        else
            obs_prob = 1.0 - prob_false_det_;
    }
    return obs_prob;
}
*/


