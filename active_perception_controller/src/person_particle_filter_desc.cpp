#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <iterator>

#include <active_perception_controller/person_particle_filter_desc.h>
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
           // part_ptr->pose_[1] = - (map_->info.origin.position.y + map_->info.resolution * (map_->info.height - free_point.second));
	    part_ptr->pose_[1] =  (map_->info.origin.position.y + map_->info.resolution * (free_point.second));
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
          //  size_t map_y = floor((part_ptr->pose_[1] + dy + map_->info.origin.position.y)/map_->info.resolution + map_->info.height);
            size_t map_y = floor((part_ptr->pose_[1] + dy - map_->info.origin.position.y)/map_->info.resolution);
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
        RfidSensorData *rfid_obs = (RfidSensorData *)&obs_data;

        for(int i = 0; i < particles_.size(); i++)
        {
            // Save previous information
            prev_weights_[i] = particles_[i]->weight_;
            PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);

            if(hypot(part_ptr->pose_[0] - rfid_obs->pose_.pose.position.x, part_ptr->pose_[1] - rfid_obs->pose_.pose.position.y) <= rfid_model_->getMaximumSensorRange()) {
                particles_[i]->weight_ = particles_[i]->weight_ * rfid_model_->applySensorModel(obs_data, particles_[i]);
            }
            else
            {
                if(rfid_obs->rfid_)
                    particles_[i]->weight_ = 0.0;
            }

            total_weight += particles_[i]->weight_;
        }

        if(total_weight > 0)
        {
        // Normalize weights
            for(int i = 0; i < particles_.size(); i++)
            {
               //particles_[i]->weight_ = particles_[i]->weight_/total_weight;
		i=i;
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
\param use_particle_idx Particles used for updating
*/
void PersonParticleFilter::update(RfidSensorModel &rfid_model,
                                  vector<Particle*> &particles,
                                  SensorData &obs_data,
                                  const vector<double>& prev_weights,
                                  vector<double>& updated_weights,
                                  const vector<size_t>& use_particle_idx)
{
    double total_weight = 1.0; //we assume that the previous weights are already normalized

    // Update weights. We assume all observations are from RFID sensor
    for(int i = 0; i < use_particle_idx.size(); i++)
    {
        size_t idx = use_particle_idx[i];
        updated_weights[idx] = prev_weights[idx]*rfid_model.applySensorModel(obs_data, particles[idx]);
        total_weight -= prev_weights[idx];
        total_weight += updated_weights[idx];
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

    float total_weight=0.0;

    for (int i=0; i<particles_.size(); i++)
	total_weight=total_weight+particles_[i]->weight_;


    for(int i = 0; i < particles_.size(); i++)
    {
         	particles_[i]->weight_ = particles_[i]->weight_/total_weight;
    }

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
              (see the original derivation and remember that lim_{x->0} x log(x) = 0).
              Particles with weight 0 or 0 probability to get observation will have
              belief zero, so entropy zero. They do not contribute to the entropy.
              */

            for(int i = 0; i < particles_.size(); i++)
            {
                obs_prob = rfid_model_->applySensorModel(*last_obs_, particles_[i]);
                if(obs_prob > 0 && prev_weights_[i] > 0)
                {
                    first_term += obs_prob*prev_weights_[i];
                    second_term += log(obs_prob*prev_weights_[i])*particles_[i]->weight_;
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
      (see the original derivation and remember that lim_{x->0} x log(x) = 0).
      Particles with weight 0 or 0 probability to get observation will have
      belief zero, so entropy zero. They do not contribute to the entropy.
      */

    for(int i = 0; i < particles.size(); i++)
    {
        obs_prob = rfid_model.applySensorModel(obs, particles[i]);
        if(obs_prob > 0 && prev_weights[i] > 0)
        {
            first_term += obs_prob*prev_weights[i];
            second_term += log(obs_prob*prev_weights[i])*current_weights[i];
        }
    }

    if(first_term == 0)
        entropy = 0;
    else
    {
        entropy = log(first_term) - second_term;
        //ROS_WARN_STREAM("first: " << first_term);
        //ROS_WARN_STREAM("second: " << second_term);
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
            if( w > 0 )
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
            if( w > 0 )
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

/** Join two subsets of particles 
  */
void PersonParticleFilter::join_sets(const sensor_msgs::PointCloud &particle_set)
{
	int num_particles1 = particles_.size();
	int num_particles2 = particle_set.points.size(); 
	int weights_channel = 0;

    	if(particles_.size() != num_particles1+num_particles2)
    	{
        	particles_.resize(num_particles1+num_particles2, 0);
    	

    		// Look for the channel with weights
    		
    		while(particle_set.channels[weights_channel].name != "weights")
    		{	
        		weights_channel++;
        		if(weights_channel >= particle_set.channels.size())
        		{
            			ROS_ERROR("PersonParticleFilter: Particle set must have a 'weights' channel");
            			return;
        		}
    		}
	}
	
	for (int i=num_particles1; i < num_particles1+num_particles2; i++)
	{
		if(particles_[i] == 0) particles_[i] = (Particle*) (new PersonParticle());
		PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
       	 	part_ptr->pose_[0] = particle_set.points[i-num_particles1].x;
        	part_ptr->pose_[1] = particle_set.points[i-num_particles1].y;
        	part_ptr->weight_ = particle_set.channels[weights_channel].values[i-num_particles1];		
	}
}

/** Divide into two subsets of particles
  */
void PersonParticleFilter::divide_1to1(double ow, double nw, double own_pos[2], double neighbor_pos[2])
{
	int num_particles = particles_.size();
	vector<PersonParticle*> own_particles;
	vector<PersonParticle*> neighbor_particles;
	double total_weight=0.0;
	double actual_ow=0.0;
	double actual_nw=0.0;
	vector<float> op_do;
	vector<float> op_dn;
	vector<float> np_do;
	vector<float> np_dn;


	for (int i=0; i<num_particles; i++)
		total_weight=total_weight+particles_[i]->weight_;

        for(int i = 0; i < particles_.size(); i++)
        {
         	particles_[i]->weight_ = particles_[i]->weight_/total_weight;
        }
	
	total_weight=0.0;
	for (int i=0; i<num_particles; i++)
		total_weight=total_weight+particles_[i]->weight_;


	ow=ow*total_weight;
	nw=nw*total_weight;


	for (int i=0; i<num_particles; i++)
	{
		
		PersonParticle * part_ptr = (PersonParticle *)(particles_[i]);
		if (sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2))-sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2))<0.0)
		{
			if (actual_ow<ow)
			{
				actual_ow=actual_ow+part_ptr->weight_;
				own_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
				op_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));
				op_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));
				
			}
			else
			{
				int tam=own_particles.size();
				//vector<float>::iterator max_ind = std::max_element(op_do.begin(),op_do.end());
				int max_ind = std::distance(op_do.begin(),std::max_element(op_do.begin(),op_do.end()));
				if (ow>0 && (tam==0 || sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2))<op_do[int(max_ind)]))
				{
					actual_ow=actual_ow+part_ptr->weight_;
					own_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
					op_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));
					op_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));	
					while((actual_ow-ow)>0.0)
					{
						max_ind = std::distance(op_do.begin(),std::max_element(op_do.begin(),op_do.end()));
						neighbor_particles.push_back((PersonParticle *)(own_particles[max_ind]));
						np_dn.push_back(sqrt(pow(own_particles[max_ind]->pose_[0]-neighbor_pos[0],2)+pow(own_particles[max_ind]->pose_[1]-neighbor_pos[1],2)));
						np_do.push_back(sqrt(pow(own_particles[max_ind]->pose_[0]-own_pos[0],2)+pow(own_particles[max_ind]->pose_[1]-own_pos[1],2)));
						actual_nw=actual_nw+own_particles[max_ind]->weight_;
						actual_ow=actual_ow-own_particles[max_ind]->weight_;
						own_particles.erase(own_particles.begin()+max_ind);
						op_do.erase(op_do.begin()+max_ind);
						op_dn.erase(op_dn.begin()+max_ind);
					}
					
				}
				else
				{
					actual_nw=actual_nw+part_ptr->weight_;
					neighbor_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
					np_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));
					np_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));
				}
			}
		}
		else
		{
			if (actual_nw<nw)
			{
				actual_nw=actual_nw+part_ptr->weight_;
				neighbor_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
				np_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));
				np_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));
			}
			else
			{
				int tam=neighbor_particles.size();
				//vector<float>::iterator max_ind = std::max_element(np_dn.begin(),np_dn.end());
				int max_ind = std::distance(np_dn.begin(),std::max_element(np_dn.begin(),np_dn.end()));
				if (nw>0 && (tam==0 || sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2))<np_dn[max_ind]))
				{
					actual_nw=actual_nw+part_ptr->weight_;
					neighbor_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
					np_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));
					np_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));	
					while((actual_nw-nw)>0.0)
					{
						max_ind = std::distance(np_dn.begin(),std::max_element(np_dn.begin(),np_dn.end()));
						own_particles.push_back((PersonParticle *)(neighbor_particles[max_ind]));
						
						op_do.push_back(sqrt(pow(neighbor_particles[max_ind]->pose_[0]-own_pos[0],2)+pow(neighbor_particles[max_ind]->pose_[1]-own_pos[1],2)));
						
						op_dn.push_back(sqrt(pow(neighbor_particles[max_ind]->pose_[0]-neighbor_pos[0],2)+pow(neighbor_particles[max_ind]->pose_[1]-neighbor_pos[1],2)));
						actual_ow=actual_ow+neighbor_particles[max_ind]->weight_;
						actual_nw=actual_nw-neighbor_particles[max_ind]->weight_;
						neighbor_particles.erase(neighbor_particles.begin()+max_ind);
						np_dn.erase(np_dn.begin()+max_ind);
						np_do.erase(np_do.begin()+max_ind);
					}
				}
				else
				{
					actual_ow=actual_ow+part_ptr->weight_;
					own_particles.push_back(new PersonParticle(*((PersonParticle *)(particles_[i]))));
					op_do.push_back(sqrt(pow(part_ptr->pose_[0]-own_pos[0],2)+pow(part_ptr->pose_[1]-own_pos[1],2)));
					op_dn.push_back(sqrt(pow(part_ptr->pose_[0]-neighbor_pos[0],2)+pow(part_ptr->pose_[1]-neighbor_pos[1],2)));
				}
			}
		}

	}
	
    	// Free old set
    	for(int i = 0; i < particles_.size(); i++)
    	{
        	delete ((PersonParticle *)(particles_[i]));
    	}	
    	particles_.clear(); 	

	// Replace with new set
    	for(int i = 0; i < own_particles.size(); i++)
	{
        	particles_.push_back((Particle *)(own_particles[i]));
	}
}


/** Divide into n subsets of particles
  */
void PersonParticleFilter::divide_cv(double la, double oa, double ra, double left_pos[2], double right_pos[2])
{
	double total_weight=0.0;
	double left_weight=0.0;
	double right_weight=0.0;
	int num_particles = particles_.size();


	for (int i=0; i<num_particles; i++)
		total_weight=total_weight+particles_[i]->weight_;
		
	if (la>0 && ra>0)
	{
		left_weight=(la+oa)*total_weight/(la+oa+ra);
		right_weight=ra*total_weight/(la+oa+ra);
		this->divide_1to1(left_weight,right_weight,left_pos,right_pos);

		total_weight=0;
		for (int i=0; i<num_particles; i++)
			total_weight=total_weight+particles_[i]->weight_;

		left_weight=la*total_weight/(la+oa);
		right_weight=oa*total_weight/(la+oa);
		this->divide_1to1(right_weight,left_weight,right_pos,left_pos);
	}
	else if (la>0)
	{
		left_weight=la*total_weight/(la+oa);
		right_weight=oa*total_weight/(la+oa);
		this->divide_1to1(right_weight,left_weight,right_pos,left_pos);
	}
	else if (ra>0)
	{
		left_weight=oa*total_weight/(oa+ra);
		right_weight=ra*total_weight/(oa+ra);
		this->divide_1to1(left_weight,right_weight,left_pos,right_pos);
	}
}

