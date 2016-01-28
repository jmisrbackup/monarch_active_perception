#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Bool.h>
#include <boost/ref.hpp>

#include <sam_helpers/writer.h>
#include <sam_helpers/reader.h>
#include <monarch_situational_awareness/CreateWriter.h>
#include <monarch_situational_awareness/CreateReader.h>

#include <active_perception_controller/person_particle_filter_desc.h>
#include <active_perception_controller/rfid_sensor_model.h>

#include <string>

using namespace std;

/**
This class implements a particle filter to estimate the position of a person. Measurements from a mobile RFID reader
on board a robot are used.
*/
class PersonEstimator
{
  protected:

    ros::NodeHandle nh_;
    ros::Publisher person_belief_pub_;                  /// To publish person pose belief
    ros::Publisher person_belief_pub_2;                  /// To publish person pose belief
    ros::Publisher robot_pos_pub_;                  /// To publish robot pose
    ros::Subscriber person_belief_right_sub_;                  /// To get person pose belief
    ros::Subscriber person_belief_left_sub_;                  /// To get person pose belief
    monarch_situational_awareness::CreateWriter person_belief_;
    monarch_situational_awareness::CreateWriter robot_pos_;
    monarch_situational_awareness::CreateReader person_belief_left;
    monarch_situational_awareness::CreateReader person_belief_right;
    monarch_situational_awareness::CreateReader robot_pose_left;
    monarch_situational_awareness::CreateReader robot_pose_right;
    ros::Subscriber robot_pose_sub_;                    /// To get robot position
    ros::Subscriber robotleft_pose_sub_;                    /// To get robot position
    ros::Subscriber robotright_pose_sub_;                    /// To get robot position
    ros::Subscriber robot_cloud_sub_;                   /// To get robot position particles
    ros::Subscriber rfid_sub_;                          /// To get RFID measurements

    double step_duration_;                              /// Step duration in seconds

    int num_particles_;                                 /// Number of particles of the filter
    string global_frame_id_;                            /// Global frame id
    string robot_name;
    string robot_left;
    string robot_right;
    bool left_received;
    bool right_received;

    PersonParticleFilter *person_pf_;                   /// Particle filter for person
    sensor_msgs::PointCloud person_pf_left;                   /// Particle filter for person
    sensor_msgs::PointCloud person_pf_right;                   /// Particle filter for person
    geometry_msgs::PoseArray robot_cloud_;              /// Particles for robot pose
    geometry_msgs::PoseWithCovariance robot_pose_;      /// Robot position
    geometry_msgs::PoseWithCovariance robot_right_pose_;
    geometry_msgs::PoseWithCovariance robot_left_pose_;
    geometry_msgs::PoseWithCovarianceStamped robot_pose_aux_;

    double left_pos[2];
    double right_pos[2];
    double left_weight;
    double own_weight;
    double right_weight;

    bool first_rob_pose_;                               /// First robot pose arrived
    bool new_measure_;                                  /// Indicates new measure arrived
    bool rfid_mes_;                                     /// RFID measurement
    bool filter_running_;                               /// True when the filter is initialized
    nav_msgs::OccupancyGrid map_;                       /// Map

    /// Callback to receive the robot poses
    void robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    /// Callback to receive the robot particles
    void robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg);
    /// Callback to receive the RFID measures
    void rfidReceived(const std_msgs::BoolConstPtr& rfid_msg);

    void personBeliefLeft(const sensor_msgs::PointCloudConstPtr& personbelief_msg);
    void personBeliefRight(const sensor_msgs::PointCloudConstPtr& personbelief_msg);
    void robotRightPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    void robotLeftPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    /// Retreive the map
    void requestMap();


  public:
  double comm_period;
  int comm_steps;
  int cont_steps;
  int side;
  PersonEstimator();
  ~PersonEstimator();
  double getStepDuration();
  void runIteration();

};

/** Constructor
*/
PersonEstimator::PersonEstimator()
{
    ros::NodeHandle private_nh("~");
    double sigma_person;
    double rfid_map_res;
    string rfid_prob_map;
    string robot_left;
    string robot_right;


    // Read parameters
    private_nh.param("step_duration", step_duration_, 0.2);
    private_nh.param("num_particles", num_particles_, 5000);
    private_nh.param("sigma_person", sigma_person, 0.05);
    private_nh.param("global_frame_id", global_frame_id_, string("map"));

    if(!private_nh.getParam("rfid_map_resolution", rfid_map_res)
            || !private_nh.getParam("rfid_prob_map", rfid_prob_map))
    {
        ROS_ERROR("No correct sensor model for RFID");
    }

    private_nh.param("robot_name", robot_name, string("mbot01"));
    private_nh.param("robot_left", robot_left, string("mbot03"));
    private_nh.param("robot_right", robot_right, string("mbot02"));

    private_nh.param("left_pos_x",left_pos[0],-30.0);
    private_nh.param("left_pos_y",left_pos[1],0.0);
    private_nh.param("right_pos_x",right_pos[0],30.0);
    private_nh.param("right_pos_y",right_pos[1],0.0);
    private_nh.param("left_weight",left_weight,0.0);
    private_nh.param("own_weight",own_weight,1.0);
    private_nh.param("right_weight",right_weight,0.0);
    private_nh.param("comm_period",comm_period,10.0);
    private_nh.param("odd",side,1);

    // Subscribe/advertise topics

    std::stringstream topic_robotname;
    topic_robotname << "[" << robot_name << "] Person_belief";
    ros::Duration d(5.0);
    while(!person_belief_.response.success)
    {
        person_belief_.request.properties.slot_name = topic_robotname.str().c_str();
        service::call("create_writer",person_belief_);
        d.sleep();
    }
    string topic_name;
    topic_name = person_belief_.response.topic_name;
    person_belief_pub_ = nh_.advertise<sensor_msgs::PointCloud> (topic_name,1);

    person_belief_pub_2 = nh_.advertise<sensor_msgs::PointCloud>("person_particle_cloud", 1);

    /*std::stringstream topic2_robotname;
    topic2_robotname << "[" << robot_name << "] RobotPose";
    while(!robot_pos_.response.success)
    {
        robot_pos_.request.properties.slot_name = topic2_robotname.str().c_str();
        service::call("create_writer",robot_pos_);
        d.sleep();
    }
    topic_name = robot_pos_.response.topic_name;
    robot_pos_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> (topic_name,1);*/


    std::stringstream topic_robotleft;
    topic_robotleft << "[" << robot_left << "] Person_belief";
    while(!person_belief_left.response.success)
    {
        person_belief_left.request.properties.slot_name = topic_robotleft.str().c_str();
        service::call("create_reader",person_belief_left);
        d.sleep();
    }
    person_belief_left_sub_ = nh_.subscribe<sensor_msgs::PointCloud> (person_belief_left.response.topic_name,1, &PersonEstimator::personBeliefLeft, this);

 /*   std::stringstream topic_robotnameleft;
    topic_robotnameleft << "[" << robot_left << "] RobotPose";
    while(!robot_pose_left.response.success)
    {
        robot_pose_left.request.properties.slot_name = topic_robotnameleft.str().c_str();
        service::call("create_reader",robot_pose_left);
        d.sleep();
    }
    robotleft_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> (robot_pose_left.response.topic_name,1, &PersonEstimator::robotLeftPoseReceived, this);*/

    std::stringstream topic_robotright;
    topic_robotright << "[" << robot_right << "] Person_belief";
    while(!person_belief_right.response.success)
    {
        person_belief_right.request.properties.slot_name = topic_robotright.str().c_str();
        service::call("create_reader",person_belief_right);
        d.sleep();
    }
    person_belief_right_sub_ = nh_.subscribe<sensor_msgs::PointCloud> (person_belief_right.response.topic_name,1, &PersonEstimator::personBeliefRight, this);

   /* std::stringstream topic_robotnameright;
    topic_robotnameright << "[" << robot_right << "] RobotPose";
    while(!robot_pose_right.response.success)
    {
        robot_pose_right.request.properties.slot_name = topic_robotnameright.str().c_str();
        service::call("create_reader",robot_pose_right);
        d.sleep();
    }
    robotright_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> (robot_pose_right.response.topic_name,1, &PersonEstimator::robotRightPoseReceived, this);*/


    robot_pose_sub_ = nh_.subscribe("amcl_pose", 1, &PersonEstimator::robotPoseReceived, this);
    robot_cloud_sub_ = nh_.subscribe("particlecloud", 1, &PersonEstimator::robotCloudReceived, this);
    rfid_sub_ = nh_.subscribe("rfid", 1, &PersonEstimator::rfidReceived, this);

    

    // Read the map
    requestMap();

    

    person_pf_ = new PersonParticleFilter(num_particles_, &map_, sigma_person, rfid_map_res, rfid_prob_map);

    first_rob_pose_ = false;
    new_measure_ = false;
    filter_running_ = false;
    left_received=true;
    right_received=true;
}

/// Destructor
PersonEstimator::~PersonEstimator()
{
    delete person_pf_;
}

/// Get the step duration
double PersonEstimator::getStepDuration()
{
    return step_duration_;
}

/** This method runs an iteration of the particle filter. Prediction and update.
*/
void PersonEstimator::runIteration()
{
    bool publish_data = false;
    bool publish_data_sam=false;
    RfidSensorData rfid_obs;
    

    // Initialize filter once first robot pose is available
    if(!filter_running_ && first_rob_pose_)
    {
        person_pf_->initUniform();
	person_pf_->divide_cv(left_weight, own_weight, right_weight, left_pos, right_pos);
        publish_data = true;
        publish_data_sam = true;
        filter_running_ = true;
        		sensor_msgs::PointCloud cloud_msg;
        		cloud_msg.header.stamp = ros::Time::now();
        		cloud_msg.header.frame_id = global_frame_id_;
        		cloud_msg.points.resize(person_pf_->getNumParticles());
        		sensor_msgs::ChannelFloat32 weights;
        		weights.values.resize(person_pf_->getNumParticles());
        		weights.name = "weights";

        		// Retrieve particles
        		for(int i = 0; i < person_pf_->getNumParticles(); i++)
        		{
        		    PersonParticle* particle =  (PersonParticle *)person_pf_->getParticle(i);
        		    cloud_msg.points[i].x = particle->pose_[0];
        	 	   cloud_msg.points[i].y = particle->pose_[1];
        	 	   weights.values[i] = particle->weight_;
        		}
        		cloud_msg.channels.push_back(weights);
        		person_belief_pub_.publish(cloud_msg);
    }
    else if(filter_running_)
    {
        // Predict
        person_pf_->predict(step_duration_);

        // Update if new measures
        if(new_measure_)
        {    
            new_measure_ = false;
        }
        else
        {
            // When there is no reading from RFID, consider a negative measure
            rfid_mes_ = false;  
        }

        rfid_obs.rfid_ = rfid_mes_;
        rfid_obs.pose_ = robot_pose_;
        person_pf_->update(rfid_obs);

	if (cont_steps>=comm_steps)
	{
		double own_pos[2];
		double neighbor_pos[2];
		own_pos[0]=robot_pose_.pose.position.x;
		own_pos[1]=robot_pose_.pose.position.y;
		//neighbor_pos[0]=0.0;
		//neighbor_pos[1]=0.0;
		if ((side==0 && person_pf_left.points.size()>0) || (side==1 && person_pf_right.points.size()>0))
		{
        		sensor_msgs::PointCloud cloud_msg;
        		cloud_msg.header.stamp = ros::Time::now();
        		cloud_msg.header.frame_id = global_frame_id_;
        		cloud_msg.points.resize(person_pf_->getNumParticles());
        		sensor_msgs::ChannelFloat32 weights;
        		weights.values.resize(person_pf_->getNumParticles());
        		weights.name = "weights";

        		// Retrieve particles
        		for(int i = 0; i < person_pf_->getNumParticles(); i++)
        		{
        		    PersonParticle* particle =  (PersonParticle *)person_pf_->getParticle(i);
        		    cloud_msg.points[i].x = particle->pose_[0];
        	 	   cloud_msg.points[i].y = particle->pose_[1];
        	 	   weights.values[i] = particle->weight_;
        		}
        		cloud_msg.channels.push_back(weights);
        		person_belief_pub_.publish(cloud_msg);
		}
		
		
		if (side==0 && person_pf_left.points.size()>0)
		{
			while (!left_received);
			neighbor_pos[0]=robot_left_pose_.pose.position.x;
			neighbor_pos[1]=robot_left_pose_.pose.position.y;
			person_pf_->join_sets(person_pf_left);
			person_pf_->resample();
			person_pf_->divide_1to1(0.5,0.5,right_pos,left_pos);
			publish_data_sam=true;
			left_received=false;
		}
		else if (side==1 && person_pf_right.points.size()>0)
		{
			while(!right_received);
			neighbor_pos[0]=robot_right_pose_.pose.position.x;
			neighbor_pos[1]=robot_right_pose_.pose.position.y;
			person_pf_->join_sets(person_pf_right);
			person_pf_->resample();
			person_pf_->divide_1to1(0.5,0.5,left_pos,right_pos);
			publish_data_sam=true;
			right_received=false;
		}
		//else
			//person_pf_->resample();	
	}

        publish_data = true;
    }

    // Publish belief
    if(publish_data)
    {
        sensor_msgs::PointCloud cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = global_frame_id_;
        cloud_msg.points.resize(person_pf_->getNumParticles());
        sensor_msgs::ChannelFloat32 weights;
        weights.values.resize(person_pf_->getNumParticles());
        weights.name = "weights";

        // Retrieve particles
        for(int i = 0; i < person_pf_->getNumParticles(); i++)
        {
            PersonParticle* particle =  (PersonParticle *)person_pf_->getParticle(i);
            cloud_msg.points[i].x = particle->pose_[0];
            cloud_msg.points[i].y = particle->pose_[1];
            weights.values[i] = particle->weight_;
        }
        cloud_msg.channels.push_back(weights);
	//if(publish_data_sam)
        //	person_belief_pub_.publish(cloud_msg);
	person_belief_pub_2.publish(cloud_msg);
    }
}

/** Callback to receive the robot poses.

*/
void PersonEstimator::robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_pose_ = pose_msg->pose;
    /*robot_pose_aux_.pose=pose_msg->pose;
    robot_pose_aux_.header=pose_msg->header;
    robot_pos_pub_.publish(robot_pose_aux_);*/

    if(!first_rob_pose_)
        first_rob_pose_ = true;
}

/** Callback to receive the robot cloud.

*/
void PersonEstimator::robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg)
{
    robot_cloud_ = *pose_msg;
}

/** Callback to receive the RFID.

*/
void PersonEstimator::rfidReceived(const std_msgs::BoolConstPtr& rfid_msg)
{
    new_measure_ = true;
    rfid_mes_ = rfid_msg->data;
}

/** Callback to receive the PersonBelief on the left.

*/
void PersonEstimator::personBeliefLeft(const sensor_msgs::PointCloudConstPtr& personbelief_msg)
{
    person_pf_left = *personbelief_msg;
    left_received=true;
}

/** Callback to receive the PersonBelief on the right.

*/
void PersonEstimator::personBeliefRight(const sensor_msgs::PointCloudConstPtr& personbelief_msg)
{
    person_pf_right = *personbelief_msg;
    right_received=true;
}

/** Callback to receive the robotPos on the right.

*/
void PersonEstimator::robotRightPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_right_pose_ = pose_msg->pose;

}


/** Callback to receive the robotPos on the left.

*/
void PersonEstimator::robotLeftPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_left_pose_ = pose_msg->pose;
}


/** Retreive the map

*/
void PersonEstimator::requestMap()
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    ROS_INFO("Map retreived");
    map_ = resp.map;
}

/// Main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_estimator");

    // Read parameters
    PersonEstimator person_filter;

    

    double step_duration = person_filter.getStepDuration();
    ros::Rate loop_rate(1.0/step_duration);

    person_filter.comm_steps=person_filter.comm_period/step_duration;
    person_filter.cont_steps=0;
    //person_filter.side=0;

    while (ros::ok())
    {
        ros::spinOnce();
        person_filter.runIteration();
	if (person_filter.cont_steps>=person_filter.comm_steps)
		person_filter.cont_steps=0;
		person_filter.side=abs(person_filter.side-1);
        loop_rate.sleep();
	person_filter.cont_steps++;
    }

    return 0;
}
