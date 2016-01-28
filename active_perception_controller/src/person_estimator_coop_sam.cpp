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

#include <active_perception_controller/person_particle_filter.h>
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
    monarch_situational_awareness::CreateWriter person_belief_;
    ros::Subscriber robot_pose_sub_[10];                    /// To get robot position
    monarch_situational_awareness::CreateReader robot_reader_[10];
    ros::Subscriber robot_cloud_sub_[10];                   /// To get robot position particles
    ros::Subscriber rfid_sub_[10];                          /// To get RFID measurements
    monarch_situational_awareness::CreateReader rfid_[10];

    double step_duration_;                              /// Step duration in seconds
    int num_particles_;                                 /// Number of particles of the filter
    string global_frame_id_;                            /// Global frame id
    int num_robots_;					/// Number of robots uisng this filter
    string robot_names_[10];

    PersonParticleFilter *person_pf_;                   /// Particle filter for person
    geometry_msgs::PoseArray robot_cloud_[10];              /// Particles for robot pose
    geometry_msgs::PoseWithCovariance robot_pose_[10];      /// Robot position

    bool first_rob_pose_;                               /// First robot pose arrived
    bool new_measure_[10];                                  /// Indicates new measure arrived
    bool rfid_mes_[10];                                     /// RFID measurement
    bool filter_running_;                               /// True when the filter is initialized
    nav_msgs::OccupancyGrid map_;                       /// Map


    /// Callback to receive the robot poses
    void robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, int index);
    /// Callback to receive the robot particles
    void robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg, int index);
    /// Callback to receive the RFID measures
    void rfidReceived(const std_msgs::BoolConstPtr& rfid_msg,int index);
    /// Retreive the map
    void requestMap();

  public:
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

    // Read parameters
    private_nh.param("step_duration", step_duration_, 0.2);
    private_nh.param("num_particles", num_particles_, 5000);
    private_nh.param("sigma_person", sigma_person, 0.05);
    private_nh.param("global_frame_id", global_frame_id_, string("map"));
    private_nh.param("num_robots", num_robots_, 1);
    for (int i=0;i<num_robots_;i++)
    {
	std::stringstream parameter;
	parameter << "robot_name_" << i;
	private_nh.param(parameter.str().c_str(), robot_names_[i], string("mbot01"));
    }

    if(!private_nh.getParam("rfid_map_resolution", rfid_map_res)
            || !private_nh.getParam("rfid_prob_map", rfid_prob_map))
    {
        ROS_ERROR("No correct sensor model for RFID");
    }


    // Subscribe/advertise topics

    string topic_name;
    ros::Duration d(5.0);
    while(!person_belief_.response.success)
    {
        person_belief_.request.properties.slot_name = "Person_belief";
        service::call("create_writer",person_belief_);
        d.sleep();
    }
    topic_name = person_belief_.response.topic_name;
    person_belief_pub_ = private_nh.advertise<sensor_msgs::PointCloud> (topic_name,1);
    //person_belief_pub_ = nh_.advertise<sensor_msgs::PointCloud>("person_particle_cloud", 1);
    for (int i=0; i<num_robots_; i++)
    {
    	   // char str[30] = "/robot_";
  	   // char index[4];
           // sprintf(index, "%d", i);
           // strcat(str, index);
           //  strcat(str, "/amcl_pose");
	    std::stringstream topicName;
	    topicName << "/robot_" << i << "/particlecloud";
	    //robot_cloud_sub_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topicName.str().c_str(), 1, boost::bind(&PersonEstimator::robotPoseReceived,this, _1, i));
	    robot_cloud_sub_[i] = nh_.subscribe<geometry_msgs::PoseArray>(topicName.str().c_str(), 1, boost::bind(&PersonEstimator::robotCloudReceived,this, _1, i));
    	   // char str1[30] = "/robot_";
           // strcat(str1, index);
           // strcat(str1, "/particlecloud");
	    std::stringstream topicName1;
	    topicName1 << "[" << robot_names_[i] << "] RobotPose";
	    ros::Duration d(0.5);
            while(!robot_reader_[i].response.success)
            {
        	robot_reader_[i].request.properties.slot_name = topicName1.str().c_str();
        	service::call("create_reader",robot_reader_[i]);
        	d.sleep();
    	    }
	    robot_pose_sub_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot_reader_[i].response.topic_name, 1, boost::bind(&PersonEstimator::robotPoseReceived,this, _1, i));
    	   // char str2[30] = "/robot_";
           // strcat(str2, index);
           // strcat(str2, "/rfid");
	    std::stringstream topicName2;
	    topicName2 << "[" << robot_names_[i] << "] RFIDDetected";
    	    
            while(!rfid_[i].response.success)
            {
        	rfid_[i].request.properties.slot_name = topicName2.str().c_str();
        	service::call("create_reader",rfid_[i]);
        	d.sleep();
    	    }
	    //topicName2 << "/robot_" << i << "/rfid";
	    rfid_sub_[i] = nh_.subscribe<std_msgs::Bool>(rfid_[i].response.topic_name, 1, boost::bind(&PersonEstimator::rfidReceived,this, _1, i));
	    new_measure_[i]=false;
    }

    // Read the map
    requestMap();

    person_pf_ = new PersonParticleFilter(num_particles_, &map_, sigma_person, rfid_map_res, rfid_prob_map);



    first_rob_pose_ = false;
    filter_running_ = false;
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
    RfidSensorData rfid_obs;

    // Initialize filter once first robot pose is available
    if(!filter_running_ && first_rob_pose_)
    {
        person_pf_->initUniform();
        publish_data = true;
        filter_running_ = true;
    }
    else if(filter_running_)
    {
        // Predict
        person_pf_->predict(step_duration_);

        // Update if new measures
	for (int i=0; i<num_robots_; i++)
	{
        	if(new_measure_[i])
        	{    
            		new_measure_[i] = false;
        	}
        	else
        	{
            		// When there is no reading from RFID, consider a negative measure
            		rfid_mes_[i] = false;  
        	}

        	rfid_obs.rfid_ = rfid_mes_[i];
        	rfid_obs.pose_ = robot_pose_[i];
        	person_pf_->update(rfid_obs);
	}

        person_pf_->resample();

        publish_data = true;
    }

    // Publish belief
    if(publish_data)
    {
	//SAMWriter<sensor_msgs::PointCloud> person_belief_pub_("Person_belief");
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
	//person_belief_pub_.remove();
    }
}

/** Callback to receive the robot poses.

*/
void PersonEstimator::robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, int index)
{
    robot_pose_[index] = pose_msg->pose;

    if(!first_rob_pose_)
        first_rob_pose_ = true;
}

/** Callback to receive the robot cloud.

*/
void PersonEstimator::robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg, int index)
{
    robot_cloud_[index] = *pose_msg;
}

/** Callback to receive the RFID.

*/
void PersonEstimator::rfidReceived(const std_msgs::BoolConstPtr& rfid_msg, int index)
{
    new_measure_[index] = true;
    rfid_mes_[index] = rfid_msg->data;
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

    while (ros::ok())
    {
        ros::spinOnce();
        person_filter.runIteration();
        loop_rate.sleep();
    }

    return 0;
}
