#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Bool.h>

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
    ros::Subscriber robot_pose_sub_;                    /// To get robot position
    ros::Subscriber robot_cloud_sub_;                   /// To get robot position particles
    ros::Subscriber rfid_sub_;                          /// To get RFID measurements

    double step_duration_;                              /// Step duration in seconds
    int num_particles_;                                 /// Number of particles of the filter
    string global_frame_id_;                            /// Global frame id

    PersonParticleFilter *person_pf_;                   /// Particle filter for person
    geometry_msgs::PoseArray robot_cloud_;              /// Particles for robot pose
    geometry_msgs::PoseWithCovariance robot_pose_;      /// Robot position

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
    string rfid_prob_pos;
    string rfid_prob_neg;


    // Read parameters
    private_nh.param("step_duration", step_duration_, 0.2);
    private_nh.param("num_particles", num_particles_, 5000);
    private_nh.param("sigma_person", sigma_person, 0.05);
    private_nh.param("global_frame_id", global_frame_id_, string("map"));

    if(!private_nh.getParam("rfid_map_resolution", rfid_map_res)
            || !private_nh.getParam("rfid_map_pos", rfid_prob_pos)
            || !private_nh.getParam("rfid_map_neg", rfid_prob_neg))
    {
        ROS_ERROR("No correct sensor model for RFID");
    }

    // Subscribe/advertise topics
    person_belief_pub_ = nh_.advertise<sensor_msgs::PointCloud>("particle_cloud", 1);
    robot_pose_sub_ = nh_.subscribe("amcl_pose", 1, &PersonEstimator::robotPoseReceived, this);
    robot_cloud_sub_ = nh_.subscribe("particlecloud", 1, &PersonEstimator::robotCloudReceived, this);
    rfid_sub_ = nh_.subscribe("rfid", 1, &PersonEstimator::rfidReceived, this);

    // Read the map
    requestMap();

    person_pf_ = new PersonParticleFilter(num_particles_, &map_, sigma_person, rfid_map_res, rfid_prob_pos, rfid_prob_neg);

    first_rob_pose_ = false;
    new_measure_ = false;
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

        person_pf_->resample();

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
        person_belief_pub_.publish(cloud_msg);
    }
}

/** Callback to receive the robot poses.

*/
void PersonEstimator::robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_pose_ = pose_msg->pose;

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
