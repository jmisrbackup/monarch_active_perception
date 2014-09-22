#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"
#include "particle_filter.h"

#include <string>

using namespace std;

/**
This class implements a particle filter to estimate the position of a person. Measurements from a mobile RFID reader
on board a robot are used.
*/
class PersonEstimator
{
  protected:

    ros::NodeHandle nh;
    ros::NodeHandle *private_nh;
    ros::Publisher person_belief_pub;   /// To publish person pose belief
    ros::Subscriber robot_pose_sub; 	/// To get robot position
    ros::Subscriber robot_cloud_sub; 	/// To get robot position particles
    ros::Subscriber rfid_sub;           /// To get RFID measurements

    double step_duration;               /// Step duration in seconds
    int num_particles;                  /// Number of particles of the filter
    string global_frame_id;             /// Global frame id

    ParticleFilter *person_pf;              /// Particle filter for person
    geometry_msgs::PoseArray robot_cloud;   /// Particles for robot pose
    double robot_x;                         /// Robot position
    double robot_y;
    bool first_rob_pose;                    /// First robot pose arrived
    bool new_measure;                       /// Indicates new measure arrived
    bool rfid_mes;                          /// RFID measurement
    bool filter_running;                    /// True when the filter is initialized

    /// Callback to receive the robot poses
    void robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    /// Callback to receive the robot particles
    void robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg);
    /// Callback to receive the RFID measures
    void rfidReceived(const std_msgs::BoolConstPtr& rfid_msg);

  public:
  PersonEstimator();
  ~PersonEstimator();
  void runIteration();
  double getStepDuration();
};

/** Constructor
*/
PersonEstimator::PersonEstimator()
{
    private_nh = new ros::NodeHandle("~");

    // Read parameters
    private_nh->param("step_duration", step_duration, 0.2);
    private_nh->param("num_particles", num_particles, 5000);
    private_nh->param("global_frame_id", global_frame_id, string("map"));

    // Subscribe/advertise topics
    person_belief_pub = nh.advertise<geometry_msgs::PoseArray>("person_pose_cloud", 1);
    robot_pose_sub = nh.subscribe("amcl_pose", 1, &PersonEstimator::robotPoseReceived, this);
    robot_cloud_sub = nh.subscribe("particlecloud", 1, &PersonEstimator::robotCloudReceived, this);
    rfid_sub = nh.subscribe("rfid", 1, &PersonEstimator::rfidReceived, this);

    person_pf = new ParticleFilter(num_particles);
    first_rob_pose = false;
    new_measure = false;
    filter_running = false;

}

/// Destructor
PersonEstimator::~PersonEstimator()
{
    delete private_nh;
    delete person_pf;
}

/// Get the step duration
double PersonEstimator::getStepDuration()
{
    return step_duration;
}

/** Callback to receive the robot poses.

*/
void PersonEstimator::robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_x = pose_msg->pose.pose.position.x;
    robot_y = pose_msg->pose.pose.position.y;
}

/** Callback to receive the robot cloud.

*/
void PersonEstimator::robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg)
{
    robot_cloud = *pose_msg;
}

/** Callback to receive the RFID.

*/
void PersonEstimator::rfidReceived(const std_msgs::BoolConstPtr& rfid_msg)
{
    new_measure = true;
    rfid_mes = rfid_msg->data;
}

/** This method runs an iteration of the particle filter. Prediction and update.
*/
void PersonEstimator::runIteration()
{
    bool publish_data = false;

    // Initialize filter once first robot pose is available
    if(!filter_running && first_rob_pose)
    {
        person_pf->initUniform();
        publish_data = true;
    }
    else if(filter_running)
    {
        // Predict
        person_pf->predict(step_duration);
        // Update if new measures
        if(new_measure)
        {
            person_pf->update(rfid_mes, robot_cloud);
            new_measure = false;
        }
        else
        {
            // When there is no leacture from RFID, consider a negative measure
            rfid_mes = false;
            person_pf->update(rfid_mes, robot_cloud);
        }
        publish_data = true;
    }

    // Publish belief
    if(publish_data)
    {
        geometry_msgs::PoseArray cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = global_frame_id;
        cloud_msg.poses.resize(person_pf->getNumParticles());

        // Retreive particles
        for(int i = 0; i < person_pf->getNumParticles(); i++)
        {
            vector<double> position = person_pf->getParticlePose(i);
            cloud_msg.poses[i].position.x = position[0];
            cloud_msg.poses[i].position.y = position[1];
        }

        person_belief_pub.publish(cloud_msg);
    }
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
