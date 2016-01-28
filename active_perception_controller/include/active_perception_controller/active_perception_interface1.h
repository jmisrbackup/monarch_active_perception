#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef ACTIVE_PERCEPTION_INTERFACE1_H
#define ACTIVE_PERCEPTION_INTERFACE1_H

namespace active_perception_interface1 {

class ActivePerceptionInterface : public nav_core::BaseGlobalPlanner {
public:
    ActivePerceptionInterface();
    ActivePerceptionInterface(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
private:
    ros::NodeHandle nh_;
    ros::ServiceClient motion_planner_srv_;
};


};
#endif
