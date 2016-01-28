#include <active_perception_controller/active_perception_interface1.h>
#include <pluginlib/class_list_macros.h>
#include <active_perception_controller/ActivePerceptionPlan.h>

PLUGINLIB_EXPORT_CLASS(active_perception_interface1::ActivePerceptionInterface, nav_core::BaseGlobalPlanner);

using namespace active_perception_interface1;

ActivePerceptionInterface::
ActivePerceptionInterface() :
nh_(),
motion_planner_srv_(nh_.serviceClient<active_perception_controller::ActivePerceptionPlan>("/robot_1/plan"))
{}

ActivePerceptionInterface::
ActivePerceptionInterface(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
nh_(),
motion_planner_srv_(nh_.serviceClient<active_perception_controller::ActivePerceptionPlan>("/robot_1/plan"))
{}

void
ActivePerceptionInterface::
initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
}

bool
ActivePerceptionInterface::
makePlan(const geometry_msgs::PoseStamped& start,
         const geometry_msgs::PoseStamped& goal,
         std::vector<geometry_msgs::PoseStamped>& plan)
{
    active_perception_controller::ActivePerceptionPlan req;
    motion_planner_srv_.call(req);

    plan.clear();
    for(size_t i = 0; i < req.response.path.poses.size(); i++)
    {
        plan.push_back(req.response.path.poses[i]);
    }
    return true;
}
