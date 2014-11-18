/*
 * robot_sensor_model.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#ifndef ROBOT_SENSOR_MODEL_H_
#define ROBOT_SENSOR_MODEL_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <active_perception_controller/utility_map.h>

namespace optimization
{
class RobotSensorModel
{
public:
    RobotSensorModel(const std::string& lfield_path);

    void applySensorModel(size_t cell_x,
                          size_t cell_y,
                          UtilityMap<UtilityIntervalMapCell>* global_umap);
private:
    UtilityMap<UtilityIntervalMapCell>* lh_interval_map_;
};
}


#endif /* ROBOT_SENSOR_MODEL_H_ */
