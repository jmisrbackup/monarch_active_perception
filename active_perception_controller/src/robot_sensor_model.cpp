/*
 * sensor_model_mask.cpp
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#include <active_perception_controller/robot_sensor_model.h>

using namespace ros;
using namespace std;
using namespace cv;
using namespace optimization;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

RobotSensorModel::
RobotSensorModel(const string& map_path,
                 const MapMetaData& map_metadata) :
sensor_map_(imread(map_path.c_str(), 1)),
map_metadata_(map_metadata)
{}

void
RobotSensorModel::
applySensorModel(const PointCloud& particle_set,
                 OccupancyGrid& map)
{
    map.header.frame_id = "/map";
    map.info = map_metadata_;
    map.data.resize(map.info.height*map.info.width,0);

    for(size_t i = 0; i < particle_set.points.size(); i++)
    {
        for(int x_h = -10; x_h <= 10; x_h++)
        {
            for(int y_h = -10; y_h <= 10; y_h++)
            {
                size_t px = floor((particle_set.points[i].x - map.info.origin.position.x)
                                  /map.info.resolution) + x_h;
                size_t py = floor((particle_set.points[i].y - map.info.origin.position.y)
                                  /map.info.resolution) + y_h;
                map.data[py*map.info.width + px] += 1;
            }
        }
    }
}

void
RobotSensorModel::
applySensorModel(const Point32& particle,
                 OccupancyGrid& map)
{

}
