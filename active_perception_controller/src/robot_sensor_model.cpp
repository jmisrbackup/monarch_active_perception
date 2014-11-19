/*
 * robot_sensor_model.cpp
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
RobotSensorModel(const string& lfield_path) :
lh_interval_map_(NULL)
{
    cv::Mat model = cv::imread(lfield_path, CV_LOAD_IMAGE_GRAYSCALE), dst;

    size_t width = model.cols, height = model.rows;
    cv::Mat test(height, width, CV_8UC1);
    if(width % 2 == 0 || height % 2 == 0)
    {
        ROS_FATAL("The sensor model image must contain an odd number of rows and columns.");
        ROS_FATAL("The center pixel corresponds to the position of the sensor.");
        abort();
    }

    lh_interval_map_ = new UtilityMap<UtilityIntervalMapCell>(width, height);

    cv::threshold( model, dst, 0.9*255, 0.9, 0);

    cv::namedWindow( "spiral", CV_WINDOW_AUTOSIZE );

    //Finding the farthest non-zero pixel (inward spiral search)
    bool found = false;
    double range = 0;
    for(size_t w = 0, h = 0, total = 0;
        w < (width-1)/2.0 && h < (height-1)/2.0 && !found;
        w++, h++)
    {
        double theta = M_PI/2.0;
        long int ctheta = 0, stheta = 1;
        size_t x = w, y = h;

        for(size_t l = 0, k = 0, step = 0;
            l < 2*(width-2*w)+2*(height-2*h) && total < width*height && !found;
            l+=1, total++)
        {
            if(ctheta == 0)
                step = width-2*w;
            else
                step = height-2*h;
            if( l - k > step-1 )
            {
                k = l;
                theta -= M_PI/2.0;
                ctheta = (long int) round(cos(theta));
                stheta = (long int) round(sin(theta));
            }
            x += ctheta;
            y += stheta;
            if(*(model.data + model.step*y + x*model.elemSize()) > 128)
            {
                found = true;
                ROS_INFO_STREAM("found at " << x << " " << y);
                range = hypot((double) x-(width-1)/2.0, (double) y-(height-1)/2.0);
            }
        }
    }

    ROS_INFO_STREAM("range is " << range);

//    for(double r = 0; r <= 1.5; r += map_metadata_.resolution)
//    {
//        for(double th = 0; th < 2*M_PI; th += atan(map_metadata_.resolution/3.0))
//        {
//            UtilityMap<MaximumRangeCell>::CellIndex idx(floor(r*cos(th)/map_metadata_.resolution) + 15,
//                                                        floor(r*sin(th)/map_metadata_.resolution) + 15);
//            sensor_map_.setUtilityCell(idx,c);
//        }
//    }
}

void
RobotSensorModel::
applySensorModel(size_t cell_x,
                 size_t cell_y,
                 UtilityMap<UtilityIntervalMapCell>* global_umap)
{
    UtilityMap<UtilityIntervalMapCell>::CellIndex cell_idx(cell_x,cell_y);
    global_umap->compose(cell_idx, lh_interval_map_);
}
