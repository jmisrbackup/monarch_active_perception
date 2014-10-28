/*
 * utility_map.h
 *
 *  Created on: Oct 27, 2014
 *      Author: jmessias
 */

#ifndef UTILITY_MAP_H_
#define UTILITY_MAP_H_

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace optimization
{

class UtilityCell
{
public:

    virtual double getMaximumUtility() const = 0;
};

class MaximumRangeCell: public UtilityCell
{
public:
    MaximumRangeCell(double utility = 0) :
    utility_(utility)
    {}

    void add(const MaximumRangeCell* cell)
    {
        utility_ += cell->getMaximumUtility();
    }

    double getMaximumUtility() const
    {
        return utility_;
    }
private:
    double utility_;
};

class SensorBinaryMaskCell : public UtilityCell
{
    double getMaximumUtility() const
    {
        return 0;
    }
};

template<class T>
class UtilityMap
{
public:
    UtilityMap(size_t width, size_t height) :
    max_utility_(0)
    {
        map_.resize(width);
        for(size_t i = 0; i < height; i++)
        {
            map_[i].resize(width, NULL);
            for(size_t j = 0; j < height; j++)
            {
                map_[i][j] = new T();
            }
        }
    }

    ~UtilityMap()
    {
        for(size_t i = 0; i < map_.size(); i++)
        {
            for(size_t j = 0; j < map_[i].size(); j++)
                free(map_[i][j]);
            map_[i].clear();
        }
        map_.clear();
    }

    void compose(size_t x_o, size_t y_o, const UtilityMap<T>& umap)
    {
        for(size_t x = 0;
            x < umap.getWidth() && x + x_o < map_.getWidth();
            x++)
        {
            for(size_t y = 0;
                y < umap.getHeight() && y + y_o < map_.getHeight();
                y++)
            {
                map_[x + x_o][y + y_o]->add(umap[x][y]);
            }
        }
    }

    const UtilityCell* getUtilityCell(size_t x, size_t y)
    {
        if( !isInRange(x,y) )
            return NULL;
        else
            return map_[x][y];
    }

    void getMaximumUtilityMap(nav_msgs::OccupancyGrid& map);
    size_t getWidth()
    {
        return map_.size();
    }

    size_t getHeight()
    {
        return map_[0].size();
    }

    bool isInRange(size_t x, size_t y)
    {
        return ( x >= map_.size() || y >= map_[x].size());
    }
private:
    double max_utility_;

    std::vector<std::vector<T*> > map_;
};
}




#endif /* UTILITY_MAP_H_ */
