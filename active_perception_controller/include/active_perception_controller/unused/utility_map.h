/*
 * utility_map.h
 *
 *  Created on: Oct 27, 2014
 *      Author: jmessias
 */

#ifndef UTILITY_MAP_H_
#define UTILITY_MAP_H_

#include <utility>
#include <vector>
#include <stdexcept>

#include <boost/icl/closed_interval.hpp>
#include <boost/icl/interval_map.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace optimization
{

class UtilityCell
{
public:
    virtual ~UtilityCell() {};
    virtual double getMaximumUtility() const = 0;
};

class MaximumRangeCell: public UtilityCell
{
public:
    MaximumRangeCell(double utility = 0) :
    utility_(utility)
    {}

    MaximumRangeCell(const MaximumRangeCell*& cell)
    {
        utility_ = cell->getMaximumUtility();
    }

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

class EntropyMeasure
{
public:
    EntropyMeasure() :
    obs_prob_ (0),
    log_obs_prob_ (0),
    max_ (NULL)
    {}

    EntropyMeasure(double obs_prob, double* max) :
    obs_prob_(obs_prob),
    log_obs_prob_(log(obs_prob)), //needs to be fixed
    max_ (max)
    {}

    double getObsProb() const
    { return obs_prob_; }
    double getLogObsProb() const
    { return log_obs_prob_; }

    //TODO: correct this according to (14) of (Boers et. al)
    EntropyMeasure& operator += (const EntropyMeasure& e)
    {
        obs_prob_ += e.getObsProb();
        log_obs_prob_ += e.getLogObsProb();
        double val = evaluate();
        if( val > *max_)
            *max_ = val;

        return *this;
    }

    bool operator == (const EntropyMeasure& e) const
    { return evaluate() == e.evaluate(); }

    double evaluate() const
    {
        return log(obs_prob_) - 0.5*log_obs_prob_;
    }
private:
    double obs_prob_;
    double log_obs_prob_;
    double* max_;
};

class UtilityIntervalMapCell : public UtilityCell
{
public:
    UtilityIntervalMapCell() :
    ivmap_ (),
    max_val_ (DBL_MIN)
    {}

    UtilityIntervalMapCell(double upper_bound,
                           double lower_bound,
                           double obs_prob) :
    ivmap_ (),
    max_val_ (DBL_MIN)
    {
        boost::icl::continuous_interval<double> iv =
                boost::icl::continuous_interval<double>::closed(lower_bound, upper_bound);
        EntropyMeasure e(obs_prob, &max_val_);
        ivmap_ += make_pair(iv,e);
    }

    double getMaximumUtility() const
    {
        return max_val_;
    }

    void add(const UtilityIntervalMapCell* cell)
    {
        max_val_+=1;
//        boost::icl::interval_map<double, EntropyMeasure>::const_iterator it = cell->begin();
//        double val = max_val_;
//        if(ivmap_.size() > 0)
//            return;
        //ivmap_ += *it;
//
//        for(it = cell->begin(); it != cell->end(); it++)
//        {
//            ivmap_ += *it;
//        }
    }

    boost::icl::continuous_interval<double> getMaximumInterval() const
    {
        double max_val(DBL_MIN);
        boost::icl::continuous_interval<double> max_iv;
        for(boost::icl::interval_map<double, EntropyMeasure>::const_iterator it = begin();
            it != end();
            it++)
        {
            double val = it->second.evaluate();
            if(val > max_val)
            {
                max_val = val;
                max_iv = it->first;
            }
        }

        return max_iv;
    }

    boost::icl::interval_map<double, EntropyMeasure>::const_iterator
    begin() const
    {
        return ivmap_.begin();
    }

    boost::icl::interval_map<double, EntropyMeasure>::const_iterator
    end() const
    {
        return ivmap_.end();
    }

private:
    boost::icl::interval_map<double, EntropyMeasure> ivmap_;
    double max_val_;
};

template<class T>
class UtilityMap
{
public:
    typedef std::pair<size_t, size_t> CellIndex;
    typedef std::vector<CellIndex > IndexSequence;

    UtilityMap(size_t width, size_t height) :
    max_utility_(0)
    {
        map_.resize(width);
        for(size_t i = 0; i < height; i++)
        {
            map_[i].resize(width, NULL);
        }
    }

    ~UtilityMap()
    {
        clear();
    }

    void clear()
    {
        for(size_t i = 0; i < map_.size(); i++)
        {
            for(size_t j = 0; j < map_[i].size(); j++)
                if(map_[i][j])
                {
                    free(map_[i][j]);
                    map_[i][j] = NULL;
                }
        }
        ix_seq_.clear();
    }

    void compose(const CellIndex& idx, const UtilityMap<T>* umap)
    {
        compose(idx.first, idx.second, umap);
    }

    void compose(size_t x_c, size_t y_c, const UtilityMap<T>* umap)
    {
        size_t o_x = x_c - umap->getWidth()/2, o_y = y_c - umap->getHeight()/2;
        size_t i = 0;
        for(IndexSequence::const_iterator it = umap->cellIndicesBegin();
            it != umap->cellIndicesEnd();
            it++, i++)
        {
            {
//                if(x_c + it->first >= umap->getWidth()/2 &&
//                   y_c + it->second >= umap->getWidth()/2)
//                {
                        size_t x = it->first + o_x,
                               y = it->second + o_y;
//                    if(isInRange(x, y))
//                    {
                        if(map_[x][y] == NULL)
                        {
                            T* u = new T();
                            map_[x][y] = u;
                            ix_seq_.push_back(std::make_pair(x,y));
                        }

                        map_[x][y]->add((*umap)[*it]);
                        double val = map_[x][y]->getMaximumUtility();
                        if(val > max_utility_)
                            max_utility_ = val;
//                    }
//                }
            }
        }
    }

    void setUtilityCell(const CellIndex& idx, T* u)
    {
        setUtilityCell(idx.first, idx.second, u);
    }

    void setUtilityCell(size_t x, size_t y, T* u)
    {
        if(!isInRange(x, y))
        {
            std::out_of_range oor("Index out of range.");
            throw oor;
        }
        if(map_[x][y])
            free(map_[x][y]);
        else
            ix_seq_.push_back(std::make_pair(x,y));
        map_[x][y] = u;
    }

    const T* getUtilityCell(const CellIndex& idx) const
    {
        return getUtilityCell(idx.first, idx.second);
    }

    const T* getUtilityCell(size_t x, size_t y) const
    {
        if( !isInRange(x,y) )
        {
            std::out_of_range oor("Index out of range.");
            throw oor;
        }
        else
            return map_[x][y];
    }

    void getMaximumUtilityMap(nav_msgs::OccupancyGrid& occgrid) const
    {
        size_t width = getWidth(), height = getHeight();

        occgrid.data.clear();
        occgrid.data.resize(width * height);
        for(IndexSequence::const_iterator it = cellIndicesBegin();
            it != cellIndicesEnd();
            it++)
        {
            int8_t score = (*this)[*it]->getMaximumUtility()/max_utility_*127;
            occgrid.data[it->second*width + it->first] = score;
        }
    }

    size_t getWidth() const
    {
        return map_.size();
    }

    size_t getHeight() const
    {
        return map_[0].size();
    }

    bool isInRange(const CellIndex& idx) const
    {
        return isInRange(idx.first, idx.second);
    }

    bool isInRange(size_t x, size_t y) const
    {
        return ( x < map_.size() && y < map_[x].size() );
    }

    IndexSequence::const_iterator cellIndicesBegin() const
    {
        return ix_seq_.begin();
    }

    IndexSequence::const_iterator cellIndicesEnd() const
    {
        return ix_seq_.end();
    }

    const T* operator[](const CellIndex& idx) const
    {
        return getUtilityCell(idx);
    }
private:
    double max_utility_;

    std::vector<std::vector<T*> > map_;
    IndexSequence ix_seq_;
};
}




#endif /* UTILITY_MAP_H_ */
