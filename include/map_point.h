/* ----------------------------------------------------------------------------
 * Copyright 2020, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "cvo_image.h"
#include <map>

#ifndef MAP_POINT_H_
#define MAP_POINT_H_

namespace cvo_slam
{

class Mappoint
{
public:
    Mappoint(int& id_, Point& position_, cv::Mat& normal_):id(id_), position(position_) {normal = normal_.clone();}
    ~Mappoint() {};

    cv::Mat GetNormal(){return normal.clone();}

    int EraseObservation(int keyframe_id){
        std::map<int,int>::iterator corres = keypoints_id.find(keyframe_id);
        if(corres == keypoints_id.end()){
            // std::cout << "The map point observation doesn't exist" << std::endl;
            return -1;
        } 
        else {
            int kp_id = corres->second;
            keypoints_id.erase(corres);
            return kp_id;
        } 
    }

public:
    int id;
    Point position;
    
    // Mean viewing direction
    cv::Mat normal;

    // Correspondence of visible keyframe and key point id 
    std::map<int,int> keypoints_id;
};

typedef boost::shared_ptr<Mappoint> MappointPtr;
typedef std::vector<MappointPtr> MappointVector;

} // namespace cvo_slam

#endif // MAP_POINT_H_