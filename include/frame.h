/* ----------------------------------------------------------------------------
 * Copyright 2020, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef FRAME_H_
#define FRAME_H_

#include "cvo_image.h"

namespace cvo_slam
{

class Frame
{
public:
    Frame(std::string& timestamp_, Transform& relative_pose_):timestamp(timestamp_), relative_pose(relative_pose_) {};
    ~Frame() {};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    std::string timestamp;

    // pose with respect to corresponding keyframe pose
    Transform relative_pose;
};

typedef boost::shared_ptr<Frame> FramePtr;
typedef std::vector<FramePtr> FrameVector;


} // namespace cvo_slam

#endif