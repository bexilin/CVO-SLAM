/* ----------------------------------------------------------------------------
 * Copyright 2019, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef TRACKING_RESULT_H
#define TRACKING_RESULT_H

#include "cvo_image.h"
#include <iostream>
#include <g2o/core/optimizable_graph.h>
#include "cvo.hpp"

namespace cvo_slam
{

class tracking_result : public g2o::OptimizableGraph::Data
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transform transform;
    Transform lc_prior;
    Transform lc_prior_pnpransac;
    // Transform prior;
    Information information;
    // Information pre_hessian;
    Information post_hessian;
    int dis_to_keyframe; // only used in r_keyframe, record the number of frames between the current frame and the current keyframe

    cvo::inn_p inn_pre; // innerproduct result of Identitiy transform
    cvo::inn_p inn_post; // innerproduct result of transformation after CVO registration
    cvo::inn_p inn_prior; // innerproduct result of relative trasformation between the pose estimation of two frames 
    cvo::inn_p inn_lc_prior; // innerproduct result of transformation given by indirect loop-closure detection

   
    int matches;  // number of matched features between two images
    float score; // similarity score between two images
    int inliers_svd; // number of cvo inliers using SVD transformation
    int inliers_pnpransac; // number of cvo inliers using　PnP RANSAC transformation

    cvo::inn_p inn_fixed_pcd; // squared norm of fixed CVO point cloud
    cvo::inn_p inn_moving_pcd; // squared norm of moving CVO point cloud
    float cos_angle; // cosine of angle betwwen CVO functions

    bool isNaN() const
    {
        return !std::isfinite(transform.matrix().sum()) || !std::isfinite(information.sum());
    }

    tracking_result()
    {
    }
    tracking_result(const tracking_result& result) ://  transform(result.transform),
                                                    //  lc_prior(result.lc_prior),
                                                    //  lc_prior_pnpransac(result.lc_prior_pnpransac),
                                                    // //  prior(result.prior),
                                                    //  information(result.information),
                                                    // //  pre_hessian(result.pre_hessian),
                                                    //  post_hessian(result.post_hessian),
                                                     inn_pre(result.inn_pre),
                                                     inn_post(result.inn_post),
                                                     inn_prior(result.inn_prior),
                                                     inn_lc_prior(result.inn_lc_prior),
                                                     matches(result.matches),
                                                     score(result.score),
                                                     inliers_svd(result.inliers_svd),
                                                     inliers_pnpransac(result.inliers_pnpransac),
                                                     inn_fixed_pcd(result.inn_fixed_pcd),
                                                     inn_moving_pcd(result.inn_moving_pcd),
                                                     cos_angle(result.cos_angle)
    {
        transform = result.transform;
        lc_prior = result.lc_prior;
        lc_prior_pnpransac = result.lc_prior_pnpransac;
        information = result.information;
        post_hessian = result.post_hessian;
    }

    virtual bool read(std::istream &is)
    {
        return true;
    }

    virtual bool write(std::ostream &os) const
    {
        return false;
    }

};

typedef std::unique_ptr<tracking_result> tracking_result_Ptr;

} // namespace cvo_slam

#endif // TRACKING_RESULT_H