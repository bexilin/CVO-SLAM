/* ----------------------------------------------------------------------------
 * Copyright 2019, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef CVO_IMAGE_H
#define CVO_IMAGE_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
// #include <ros/time.h>
#include "vertex_and_edge.h"

// #include <Python.h>
// // #include <numpy/ndarrayobject.h>
// #include <ndarrayobject.h>

namespace cvo_slam{

typedef Eigen::Affine3d Transform; // data type for pose estimation
typedef Eigen::Matrix<double, 6, 6> Information; // data type for information matrix
typedef g2o::SE3Quat Transform_Vertex; // data type for pose estimation in graph
typedef Eigen::Matrix3d Rotation; // data type for rotation part of pose estimation
typedef g2o::Vector3D Point; // data type for map point 3D coordinate

struct Image
{
    cv::Mat rgb;
    cv::Mat depth;
    std::string timestamp;

    // For SuperPoint implementation
    std::string rgb_path;
};

typedef boost::shared_ptr<Image> Image_Ptr;

namespace internal
{

static cvo_slam::Transform_Vertex toSE3Quat(const cvo_slam::Transform& pose)
{
  cvo_slam::Transform_Vertex p(pose.rotation(), pose.translation());

  return p;
}

static cvo_slam::Transform toAffine(const cvo_slam::Transform_Vertex& pose)
{
  cvo_slam::Transform p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

} // namespace internal

} // namespace cvo_slam

#endif // CVO_IMAGE_H