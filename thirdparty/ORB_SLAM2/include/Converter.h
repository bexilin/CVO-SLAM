/**
 * This is a modified version of Converter.h from ORB-SLAM2 (see below).
 * Changes: 1) remove some original member functions in class Converter; 
            2) remove some original header files;
            3) add new functions Converter::toCvMat and Converter::toAffine.
 * Date: Nov 2019
 * Xi Lin, Dingyi Sun
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<Eigen/Core>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/eigen_types.h>

namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static cv::Mat toCvMat(const Eigen::Affine3d &m);
    static cv::Mat toCvMat(const g2o::SE3Quat &m);
    static cv::Mat toCvMat(const g2o::Vector3D& m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static Eigen::Affine3d toAffine(const Eigen::Matrix<double,4,4> &m);
    // static Eigen::Affine3d toAffine(const cv::Mat &m);
    static Eigen::Matrix<double,3,3> toEigenMat(const cv::Mat &m);
    static g2o::Vector3D toEigenVector(const cv::Mat &m);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
