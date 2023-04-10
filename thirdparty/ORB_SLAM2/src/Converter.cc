/**
 * This is a modified version of Converter.cc from ORB-SLAM2 (see below).
 * Changes: 1) remove definition of some original member functions in class Converter; 
            2) add definition of functions Converter::toCvMat and Converter::toAffine.
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


#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

cv::Mat Converter::toCvMat(const Eigen::Affine3d &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &m)
{
    Eigen::Matrix<double,4,4> EigenMat = m.to_homogeneous_matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=EigenMat(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const g2o::Vector3D& m)
{
    cv::Mat cvMat(3,1,CV_32F);
    cvMat.at<float>(0) = m(0);
    cvMat.at<float>(1) = m(1);
    cvMat.at<float>(2) = m(2);
    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

Eigen::Affine3d Converter::toAffine(const Eigen::Matrix<double,4,4> &m)
{
    Eigen::Affine3d affineMat = Eigen::Affine3d::Identity();
    affineMat.matrix() = m;
    return affineMat;
}

// Eigen::Affine3d Converter::toAffine(const cv::Mat &m)
// {
//     Eigen::Matrix<double,4,4> EigenMat;
//     for(int i=0;i<4;i++)
//         for(int j=0; j<4; j++)
//             EigenMat(i,j)=m.at<double>(i,j);

//     Eigen::Affine3d affineMat;
//     affineMat.matrix() = EigenMat;
//     return affineMat;
// }

Eigen::Matrix<double,3,3> Converter::toEigenMat(const cv::Mat &m)
{
    Eigen::Matrix<double,3,3> EigenMat;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            EigenMat(i,j)=m.at<float>(i,j);
    
    return EigenMat;
}

g2o::Vector3D Converter::toEigenVector(const cv::Mat &m)
{
    g2o::Vector3D EigenVector(m.at<float>(0),m.at<float>(1),m.at<float>(2));
    return EigenVector;
}

} //namespace ORB_SLAM
