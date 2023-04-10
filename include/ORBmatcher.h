/**
 * This is a modified version of ORBmatcher.h from ORB-SLAM2 (see below).
 * Changes: 1) remove some original member functions in class ORBmatcher; 
            2) remove some original header files;
            3) add a new function ORBmatcher::GetMatches;
            4) add PCL library header files;
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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d.hpp>
// #include<opencv2/line_descriptor.hpp>

#include<limits.h>

// #include <opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/transformation_estimation_svd.h>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <algorithm>
#include <random>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

#include "keyframe.h"
#include "vertex_and_edge.h"
#include "Converter.h"
#include "map_point.h"
// #include "ORBextractor.h"

// #include <boost/filesystem.hpp>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>


namespace ORB_SLAM2
{

typedef std::unique_ptr<g2o::SparseOptimizer> PoseGraphPtr;

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true, float UseRobustKernel=true, float RobustKernelDelta=5.0, int MinMatch=10);
    ~ORBmatcher();

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    bool GetInitialTransformation(bool check_map, cvo_slam::KeyframePtr& reference, cvo_slam::KeyframePtr& current, cvo_slam::KeyframeVector& keyframes, cvo_slam::MappointVector& map_points, g2o::SparseOptimizer& graph, int id_interval_, int& mappoint_vertex_id_, int& projection_edge_id_, \
                                  int& projection_num, vector<cv::KeyPoint>& reference_final_kp, vector<cv::KeyPoint>& current_final_kp, vector<cv::DMatch>& r_to_c_match, Eigen::Affine3d& T_r_to_c, int& matches, Eigen::Matrix<double,4,4>& relative_transform);

    void ResetPoseOptimizer();

    void AddPoseVertexToOptimizer(const g2o::VertexSE3ExpmapInv* frame_vertex);

    void CheckExistingMappoints(cvo_slam::KeyframePtr& keyframe, cvo_slam::MappointVector& map_points);

    void GetBestCovisibleKeyframeList(std::vector<int>& keyframe_list, int& farest_keyframe);

    void GetBestCovisibleKeyframeList(cvo_slam::KeyframePtr& reference);

    void ReleasePoseOptimizer();

    int ExistingMappoints() {return map_point_kp_pairs.size();}

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    g2o::RobustKernel* createRobustKernel();

    void CreateNewMapPoints(cvo_slam::MappointPtr& new_map_point, const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
                            g2o::SparseOptimizer& graph, const int& id_interval_, int& mappoint_vertex_id_, int& projection_edge_id_, int& projection_num, const cv::Mat& F_c_wrt_r, \
                            const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& r_inv_R, const cv::Mat& r_inv_t, const cv::Mat& r_inv_T, const cv::Mat& c_R, const cv::Mat& c_t, \
                            const cv::Mat& c_inv_R, const cv::Mat& c_inv_t, const cv::Mat& c_inv_T, const float& fx, const float& fy, const float& cx, const float& cy);

    void CreateNewMapPoints(cvo_slam::MappointPtr& new_map_point, const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
                            const int& id_interval_, int& mappoint_vertex_id_, const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& c_t, const g2o::Vector3D& reference_pc_inlier);

    bool CheckExistingMappointByProjection(cvo_slam::KeyframePtr& keyframe, cvo_slam::MappointPtr& map_point, int keypoint_id, \
                                            const cv::Mat& t, const cv::Mat& inv_R, const cv::Mat& inv_t, const cv::Mat& intrinsic);

    // bool CheckEpipolarConstraint(const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, const cv::Mat& F_c_wrt_r, \
    //                              const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& r_inv_R, const cv::Mat& r_inv_t, const cv::Mat& r_inv_T, const cv::Mat& c_R, const cv::Mat& c_t, \
    //                              const cv::Mat& c_inv_R, const cv::Mat& c_inv_t, const cv::Mat& c_inv_T, const float& fx, const float& fy, const float& cx, const float& cy);

    // void CreateNewMapPoint(cvo_slam::MappointVector& map_points, int r_kp_idx, int c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
    //                        int id_interval_, int& mappoint_vertex_id_);

    // void IncludeMappointIntoGraph(cvo_slam::MappointPtr& map_point, cvo_slam::KeyframeVector& keyframes, float fx, float fy, float cx, float cy, g2o::SparseOptimizer& graph, \
    //                               int id_interval_, int& projection_edge_id_, int& projection_num);

    // void UpdateVisibility(cvo_slam::MappointPtr& map_point, cvo_slam::KeyframePtr& keyframe, int kp_idx, g2o::SparseOptimizer& graph, \
    //                       int id_interval_, int& projection_edge_id_, int& projection_num);
    
    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const cvo_slam::KeyframePtr& pKF2);

    g2o::EdgeSE3ProjectionOnlyPose* AddProjectionEdgeToOptimizer(cvo_slam::KeyframePtr& keyframe, int kp_id, cvo_slam::MappointPtr& map_point);

    // g2o::EdgeSE3Projection* AddMappointAndEdgeToOptimizer(cvo_slam::KeyframePtr& keyframe, int kp_id, cvo_slam::MappointPtr& map_point);

    // g2o::EdgeSE3ProjectionOnlyPose* AddProjectionEdgeToOptimizer(const g2o::VertexPointXYZ* map_point_vertex, const cv::KeyPoint& kp, float info, const cv::Mat& intrinsic);

    // void drawLoopClosure_debug(const std::vector<cv::KeyPoint>& r_kp, const std::vector<cv::KeyPoint>& c_kp, const std::vector<cv::DMatch>& r_to_c_match, const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current);

    Eigen::Matrix<double,4,4> computeRigidTransformSVD(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst);

    Eigen::Matrix<double,4,4> optimizeRelativeTransformation(const vector<int>& current_kp_idx_inliers, const vector<g2o::Vector3D>& reference_pc_inliers, const cvo_slam::KeyframePtr& current, const Eigen::Matrix<double,4,4>& T_cr);

    float mfNNratio;
    bool mbCheckOrientation;
    bool UseRobustKernel_;
    float RobustKernelDelta_;
    int MinMatch_;

    PoseGraphPtr pose_optimizer;
    
    // potential pairs of edge of existing map points and key points in reference keyframe
    std::vector<std::pair<g2o::EdgeSE3ProjectionOnlyPose*,int>> map_point_kp_pairs;

    // number of covisibile current map points in previous keyframes
    std::map<int,int> keyframe_map_point_pairs;

    // pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ,double> PointCloudRegistrator;

    // // For SuperPoint implementation
    // PyObject*SPF_object, *read_img_function, *point_visualization_function;
};

}// namespace ORB_SLAM2

#endif // ORBMATCHER_H
