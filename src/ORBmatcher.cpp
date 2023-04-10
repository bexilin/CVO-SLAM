/**
 * This is a modified version of ORBmatcher.cc from ORB-SLAM2 (see below).
 * Changes: 1) remove definition of some original member functions in class Converter; 
            2) add definition of function ORBmatcher::GetMatches.
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

#include "ORBmatcher.h"

// #include<stdint-gcc.h>
#include <stdint.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri, float UseRobustKernel, float RobustKernelDelta, int MinMatch): mfNNratio(nnratio), mbCheckOrientation(checkOri), 
                                                                                                      UseRobustKernel_(UseRobustKernel), 
                                                                                                      RobustKernelDelta_(RobustKernelDelta),
                                                                                                      MinMatch_(MinMatch),
                                                                                                      pose_optimizer(nullptr)
{
    // // For SuperPoint implementation
    // Py_Initialize();
    // PyRun_SimpleString("import sys");
    // PyRun_SimpleString("import os");
    // PyRun_SimpleString("sys.path.append(os.getcwd())");
    // PyObject* demo_superpoint = PyImport_ImportModule("demo_superpoint");
    // if(demo_superpoint == NULL){
    //     std::cout << "Fail to import module demo_superpoint" << std::endl;
    //     PyErr_Print();
    //     exit(1); 
    // }

    // // Get SuperPointFrontend (called SPF below) class
    // PyObject* SPF = PyObject_GetAttrString(demo_superpoint,"SuperPointFrontend");
    // if(SPF == NULL){
    //     std::cout << "Fail to get SPF" << std::endl;
    //     PyErr_Print();
    //     exit(1); 
    // }

    // // Create a SPF object
    // PyObject* SPF_init_args = Py_BuildValue("(siffi)","superpoint_v1.pth",4,0.015,0.7,1);
    // if(SPF_init_args == NULL){
    //     std::cout << "Fail to get SPF_init_args" << std::endl;
    //     PyErr_Print();
    //     exit(1); 
    // }
    // SPF_object = PyObject_CallObject(SPF, SPF_init_args);
    // if(SPF_object == NULL){
    //     std::cout << "Fail to initialize SPF_object" << std::endl;
    //     PyErr_Print();
    //     exit(1); 
    // }

    // // Get image reading function
    // read_img_function = PyObject_GetAttrString(demo_superpoint,"read_original_image");
    // if(read_img_function == NULL){
    //     std::cout << "Fail to get read_img_function" << std::endl;
    //     PyErr_Print();
    //     exit(1); 
    // }

    // // Get point visualization function
    // point_visualization_function = PyObject_GetAttrString(demo_superpoint,"visualize_points_and_matches");
    // if(point_visualization_function == NULL){
    //     std::cout << "Fail to get point_visualization_function" << std::endl;
    //     PyErr_Print();
    //     exit(1);
    // }
}

ORBmatcher::~ORBmatcher()
{
    // Py_Finalize();
}

bool ORBmatcher::GetInitialTransformation(bool check_map, cvo_slam::KeyframePtr& reference, cvo_slam::KeyframePtr& current, cvo_slam::KeyframeVector& keyframes, cvo_slam::MappointVector& map_points, g2o::SparseOptimizer& graph, int id_interval_, int& mappoint_vertex_id_, int& projection_edge_id_, \
                                          int& projection_num, vector<cv::KeyPoint>& reference_final_kp, vector<cv::KeyPoint>& current_final_kp, vector<cv::DMatch>& r_to_c_match, Eigen::Affine3d& T_r_to_c, int& matches, Eigen::Matrix<double,4,4>& relative_transform)
{
    /** 
    * In this function we reference to ORB-SLAM2 by Raúl et al.
    * https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L524-654
    **/
    
    vector<cv::Point2f> reference_matched_kp_1;
    vector<cv::Point2f> current_matched_kp_1;

    vector<int> reference_matched_kp_idx_1;
    vector<int> current_matched_kp_idx_1;

    vector<cv::Point2f> reference_matched_kp_2;
    vector<cv::Point2f> current_matched_kp_2;

    vector<int> reference_matched_kp_idx_2;
    vector<int> current_matched_kp_idx_2;

    vector<float> discriptor_distance_1;
    vector<float> discriptor_distance_2;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = reference->FeatVec.begin();
    DBoW2::FeatureVector::const_iterator f2it = current->FeatVec.begin();
    DBoW2::FeatureVector::const_iterator f1end = reference->FeatVec.end();
    DBoW2::FeatureVector::const_iterator f2end = current->FeatVec.end();

    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                const cv::Mat &d1 = reference->descriptors.row(idx1);

                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    const cv::Mat &d2 = current->descriptors.row(idx2);

                    int dist = DescriptorDistance(d1,d2);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        reference_matched_kp_1.push_back(reference->keypoints[idx1].pt);
                        current_matched_kp_1.push_back(current->keypoints[bestIdx2].pt);

                        reference_matched_kp_idx_1.push_back(idx1);
                        current_matched_kp_idx_1.push_back(bestIdx2);

                        discriptor_distance_1.push_back(static_cast<float>(bestDist1));

                        assert(reference_matched_kp_1.size() == current_matched_kp_1.size());
                        int match_id = reference_matched_kp_1.size()-1;

                        if(mbCheckOrientation)
                        {
                            float rot = reference->keypoints[idx1].angle-current->keypoints[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(match_id);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = reference->FeatVec.lower_bound(f2it->first);
        }
        else
        {
            f2it = current->FeatVec.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3){
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    reference_matched_kp_2.push_back(reference_matched_kp_1[rotHist[i][j]]);
                    current_matched_kp_2.push_back(current_matched_kp_1[rotHist[i][j]]);

                    reference_matched_kp_idx_2.push_back(reference_matched_kp_idx_1[rotHist[i][j]]);
                    current_matched_kp_idx_2.push_back(current_matched_kp_idx_1[rotHist[i][j]]);

                    discriptor_distance_2.push_back(discriptor_distance_1[rotHist[i][j]]);

                    assert(reference_matched_kp_2.size() == current_matched_kp_2.size());
                }
            }
            else{
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    nmatches--;
            }
        }
    }
    else{
        reference_matched_kp_2 = reference_matched_kp_1;
        current_matched_kp_2 = current_matched_kp_1;

        reference_matched_kp_idx_2 = reference_matched_kp_idx_1;
        current_matched_kp_idx_2 = current_matched_kp_idx_1;

        discriptor_distance_2 = discriptor_distance_1;
    }
    
    // std::cout << "initial match threshold: " << MinMatch_ << std::endl;
    std::cout << "Pairs of initially matched features: " << nmatches << std::endl;
    if(nmatches < MinMatch_){
        matches = 0;
        std::cout << "Initial transformation: Reject, not enough pairs of initially matched features" << std::endl;
        return false;
    } 

    // // For SuperPoint implementation
    // // Call method nn_match_two_way in SPF class
    // PyArrayObject* match_idx = (PyArrayObject *)PyObject_CallMethod(SPF_object,"nn_match_two_way","(OO)",current->SP_keypoints_and_descriptors,reference->SP_keypoints_and_descriptors);
    // if(match_idx == NULL){
    //     std::cout << "Fail to get matches from PyArrayObject" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }

    // int columns = match_idx->dimensions[1];
    // std::cout << "Pairs of initially matched features: " << columns << std::endl;
    // if(columns < MinMatch_){
    //     matches = 0;
    //     std::cout << "Initial transformation: Reject, not enough pairs of initially matched features" << std::endl;
    //     return false;
    // }

    // // Visualize matches
    // // Read image
    // PyObject* img_path_current = Py_BuildValue("(s)", current->rgb_path.c_str());
    // // PyObject* img_path = PyTuple_Pack(1,RGB_pth.c_str());
    // if(img_path_current == NULL){
    //     std::cout << "Fail to get img_path_current" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }

    // PyObject* img_current = PyObject_CallObject(read_img_function,img_path_current);
    // if(img_current == NULL){
    //     std::cout << "Fail to get img_current" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }

    // PyObject* img_path_reference = Py_BuildValue("(s)", reference->rgb_path.c_str());
    // // PyObject* img_path = PyTuple_Pack(1,RGB_pth.c_str());
    // if(img_path_reference == NULL){
    //     std::cout << "Fail to get img_path_reference" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }

    // PyObject* img_reference = PyObject_CallObject(read_img_function,img_path_reference);
    // if(img_reference == NULL){
    //     std::cout << "Fail to get img_reference" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }

    // PyObject* visualization_args = Py_BuildValue("(OOOOO)", current->SP_keypoints_and_descriptors,reference->SP_keypoints_and_descriptors, match_idx, img_current, img_reference);
    // if(visualization_args == NULL){
    //     std::cout << "Fail to get visualization_args" << std::endl;
    //     PyErr_Print();
    //     return false; 
    // }
    // PyObject_CallObject(point_visualization_function,visualization_args);

    // PyArrayObject *reference_points = (PyArrayObject *)PyList_GetItem(reference->SP_keypoints_and_descriptors, 0);
    // PyArrayObject *current_points = (PyArrayObject *)PyList_GetItem(current->SP_keypoints_and_descriptors, 0);

    // for(int j=0; j<columns; j++){
    //     // current keypoint coordinate
    //     int cur_idx = (int)*(double *)(match_idx->data + j * match_idx->strides[1]);
    //     float cur_x = (float)*(double *)(current_points->data + cur_idx * current_points->strides[1]);
    //     float cur_y = (float)*(double *)(current_points->data + current_points->strides[0] + cur_idx * current_points->strides[1]);
    //     cv::Point2f cur_kp(cur_x,cur_y);
    //     current_matched_kp_2.push_back(cur_kp);

    //     // reference keypoint coordinate
    //     int ref_idx = (int)*(double *)(match_idx->data + match_idx->strides[0] + j * match_idx->strides[1]);
    //     float ref_x = (float)*(double *)(reference_points->data + ref_idx * reference_points->strides[1]);
    //     float ref_y = (float)*(double *)(reference_points->data + reference_points->strides[0] + ref_idx * reference_points->strides[1]);
    //     cv::Point2f ref_kp(ref_x,ref_y);
    //     reference_matched_kp_2.push_back(ref_kp);
    // }

    // cv::Mat mask;
    // cv::Mat homography = cv::findHomography(reference_matched_kp_2, current_matched_kp_2, CV_RANSAC, 3.0, mask);

    // intrinsic
    cv::Mat intrinsic = current->intrinsic;
    float fx = intrinsic.at<float>(0,0);
    float fy = intrinsic.at<float>(1,1);
    float cx = intrinsic.at<float>(0,2);
    float cy = intrinsic.at<float>(1,2);

    // use a PnPRANSAC solver to solve relative transformation between two frames to check inliers
    // Solve it twice 

    // Create 3D point clouds with both keyframes
    vector<int> reference_matched_kp_idx_3;
    vector<int> current_matched_kp_idx_3;
    // vector<float> discriptor_distance_3;
    vector<cv::Point3f> current_pc;
    vector<cv::Point2f> reference_kp;
    vector<cv::Point3f> reference_pc;
    vector<cv::Point2f> current_kp;
    // vector<cv::KeyPoint> r_kp;
    // vector<cv::KeyPoint> c_kp;

    for(std::vector<cv::Point2f>::size_type i = 0; i < reference_matched_kp_2.size(); i++)
    {
        float r_dep = reference->ImDepth.at<float>(reference_matched_kp_2[i].y,reference_matched_kp_2[i].x);
        float c_dep = current->ImDepth.at<float>(current_matched_kp_2[i].y,current_matched_kp_2[i].x);
        if(r_dep > 0 && c_dep > 0){
            float c_x = (current_matched_kp_2[i].x - cx) * c_dep / fx;
            float c_y = (current_matched_kp_2[i].y - cy) * c_dep / fy;
            cv::Point3f c_p(c_x, c_y, c_dep);
            current_pc.push_back(c_p);
            reference_kp.push_back(reference_matched_kp_2[i]);

            // cv::KeyPoint kp_c(current_matched_kp_2[i],1);
            // c_kp.push_back(kp_c);

            float r_x = (reference_matched_kp_2[i].x - cx) * r_dep / fx;
            float r_y = (reference_matched_kp_2[i].y - cy) * r_dep / fy;
            cv::Point3f r_p(r_x, r_y, r_dep);
            reference_pc.push_back(r_p);
            current_kp.push_back(current_matched_kp_2[i]);

            // cv::KeyPoint kp_r(reference_matched_kp_2[i],1);
            // r_kp.push_back(kp_r);

            reference_matched_kp_idx_3.push_back(reference_matched_kp_idx_2[i]);
            current_matched_kp_idx_3.push_back(current_matched_kp_idx_2[i]);
            // discriptor_distance_3.push_back(discriptor_distance_2[i]);
        }
    }

    // // Create 3D point clouds with reference keyframe
    // vector<int> reference_matched_kp_idx_3_2;
    // vector<int> current_matched_kp_idx_3_2;
    // vector<float> discriptor_distance_3_2;
    // vector<cv::Point3f> reference_pc;
    // vector<cv::Point2f> current_kp;

    // for(std::vector<cv::Point2f>::size_type i = 0; i < reference_matched_kp_2.size(); i++)
    // {
    //     float c_dep = reference->ImDepth.at<float>(reference_matched_kp_2[i].y,reference_matched_kp_2[i].x);
    //     if(c_dep >0){
    //         float c_x = (reference_matched_kp_2[i].x - cx) * c_dep / fx;
    //         float c_y = (reference_matched_kp_2[i].y - cy) * c_dep / fy;
    //         cv::Point3f c_p(c_x, c_y, c_dep);
    //         reference_pc.push_back(c_p);
    //         current_kp.push_back(current_matched_kp_2[i]);

    //         reference_matched_kp_idx_3_2.push_back(reference_matched_kp_idx_2[i]);
    //         current_matched_kp_idx_3_2.push_back(current_matched_kp_idx_2[i]);
    //         discriptor_distance_3_2.push_back(discriptor_distance_2[i]);
    //     }
    // }

    std::cout << "Points in the corresponding 3D point clouds: " << current_pc.size() << std::endl;
    // std::cout << "Points in the reference 3D point cloud: " << reference_pc.size() << std::endl;

    if(current_pc.size() < MinMatch_){
        matches = 0;
        std::cout << "Initial transformation: Reject, not enough number of points in 3D point clouds" << std::endl;
        return false;
    }

    // Run RANSAC to find inlier matches whose 2D warping errors and 3D reprojection errors are less than 3 respectively
    int max_inliers = 0;
    vector<int> max_inliers_idx;
    Eigen::Matrix<double,4,4> best_T_cr;

    int all_matches = current_pc.size();
    for(int ransac_count = 0; ransac_count < 100; ransac_count++){
        
        // Select 4 random samples
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, all_matches-1);
        std::set<int> selection;
        vector<cv::Point2f> reference_samples_2d;
        vector<cv::Point2f> current_samples_2d;

        // // use in SVD
        // pcl::PointCloud<pcl::PointXYZ>::Ptr reference_samples_3d(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::PointCloud<pcl::PointXYZ>::Ptr current_samples_3d(new pcl::PointCloud<pcl::PointXYZ>());

        std::vector<Eigen::Vector3d> reference_samples_3d;
        std::vector<Eigen::Vector3d> current_samples_3d;

        // // used in PnP
        // vector<cv::Point3f> reference_samples_3d_2;
        // vector<cv::Point3f> current_samples_3d_2;

        while(selection.size() < 4){
            int rand_int = dis(gen);
            // std::cout << "rand int: " << rand_int << std::endl;
            if(selection.find(rand_int) == selection.end()){
                selection.insert(rand_int);
                reference_samples_2d.push_back(reference_kp[rand_int]);
                current_samples_2d.push_back(current_kp[rand_int]);

                // pcl::PointXYZ r_point;
                // r_point.x = reference_pc[rand_int].x;
                // r_point.y = reference_pc[rand_int].y;
                // r_point.z = reference_pc[rand_int].z;

                // reference_samples_3d->points.push_back(r_point);

                // // reference_samples_3d_2.push_back(reference_pc[rand_int]);

                // pcl::PointXYZ c_point;
                // c_point.x = current_pc[rand_int].x;
                // c_point.y = current_pc[rand_int].y;
                // c_point.z = current_pc[rand_int].z;

                // current_samples_3d->points.push_back(c_point);

                // current_samples_3d_2.push_back(current_pc[rand_int]);

                Eigen::Vector3d r_point;
                r_point << reference_pc[rand_int].x, reference_pc[rand_int].y, reference_pc[rand_int].z;
                reference_samples_3d.push_back(r_point);

                Eigen::Vector3d c_point;
                c_point << current_pc[rand_int].x, current_pc[rand_int].y, current_pc[rand_int].z;
                current_samples_3d.push_back(c_point);
            } 
        }
        // std::cout << std::endl;

        // solve the homography and rigid body transformation that align 2d and 3d samples
        // std::cout << "here 1" << std::endl;
        cv::Mat homography = cv::findHomography(current_samples_2d,reference_samples_2d);
        if(homography.empty()) continue;

        // std::cout << "here 2" << std::endl;
        vector<cv::Point2f> current_kp_in_r;
        cv::perspectiveTransform(current_kp,current_kp_in_r,homography);

        // Use SVD to estimate relative transformation
        Eigen::Matrix<double,4,4> T_cr = computeRigidTransformSVD(current_samples_3d, reference_samples_3d);
        // PointCloudRegistrator.estimateRigidTransformation(*current_samples_3d, *reference_samples_3d, T_cr);
        cv::Mat T = Converter::toCvMat(T_cr);
        cv::Mat R = T.rowRange(0,3).colRange(0,3);
        cv::Mat inv_R = R.t();
        cv::Mat t = T.rowRange(0,3).col(3);
        cv::Mat inv_t = -inv_R * t;

        // // Use PnP to estimate relative transformation
        // cv::Mat R_vector, t;
        // cv::solvePnP(current_samples_3d_2, reference_samples_2d, intrinsic, cv::Mat(), R_vector, t, false, cv::SOLVEPNP_P3P);
        // cv::Mat R;
        // cv::Rodrigues(R_vector, R);
        // Eigen::Matrix<double,4,4> T_cr = Eigen::Matrix<double,4,4>::Identity();
        // for(int row=0; row<3; row++){
        //     for(int col=0; col<3; col++){
        //         T_cr(row,col) = R.at<double>(row,col);
        //     }
        //     T_cr(row,3) = t.at<double>(row,0);
        // }
        // cv::Mat inv_R = R.t();
        // cv::Mat inv_t = -inv_R * t;

        // // kd tree to guarantee that inlier matches are not too close
        // PointCloud<float> keypoints_r;
        // keypoint_kd_tree kp_kd_tree_r(3, keypoints_r, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        // PointCloud<float> keypoints_c;
        // keypoint_kd_tree kp_kd_tree_c(3, keypoints_c, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        // SearchParams search_params;
        // search_params.sorted = false;
        // int inliers_num = 0;

        // First check if current matched features are very close to an inlier match, if not then
        // check 2D warping errors and 3D reprojection errors for all matches
        vector<int> inliers_idx;
        for(int i = 0; i < all_matches; i++){
            
            // PointCloud<float>::Point kp_r;
            // kp_r.x = reference_kp[i].x;
            // kp_r.y = reference_kp[i].y;
            // kp_r.z = 0.0;

            // PointCloud<float>::Point kp_c;
            // kp_c.x = current_kp[i].x;
            // kp_c.y = current_kp[i].y;
            // kp_c.z = 0.0;

            // // find nearest neighbour in reference and current kd tree
            // if(inliers_num>0){
            //     float kp_query_r[3] = {kp_r.x, kp_r.y, kp_r.z};
            //     float kp_query_c[3] = {kp_c.x, kp_c.y, kp_c.z};

            //     size_t idx_r;
            //     float distance_r;
            //     KNNResultSet<float> resultSet_r(1);
            //     resultSet_r.init(&idx_r, &distance_r);
            //     kp_kd_tree_r.findNeighbors(resultSet_r, kp_query_r, search_params);

            //     if(distance_r < 5.0) continue;
        
            //     size_t idx_c;
            //     float distance_c;
            //     KNNResultSet<float> resultSet_c(1);
            //     resultSet_c.init(&idx_c, &distance_c);
            //     kp_kd_tree_c.findNeighbors(resultSet_c, kp_query_c, search_params);

            //     if(distance_c < 5.0) continue;
            // }
            
            // // warp keypoints in current keyframe to reference keyframe
            // cv::Mat c_2d(3,1,CV_32F);
            // c_2d.at<float>(0) = current_kp[i].x;
            // c_2d.at<float>(1) = current_kp[i].y;
            // c_2d.at<float>(2) = 1.0;

            // std::cout << homography.at<float>(0,0) << " " << homography.at<float>(0,1) << " " << homography.at<float>(0,2) << std::endl;
            // std::cout << homography.at<float>(1,0) << " " << homography.at<float>(1,1) << " " << homography.at<float>(1,2) << std::endl;
            // std::cout << homography.at<float>(2,0) << " " << homography.at<float>(2,1) << " " << homography.at<float>(2,2) << std::endl;

            // std::cout << "here 3" << std::endl;
            // cv::Mat c_2d_in_r = homography * c_2d;
            // std::cout << "here 4" << std::endl;
            // float c_2d_in_r_x = c_2d_in_r.at<float>(0) / c_2d_in_r.at<float>(2);
            // float c_2d_in_r_y = c_2d_in_r.at<float>(1) / c_2d_in_r.at<float>(2);
            float x_error_2d = current_kp_in_r[i].x - reference_kp[i].x;
            float y_error_2d = current_kp_in_r[i].y - reference_kp[i].y;

            if(x_error_2d*x_error_2d + y_error_2d*y_error_2d > 9.0) continue;

            // reproject keypoints in current keyframe into reference keyframe
            cv::Mat c_3d(3,1,CV_32F);
            c_3d.at<float>(0) = current_pc[i].x;
            c_3d.at<float>(1) = current_pc[i].y;
            c_3d.at<float>(2) = current_pc[i].z;

            // std::cout << "here 5" << std::endl;
            cv::Mat c_3d_in_r = R * c_3d + t;
            cv::Mat c_3d_in_r_projection = intrinsic * c_3d_in_r;
            // std::cout << "here 6" << std::endl;
            float c_3d_in_r_projection_x = c_3d_in_r_projection.at<float>(0) / c_3d_in_r_projection.at<float>(2);
            float c_3d_in_r_projection_y = c_3d_in_r_projection.at<float>(1) / c_3d_in_r_projection.at<float>(2);
            float x_error_3d_1 = c_3d_in_r_projection_x - reference_kp[i].x;
            float y_error_3d_1 = c_3d_in_r_projection_y - reference_kp[i].y;

            if(x_error_3d_1*x_error_3d_1 + y_error_3d_1*y_error_3d_1 > 64.0) continue;

            // reproject keypoints in reference keyframe into current keyframe
            cv::Mat r_3d(3,1,CV_32F);
            r_3d.at<float>(0) = reference_pc[i].x;
            r_3d.at<float>(1) = reference_pc[i].y;
            r_3d.at<float>(2) = reference_pc[i].z;

            cv::Mat r_3d_in_c = inv_R * r_3d + inv_t;
            cv::Mat r_3d_in_c_projection = intrinsic * r_3d_in_c;
            float r_3d_in_c_projection_x = r_3d_in_c_projection.at<float>(0) / r_3d_in_c_projection.at<float>(2);
            float r_3d_in_c_projection_y = r_3d_in_c_projection.at<float>(1) / r_3d_in_c_projection.at<float>(2);
            float x_error_3d_2 = r_3d_in_c_projection_x - current_kp[i].x;
            float y_error_3d_2 = r_3d_in_c_projection_y - current_kp[i].y;

            if(x_error_3d_2*x_error_3d_2 + y_error_3d_2*y_error_3d_2 > 64.0) continue;

            // Add matched features to kd trees, and store the index
            // keypoints_r.pts.push_back(kp_r);
            // kp_kd_tree_r.addPoints(inliers_num,inliers_num);
            // keypoints_c.pts.push_back(kp_c);
            // kp_kd_tree_c.addPoints(inliers_num,inliers_num);
            
            // inliers_num++;
            inliers_idx.push_back(i);
        }

        if(inliers_idx.size() > max_inliers){
            max_inliers = inliers_idx.size();
            max_inliers_idx = inliers_idx;
            best_T_cr = T_cr;
        }
    }

    std::cout << "Number of inliers: " << max_inliers << std::endl;
    if(max_inliers < MinMatch_){
        matches = 0;
        std::cout << "Initial transformation: Reject, not enough number of inliers" << std::endl;
        return false;
    }
    else{
        matches = max_inliers;
        std::cout << "Initial transformation: Accept" << std::endl;

        // vector<cv::KeyPoint> r_kp_2;
        // vector<cv::KeyPoint> c_kp_2;

        // for(int i=0; i<max_inliers_idx.size(); i++){
        //     r_kp_2.push_back(r_kp[max_inliers_idx[i]]);
        //     c_kp_2.push_back(c_kp[max_inliers_idx[i]]);
        // }

        // drawLoopClosure_debug(r_kp_2,c_kp_2,r_to_c_match,reference,current);
    }

    vector<int> reference_matched_kp_idx_4;
    vector<int> current_matched_kp_idx_4;
    vector<int> current_kp_idx_inliers;
    vector<g2o::Vector3D> reference_pc_inliers;

    for (int i = 0; i < max_inliers_idx.size(); i++) {
        int idx = max_inliers_idx[i];

        reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
        current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);

        current_kp_idx_inliers.push_back(current_matched_kp_idx_3[idx]);
        g2o::Vector3D point(reference_pc[idx].x,reference_pc[idx].y,reference_pc[idx].z);
        reference_pc_inliers.push_back(point);
        
    //     cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //     cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //     reference_final_kp.push_back(ref_kp);
    //     current_final_kp.push_back(cur_kp);

    //     cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //     // r_to_c_match.push_back(r_to_c);

    //     // store corresponding point clouds
    //     pcl::PointXYZ r_point;
    //     r_point.x = reference_pc[idx].x;
    //     r_point.y = reference_pc[idx].y;
    //     r_point.z = reference_pc[idx].z;

    //     reference_inliers_pc->points.push_back(r_point);

    //     pcl::PointXYZ c_point;
    //     c_point.x = current_pc[idx].x;
    //     c_point.y = current_pc[idx].y;
    //     c_point.z = current_pc[idx].z;

    //     current_inliers_pc->points.push_back(c_point);
    }

    // relative_transform = best_T_cr;

    // Optimize relative transformation with all inliers
    relative_transform = optimizeRelativeTransformation(current_kp_idx_inliers, reference_pc_inliers, current, best_T_cr);




    // else if(current_pc.size() >= MinMatch_ && reference_pc.size() < MinMatch_){
    //     std::cout << "Only current point clouds have enough points" << std::endl;
        
    //     cv::Mat R, t, inliers;
    //     cv::solvePnPRansac(current_pc, reference_kp, intrinsic, cv::Mat(), R, t, false, 100, 8.0, 0.99, inliers);

    //     std::cout << "Inliers in PnP Ransac solution: " << inliers.rows << std::endl;
    //     if(inliers.rows < MinMatch_){
    //         matches = 0;
    //         std::cout << "Initial transformation: Reject, not enough number of inliers in PnP Ransac solution" << std::endl;
    //         return false;
    //     }
    //     else{
    //         matches = inliers.rows;
    //         std::cout << "Initial transformation: Accept" << std::endl;
    //     }

    //     cv::Mat R_matrix;
    //     cv::Rodrigues(R, R_matrix);

    //     Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //     for(int row=0; row<3; row++){
    //         for(int col=0; col<3; col++){
    //             T(row,col) = R_matrix.at<double>(row,col);
    //         }
    //         T(row,3) = t.at<double>(row,0);
    //     }

    //     T_r_to_c.matrix() = T;

    //     for (int i = 0; i < inliers.rows; i++) {
    //         int idx = inliers.at<int>(i, 0);

    //         reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //         current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
            
    //         cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //         cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //         reference_final_kp.push_back(ref_kp);
    //         current_final_kp.push_back(cur_kp);

    //         cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //         // r_to_c_match.push_back(r_to_c);

    //         // store corresponding point clouds
    //         pcl::PointXYZ r_point;
    //         r_point.x = reference_pc[idx].x;
    //         r_point.y = reference_pc[idx].y;
    //         r_point.z = reference_pc[idx].z;

    //         reference_inliers_pc->points.push_back(r_point);

    //         pcl::PointXYZ c_point;
    //         c_point.x = current_pc[idx].x;
    //         c_point.y = current_pc[idx].y;
    //         c_point.z = current_pc[idx].z;

    //         current_inliers_pc->points.push_back(c_point);
    //     }
    // }
    // else if(current_pc.size() < MinMatch_ && reference_pc.size() >= MinMatch_){
    //     std::cout << "Only reference point clouds have enough points" << std::endl;

    //     cv::Mat R, t, inliers;
    //     cv::solvePnPRansac(reference_pc, current_kp, intrinsic, cv::Mat(), R, t, false, 100, 8.0, 0.99, inliers);

    //     std::cout << "Inliers in PnP Ransac solution: " << inliers.rows << std::endl;
    //     if(inliers.rows < MinMatch_){
    //         matches = 0;
    //         std::cout << "Initial transformation: Reject, not enough number of inliers in PnP Ransac solution" << std::endl;
    //         return false;
    //     }
    //     else{
    //         matches = inliers.rows;
    //         std::cout << "Initial transformation: Accept" << std::endl;
    //     }

    //     cv::Mat R_matrix;
    //     cv::Rodrigues(R, R_matrix);

    //     cv::Mat inv_R_matrix = R_matrix.t();
    //     cv::Mat inv_t = -inv_R_matrix * t;

    //     Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //     for(int row=0; row<3; row++){
    //         for(int col=0; col<3; col++){
    //             T(row,col) = inv_R_matrix.at<double>(row,col);
    //         }
    //         T(row,3) = inv_t.at<double>(row,0);
    //     }

    //     T_r_to_c.matrix() = T;

    //     for (int i = 0; i < inliers.rows; i++) {
    //         int idx = inliers.at<int>(i, 0);

    //         reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //         current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
            
    //         cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //         cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //         reference_final_kp.push_back(ref_kp);
    //         current_final_kp.push_back(cur_kp);

    //         cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //         // r_to_c_match.push_back(r_to_c);

    //         // store corresponding point clouds
    //         pcl::PointXYZ r_point;
    //         r_point.x = reference_pc[idx].x;
    //         r_point.y = reference_pc[idx].y;
    //         r_point.z = reference_pc[idx].z;

    //         reference_inliers_pc->points.push_back(r_point);

    //         pcl::PointXYZ c_point;
    //         c_point.x = current_pc[idx].x;
    //         c_point.y = current_pc[idx].y;
    //         c_point.z = current_pc[idx].z;

    //         current_inliers_pc->points.push_back(c_point);
    //     }
    // }
    // else{
    //     std::cout << "Both point clouds have enough points" << std::endl;

    //     cv::Mat R, t, inliers;
    //     cv::solvePnPRansac(current_pc, reference_kp, intrinsic, cv::Mat(), R, t, false, 100, 8.0, 0.99, inliers);

    //     cv::Mat R_2, t_2, inliers_2;
    //     cv::solvePnPRansac(reference_pc, current_kp, intrinsic, cv::Mat(), R_2, t_2, false, 100, 8.0, 0.99, inliers_2);

    //     std::cout << "Inliers in current point cloud PnP Ransac solution: " << inliers.rows << std::endl;
    //     std::cout << "Inliers in reference point cloud PnP Ransac solution: " << inliers_2.rows << std::endl;

    //     if(inliers.rows < MinMatch_ && inliers_2.rows < MinMatch_){
    //         matches = 0;
    //         std::cout << "Initial transformation: Reject, not enough number of inliers in both PnP Ransac solutions" << std::endl;
    //         return false;
    //     }
    //     else if(inliers.rows >= MinMatch_ && inliers_2.rows < MinMatch_){
    //         matches = inliers.rows;
    //         std::cout << "Initial transformation: Accept, only current point cloud PnP Ransac solution has enough number of inliers" << std::endl;

    //         cv::Mat R_matrix;
    //         cv::Rodrigues(R, R_matrix);

    //         Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //         for(int row=0; row<3; row++){
    //             for(int col=0; col<3; col++){
    //                 T(row,col) = R_matrix.at<double>(row,col);
    //             }
    //             T(row,3) = t.at<double>(row,0);
    //         }

    //         T_r_to_c.matrix() = T;

    //         for (int i = 0; i < inliers.rows; i++) {
    //             int idx = inliers.at<int>(i, 0);

    //             reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //             current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
                
    //             cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //             cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //             reference_final_kp.push_back(ref_kp);
    //             current_final_kp.push_back(cur_kp);

    //             cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //             // r_to_c_match.push_back(r_to_c);

    //             // store corresponding point clouds
    //             pcl::PointXYZ r_point;
    //             r_point.x = reference_pc[idx].x;
    //             r_point.y = reference_pc[idx].y;
    //             r_point.z = reference_pc[idx].z;

    //             reference_inliers_pc->points.push_back(r_point);

    //             pcl::PointXYZ c_point;
    //             c_point.x = current_pc[idx].x;
    //             c_point.y = current_pc[idx].y;
    //             c_point.z = current_pc[idx].z;

    //             current_inliers_pc->points.push_back(c_point);
    //         }
    //     }
    //     else if(inliers.rows < MinMatch_ && inliers_2.rows >= MinMatch_){
    //         matches = inliers_2.rows;
    //         std::cout << "Initial transformation: Accept, only reference point cloud PnP Ransac solution has enough number of inliers" << std::endl;

    //         cv::Mat R_matrix;
    //         cv::Rodrigues(R_2, R_matrix);

    //         cv::Mat inv_R_matrix = R_matrix.t();
    //         cv::Mat inv_t = -inv_R_matrix * t_2;

    //         Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //         for(int row=0; row<3; row++){
    //             for(int col=0; col<3; col++){
    //                 T(row,col) = inv_R_matrix.at<double>(row,col);
    //             }
    //             T(row,3) = inv_t.at<double>(row,0);
    //         }

    //         T_r_to_c.matrix() = T;

    //         for (int i = 0; i < inliers_2.rows; i++) {
    //             int idx = inliers_2.at<int>(i, 0);

    //             reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //             current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
                
    //             cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //             cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //             reference_final_kp.push_back(ref_kp);
    //             current_final_kp.push_back(cur_kp);

    //             cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //             // r_to_c_match.push_back(r_to_c);

    //             // store corresponding point clouds
    //             pcl::PointXYZ r_point;
    //             r_point.x = reference_pc[idx].x;
    //             r_point.y = reference_pc[idx].y;
    //             r_point.z = reference_pc[idx].z;

    //             reference_inliers_pc->points.push_back(r_point);

    //             pcl::PointXYZ c_point;
    //             c_point.x = current_pc[idx].x;
    //             c_point.y = current_pc[idx].y;
    //             c_point.z = current_pc[idx].z;

    //             current_inliers_pc->points.push_back(c_point);
    //         }
    //     }
    //     else{
    //         std::cout << "Initial transformation: Accept, both PnP Ransac solution have enough number of inliers" << std::endl;

    //         if(inliers.rows >= inliers_2.rows){
    //             std::cout << "current point cloud PnP Ransac solution has more inliers" << std::endl;
    //             matches = inliers.rows;

    //             cv::Mat R_matrix;
    //             cv::Rodrigues(R, R_matrix);

    //             Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //             for(int row=0; row<3; row++){
    //                 for(int col=0; col<3; col++){
    //                     T(row,col) = R_matrix.at<double>(row,col);
    //                 }
    //                 T(row,3) = t.at<double>(row,0);
    //             }

    //             T_r_to_c.matrix() = T;

    //             for (int i = 0; i < inliers.rows; i++) {
    //                 int idx = inliers.at<int>(i, 0);

    //                 reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //                 current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
                    
    //                 cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //                 cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //                 reference_final_kp.push_back(ref_kp);
    //                 current_final_kp.push_back(cur_kp);

    //                 cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //                 // r_to_c_match.push_back(r_to_c);

    //                 // store corresponding point clouds
    //                 pcl::PointXYZ r_point;
    //                 r_point.x = reference_pc[idx].x;
    //                 r_point.y = reference_pc[idx].y;
    //                 r_point.z = reference_pc[idx].z;

    //                 reference_inliers_pc->points.push_back(r_point);

    //                 pcl::PointXYZ c_point;
    //                 c_point.x = current_pc[idx].x;
    //                 c_point.y = current_pc[idx].y;
    //                 c_point.z = current_pc[idx].z;

    //                 current_inliers_pc->points.push_back(c_point);
    //             }
    //         }
    //         else{
    //             std::cout << "reference point cloud PnP Ransac solution has more inliers" << std::endl;
    //             matches = inliers_2.rows;

    //             cv::Mat R_matrix;
    //             cv::Rodrigues(R_2, R_matrix);

    //             cv::Mat inv_R_matrix = R_matrix.t();
    //             cv::Mat inv_t = -inv_R_matrix * t_2;

    //             Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    //             for(int row=0; row<3; row++){
    //                 for(int col=0; col<3; col++){
    //                     T(row,col) = inv_R_matrix.at<double>(row,col);
    //                 }
    //                 T(row,3) = inv_t.at<double>(row,0);
    //             }

    //             T_r_to_c.matrix() = T;

    //             for (int i = 0; i < inliers_2.rows; i++) {
    //                 int idx = inliers_2.at<int>(i, 0);

    //                 reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //                 current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
                    
    //                 cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //                 cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //                 reference_final_kp.push_back(ref_kp);
    //                 current_final_kp.push_back(cur_kp);

    //                 cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //                 // r_to_c_match.push_back(r_to_c);

    //                 // store corresponding point clouds
    //                 pcl::PointXYZ r_point;
    //                 r_point.x = reference_pc[idx].x;
    //                 r_point.y = reference_pc[idx].y;
    //                 r_point.z = reference_pc[idx].z;

    //                 reference_inliers_pc->points.push_back(r_point);

    //                 pcl::PointXYZ c_point;
    //                 c_point.x = current_pc[idx].x;
    //                 c_point.y = current_pc[idx].y;
    //                 c_point.z = current_pc[idx].z;

    //                 current_inliers_pc->points.push_back(c_point);
    //             }
    //         }
    //     }
    // }

    // vector<int> reference_matched_kp_idx_4;
    // vector<int> current_matched_kp_idx_4;

    // for (int i = 0; i < inliers.rows; i++) {
    //     int idx = inliers.at<int>(i, 0);

    //     reference_matched_kp_idx_4.push_back(reference_matched_kp_idx_3[idx]);
    //     current_matched_kp_idx_4.push_back(current_matched_kp_idx_3[idx]);
        
    //     cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_3[idx]];
    //     cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_3[idx]];

    //     reference_final_kp.push_back(ref_kp);
    //     current_final_kp.push_back(cur_kp);

    //     cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_3[idx]);
    //     // r_to_c_match.push_back(r_to_c);
    // }

    // cv::Mat R_matrix;
    // cv::Rodrigues(R, R_matrix);

    // Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    // for(int row=0; row<3; row++){
    //     for(int col=0; col<3; col++){
    //         T(row,col) = R_matrix.at<double>(row,col);
    //     }
    //     T(row,3) = t.at<double>(row,0);
    // }

    // T_r_to_c.matrix() = T;

    // if(check_map){
    //     int new_edge_of_map_points = 0;

    //     for(int i=0; i<reference_matched_kp_idx_4.size(); i++){
    //         std::map<int,int>::iterator correspondence = current->mappoints_id.find(current_matched_kp_idx_4[i]);

    //         if(correspondence == current->mappoints_id.end()){
    //         // If the matched features do not correspond to an existing map point, create a new map point
    //             CreateNewMapPoint(map_points, reference_matched_kp_idx_4[i], current_matched_kp_idx_4[i], reference, current, \
    //                               id_interval_, mappoint_vertex_id_);
    //         }
    //         else{
    //             cvo_slam::MappointPtr map_point = map_points[(correspondence->second - 1)/2];
    //             if(map_point->in_graph){
    //                 // If map point is already in the graph (visible in at least 3 keyframes), update visibility
    //                 UpdateVisibility(map_point, reference, reference_matched_kp_idx_4[i], graph, id_interval_, projection_edge_id_, projection_num);
    //                 new_edge_of_map_points++;
    //             }
    //             else{
    //                 IncludeMappointIntoGraph(map_point, keyframes, fx, fy, cx, cy, graph, id_interval_, projection_edge_id_, projection_num);
    //                 map_point->in_graph = true;
    //                 UpdateVisibility(map_point, reference, reference_matched_kp_idx_4[i], graph, id_interval_, projection_edge_id_, projection_num);
    //                 new_edge_of_map_points += 3;
    //             }
    //         }
    //     }

    //     std::cout << "new edge of map points: " << new_edge_of_map_points << std::endl;
    // }

    // reference and current frame poses (used for map point creation)
    g2o::VertexSE3ExpmapInv* reference_vertex = (g2o::VertexSE3ExpmapInv*) graph.vertex(reference->id);
    g2o::VertexSE3ExpmapInv* current_vertex = (g2o::VertexSE3ExpmapInv*) graph.vertex(current->id);

    cv::Mat r_inv_pose = Converter::toCvMat(reference_vertex->estimate());
    cv::Mat r_inv_R = r_inv_pose.rowRange(0,3).colRange(0,3);
    cv::Mat r_R = r_inv_R.t();
    cv::Mat r_inv_t = r_inv_pose.rowRange(0,3).col(3);
    cv::Mat r_t = -r_inv_R.t() * r_inv_t;
    cv::Mat r_inv_T(3,4,CV_32F);
    r_inv_R.copyTo(r_inv_T.colRange(0,3));
    r_inv_t.copyTo(r_inv_T.col(3));

    cv::Mat c_inv_pose = Converter::toCvMat(current_vertex->estimate());
    cv::Mat c_inv_R = c_inv_pose.rowRange(0,3).colRange(0,3);
    cv::Mat c_R = c_inv_R.t();
    cv::Mat c_inv_t = c_inv_pose.rowRange(0,3).col(3);
    cv::Mat c_t = -c_inv_R.t() * c_inv_t;
    cv::Mat c_inv_T(3,4,CV_32F);
    c_inv_R.copyTo(c_inv_T.colRange(0,3));
    c_inv_t.copyTo(c_inv_T.col(3));

    // Compute fundamental matrix
    cv::Mat R_c_wrt_r = r_inv_R * c_inv_R.t();
    cv::Mat t_c_wrt_r = -r_inv_R * c_inv_R.t() * c_inv_t + r_inv_t;
    
    cv::Mat t_c_wrt_r_skew = (cv::Mat_<float>(3,3) << 0.0, -t_c_wrt_r.at<float>(2), t_c_wrt_r.at<float>(1),
                                                      t_c_wrt_r.at<float>(2), 0.0, -t_c_wrt_r.at<float>(0),
                                                      -t_c_wrt_r.at<float>(1), t_c_wrt_r.at<float>(0), 0.0);

    cv::Mat F_c_wrt_r = (reference->intrinsic).t().inv() * t_c_wrt_r_skew * R_c_wrt_r * (current->intrinsic).inv();

    // ResetPoseOptimizer();
    // AddPoseVertexToOptimizer(reference_vertex);
    // std::vector<std::pair<g2o::EdgeSE3ProjectionOnlyPose*,int>> edge_kp_id_pairs;
    // int new_edge_of_new_map_point = 0;
    // int new_edge_of_existing_map_point = 0;

    int new_map_points = 0;
    int potential_existing_map_points = 0;
    int existing_map_points = 0;

    // Iterate over all inliers
    for(int i=0; i<reference_matched_kp_idx_4.size(); i++){
        std::map<int,int>::iterator corres_reference = reference->mappoints_id.find(reference_matched_kp_idx_4[i]);
        std::map<int,int>::iterator corres_current = current->mappoints_id.find(current_matched_kp_idx_4[i]);

        if(corres_reference == reference->mappoints_id.end() && corres_current == current->mappoints_id.end()){
            // If there are already enough number of map points seen in either reference or current keyframe, do not create a new map point
            if(reference->mappoints_id.size() >= 500 || current->mappoints_id.size() >= 500) continue;

            // If the matched features do not correspond to an existing map point, try creating a new map point
            cvo_slam::MappointPtr new_map_point(nullptr);
            CreateNewMapPoints(new_map_point, reference_matched_kp_idx_4[i], current_matched_kp_idx_4[i], reference, \
                            current, graph, id_interval_, mappoint_vertex_id_, projection_edge_id_, projection_num, \
                            F_c_wrt_r, r_R, r_t, r_inv_R, r_inv_t, r_inv_T, c_R, c_t, c_inv_R, c_inv_t, c_inv_T, fx, fy, cx, cy);
            if(new_map_point.get() != nullptr){
                map_points.push_back(new_map_point);
                std::map<int,int>::iterator kf_iter = keyframe_map_point_pairs.find(current->id);
                if(kf_iter == keyframe_map_point_pairs.end()) keyframe_map_point_pairs.insert({current->id,1});
                else keyframe_map_point_pairs[kf_iter->first]++;
                new_map_points++;
            }
            // CreateNewMapPoints(new_map_point, reference_matched_kp_idx_4[i], current_matched_kp_idx_4[i], reference, current, \
            //                    id_interval_, mappoint_vertex_id_, r_R, r_t, c_t, reference_pc_inliers[i]);
            // map_points.push_back(new_map_point);
            // std::map<int,int>::iterator kf_iter = keyframe_map_point_pairs.find(current->id);
            // if(kf_iter == keyframe_map_point_pairs.end()) keyframe_map_point_pairs.insert({current->id,1});
            // else keyframe_map_point_pairs[kf_iter->first]++;
            // new_map_points++; 
        }
        else if(corres_reference == reference->mappoints_id.end() && corres_current != current->mappoints_id.end()){
            // If there are already enough number of map points seen in reference keyframe, do not check visibility of new map points
            if(reference->mappoints_id.size() >= 500) continue;

            // If the matched feature in current keyframe correspond to an existing map point, add the map point to the potential list
            cvo_slam::MappointPtr map_point = map_points[(corres_current->second - 1)/2];
            assert(map_point->id == corres_current->second);
            // // std::cout << "map point id: " << map_point->id << std::endl;
            // g2o::EdgeSE3ProjectionOnlyPose* edge = AddProjectionEdgeToOptimizer(reference, reference_matched_kp_idx_4[i], map_point);
            // map_point_kp_pairs.push_back({edge,reference_matched_kp_idx_4[i]});

            potential_existing_map_points++;

            if(CheckExistingMappointByProjection(reference, map_point, reference_matched_kp_idx_4[i], r_t, r_inv_R, r_inv_t, intrinsic)){
                existing_map_points++;
            }
        }
        else if(corres_reference != reference->mappoints_id.end() && corres_current == current->mappoints_id.end()){
            // Matched feature in reference keyframe ready correspond to an existing map point, but that in current keyframe doesn't,
            // we do not deal with this case here    
        }
        else{
            // both features in reference and current keyframe correspond to an existing map point, they may correspond to different
            // map points, then merging map points may be considerred 
        }
    }

    std::cout << "new_map_points: " << new_map_points << std::endl;
    std::cout << "potential_existing_map_points: " << potential_existing_map_points << std::endl;
    std::cout << "existing_map_points: " << existing_map_points << std::endl << std::endl;

    // // Iterate over all inliers
    // for(int i=0; i<reference_matched_kp_idx_4.size(); i++){
    //     std::map<int,int>::iterator correspondence = current->mappoints_id.find(current_matched_kp_idx_4[i]);

    //     if(correspondence == current->mappoints_id.end()){
    //         // If the matched features do not correspond to an existing map point, try creating a new map point
    //         cvo_slam::MappointPtr new_map_point(nullptr);
    //         CreateNewMapPoints(new_map_point, reference_matched_kp_idx_4[i], current_matched_kp_idx_4[i], reference, \
    //                         current, graph, id_interval_, mappoint_vertex_id_, projection_edge_id_, projection_num, \
    //                         F_c_wrt_r, r_R, r_t, r_inv_R, r_inv_t, r_inv_T, c_R, c_t, c_inv_R, c_inv_t, c_inv_T, fx, fy, cx, cy);
    //         if(new_map_point.get() != nullptr){
    //             map_points.push_back(new_map_point);
    //             new_edge_of_new_map_point += 2;
    //         } 
    //     }
    //     else{
    //         // // Otherwise, check epipolar constraints, if succeeds create a new projection edge and update visibility
    //         // if(CheckEpipolarConstraint(reference_matched_kp_idx_4[i], current_matched_kp_idx_4[i], reference, current, \
    //         //                 F_c_wrt_r, r_R, r_t, r_inv_R, r_inv_t, r_inv_T, c_R, c_t, c_inv_R, c_inv_t, c_inv_T, fx, fy, cx, cy)){
    //         //     g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) graph.vertex(correspondence->second);
    //         //     assert(map_point_vertex != nullptr);

    //         //     int map_point_id = map_point_vertex->id();
    //         //     cvo_slam::MappointPtr map_point = map_points[(map_point_id - 1)/2];
    //         //     assert(map_point->id == map_point_id);
    //         //     map_point->keypoints_id.insert({reference->id, reference_matched_kp_idx_4[i]});
    //         //     reference->mappoints_id.insert({reference_matched_kp_idx_4[i], map_point_id});

    //         //     // create a new projection edge
    //         //     g2o::EdgeSE3Projection* reference_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(reference->intrinsic));
                
    //         //     cv::KeyPoint kp = reference->keypoints[reference_matched_kp_idx_4[i]];
    //         //     g2o::Vector2D r_xy(kp.pt.x, kp.pt.y);
    //         //     reference_edge->setId(projection_edge_id_);

    //         //     projection_edge_id_ += id_interval_;
    //         //     projection_num++;
    //         //     new_edge_of_existing_map_point++;

    //         //     reference_edge->setMeasurement(r_xy);
    //         //     reference_edge->setRobustKernel(createRobustKernel());
    //         //     reference_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * reference->mvInvLevelSigma2[kp.octave]);
    //         //     reference_edge->resize(2);
    //         //     reference_edge->setLevel(0);
    //         //     reference_edge->setVertex(0,graph.vertex(map_point_id));
    //         //     reference_edge->setVertex(1,graph.vertex(reference->id));

    //         //     graph.addEdge(reference_edge);
    //         // }

    //         g2o::EdgeSE3ProjectionOnlyPose* edge = AddProjectionEdgeToOptimizer(map_point_vertex, kp, info, reference->intrinsic);
            
    //         edge_kp_id_pairs.push_back(std::make_pair(edge,reference_matched_kp_idx_4[i]));
    //     }
    // }

    // // if reference keyframe initially corresponds to enough number of existing map points,
    // // perform pose optimization to determine map points that are probably visible.    
    // // if(edge_kp_id_pairs.size() >= MinMatch_){
    // //     g2o::VertexSE3ExpmapInv* v = (g2o::VertexSE3ExpmapInv*) pose_optimizer->vertex(0);
    // //     std::vector<bool> inliers(edge_kp_id_pairs.size(),true);
    // //     bool enough_inliers = true;
        
    // //     for(size_t i = 0; i < 4; i++){
    // //         v->setEstimate(reference_vertex->estimate());
    // //         pose_optimizer->initializeOptimization(0);
    // //         pose_optimizer->optimize(10);

    // //         int inliers_count = 0;
    // //         for(size_t j = 0; j < edge_kp_id_pairs.size(); j++){
    // //             g2o::EdgeSE3ProjectionOnlyPose* edge = edge_kp_id_pairs[j].first;
    // //             if(!inliers[j]) edge->computeError();
    // //             float chi2 = edge->chi2();
    // //             if(chi2 > 5.991){
    // //                 inliers[j] = false;
    // //                 edge->setLevel(1);
    // //             }
    // //             else{
    // //                 inliers[j] = true;
    // //                 edge->setLevel(0);
    // //                 inliers_count++;
    // //             }
    // //         }

    // //         if(inliers_count < MinMatch_){
    // //             enough_inliers = false;
    // //             break;
    // //         } 
    // //     }

    // //     if(enough_inliers){
    // //         for(size_t k = 0; k < edge_kp_id_pairs.size(); k++){
    // //             if(inliers[k]){
    // //                 int map_point_id = (edge_kp_id_pairs[k].first)->id();
    // //                 int kp_id = edge_kp_id_pairs[k].second;

    // //                 cvo_slam::MappointPtr map_point = map_points[(map_point_id - 1)/2];
    // //                 assert(map_point->id == map_point_id);
    // //                 map_point->keypoints_id.insert({reference->id, kp_id});
    // //                 reference->mappoints_id.insert({kp_id, map_point_id});

    // //                 // create a new projection edge
    // //                 g2o::EdgeSE3Projection* reference_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(reference->intrinsic));
                    
    // //                 cv::KeyPoint kp = reference->keypoints[kp_id];
    // //                 g2o::Vector2D r_xy(kp.pt.x, kp.pt.y);
    // //                 reference_edge->setId(projection_edge_id_);

    // //                 projection_edge_id_ += id_interval_;
    // //                 projection_num++;
    // //                 new_edge_of_existing_map_point++;

    // //                 reference_edge->setMeasurement(r_xy);
    // //                 reference_edge->setRobustKernel(createRobustKernel());
    // //                 reference_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * reference->mvInvLevelSigma2[kp.octave]);
    // //                 reference_edge->resize(2);
    // //                 reference_edge->setLevel(0);
    // //                 reference_edge->setVertex(0,graph.vertex(map_point_id));
    // //                 reference_edge->setVertex(1,graph.vertex(reference->id));

    // //                 graph.addEdge(reference_edge);
    // //             }
    // //         }
    // //     }
    // // }

    // std::cout << "edge_kp_id_pairs num: " << edge_kp_id_pairs.size() << std::endl;
    // std::cout << "new edge of new map points: " << new_edge_of_new_map_point << std::endl;
    // std::cout << "new edge of existing map points: " << new_edge_of_existing_map_point << std::endl << std::endl;
    
    // pose_optimizer.reset(nullptr);

    return true;

    // // Iterate over all matched features
    // for(std::vector<cv::Point2f>::size_type i = 0; i < reference_matched_kp_2.size(); i++)
    // {      
    //     if((unsigned int)mask.at<uchar>(i)){
    //         float r_dep = reference->ImDepth.at<float>(reference_matched_kp_2[i].y,reference_matched_kp_2[i].x);
    //         float c_dep = current->ImDepth.at<float>(current_matched_kp_2[i].y,current_matched_kp_2[i].x);
    //         if(r_dep > 0 && c_dep > 0){
    //             pcl::PointXYZ r_point;
    //             r_point.x = (reference_matched_kp_2[i].x - cx) * r_dep / fx;
    //             r_point.y = (reference_matched_kp_2[i].y - cy) * r_dep / fy;
    //             r_point.z = r_dep; 

    //             reference_pc->points.push_back(r_point);
                
    //             float c_x = (current_matched_kp_2[i].x - cx) * c_dep / fx;
    //             float c_y = (current_matched_kp_2[i].y - cy) * c_dep / fy;

    //             pcl::PointXYZ c_point;
    //             c_point.x = c_x;
    //             c_point.y = c_y;
    //             c_point.z = c_dep;

    //             current_pc->points.push_back(c_point);

    //             final_matches++;

    //             cv::KeyPoint ref_kp = reference->keypoints[reference_matched_kp_idx_2[i]];
    //             cv::KeyPoint cur_kp = current->keypoints[current_matched_kp_idx_2[i]];

    //             reference_final_kp.push_back(ref_kp);
    //             current_final_kp.push_back(cur_kp);

    //             cv::DMatch r_to_c(reference_final_kp.size(),current_final_kp.size(),discriptor_distance_2[i]);



    //             if(check_map){
    //                 // Check if the matched features correspond to an existing map point
    //                 std::map<int,int>::iterator correspondence = current->mappoints_id.find(current_matched_kp_idx_2[i]);

    //                 if(correspondence == current->mappoints_id.end()){
    //                     cvo_slam::MappointPtr new_map_point(nullptr);
    //                     CreateNewMapPoints(new_map_point, reference_matched_kp_idx_2[i], current_matched_kp_idx_2[i], reference, \
    //                                     current, graph, id_interval_, mappoint_vertex_id_, projection_edge_id_, projection_num, \
    //                                     F_c_wrt_r, r_R, r_t, r_inv_R, r_inv_t, r_inv_T, c_R, c_t, c_inv_R, c_inv_t, c_inv_T, fx, fy, cx, cy);
    //                     if(new_map_point.get() != nullptr){
    //                         map_points.push_back(new_map_point);
    //                         new_edge_of_new_map_point = new_edge_of_new_map_point + 2;
    //                     } 
    //                 }
    //                 else{
    //                     cv::KeyPoint kp = reference->keypoints[reference_matched_kp_idx_2[i]];
    //                     float info = reference->mvInvLevelSigma2[kp.octave];
    //                     g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) graph.vertex(correspondence->second);
    //                     assert(map_point_vertex != nullptr);
                        
    //                     g2o::EdgeSE3ProjectionOnlyPose* edge = AddProjectionEdgeToOptimizer(map_point_vertex, kp, info, reference->intrinsic);
                        
    //                     edge_kp_id_pairs.push_back(std::make_pair(edge,reference_matched_kp_idx_2[i]));
    //                 }
    //             }
    //         }
    //     }
    // }

    // if(check_map){
    //     // if reference keyframe initially corresponds to enough number of existing map points,
    //     // perform pose optimization to determine map points that are probably visible.    
    //     if(edge_kp_id_pairs.size() >= 15){
    //         g2o::VertexSE3ExpmapInv* v = (g2o::VertexSE3ExpmapInv*) pose_optimizer->vertex(0);
    //         std::vector<bool> inliers(edge_kp_id_pairs.size(),true);
    //         bool enough_inliers = true;
            
    //         for(size_t i = 0; i < 4; i++){
    //             v->setEstimate(reference_vertex->estimate());
    //             pose_optimizer->initializeOptimization(0);
    //             pose_optimizer->optimize(10);

    //             int inliers_count = 0;
    //             for(size_t j = 0; j < edge_kp_id_pairs.size(); j++){
    //                 g2o::EdgeSE3ProjectionOnlyPose* edge = edge_kp_id_pairs[j].first;
    //                 if(!inliers[j]) edge->computeError();
    //                 float chi2 = edge->chi2();
    //                 if(chi2 > 5.991){
    //                     inliers[j] = false;
    //                     edge->setLevel(1);
    //                 }
    //                 else{
    //                     inliers[j] = true;
    //                     edge->setLevel(0);
    //                     inliers_count++;
    //                 }
    //             }

    //             if(inliers_count < 10){
    //                 enough_inliers = false;
    //                 break;
    //             } 
    //         }

    //         if(enough_inliers){
    //             for(size_t k = 0; k < edge_kp_id_pairs.size(); k++){
    //                 if(inliers[k]){
    //                     int map_point_id = (edge_kp_id_pairs[k].first)->id();
    //                     int kp_id = edge_kp_id_pairs[k].second;

    //                     cvo_slam::MappointPtr map_point = map_points[(map_point_id - 1)/2];
    //                     assert(map_point->id == map_point_id);
    //                     map_point->keypoints_id.insert({reference->id, kp_id});
    //                     reference->mappoints_id.insert({kp_id, map_point_id});

    //                     // create a new projection edge
    //                     g2o::EdgeSE3Projection* reference_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(reference->intrinsic));
                        
    //                     cv::KeyPoint kp = reference->keypoints[kp_id];
    //                     g2o::Vector2D r_xy(kp.pt.x, kp.pt.y);
    //                     reference_edge->setId(projection_edge_id_);

    //                     projection_edge_id_ += id_interval_;
    //                     projection_num++;
    //                     new_edge_of_existing_map_point++;

    //                     reference_edge->setMeasurement(r_xy);
    //                     reference_edge->setRobustKernel(createRobustKernel());
    //                     reference_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * reference->mvInvLevelSigma2[kp.octave]);
    //                     reference_edge->resize(2);
    //                     reference_edge->setLevel(0);
    //                     reference_edge->setVertex(0,graph.vertex(map_point_id));
    //                     reference_edge->setVertex(1,graph.vertex(reference->id));

    //                     graph.addEdge(reference_edge);
    //                 }
    //             }
    //         }
    //     }

    //     std::cout << "edge_kp_id_pairs num: " << edge_kp_id_pairs.size() << std::endl;
    //     std::cout << "new edge of new map points: " << new_edge_of_new_map_point << std::endl;
    //     std::cout << "new edge of existing map points: " << new_edge_of_existing_map_point << std::endl << std::endl;
    // }
    

    // pose_optimizer.reset(nullptr);
    
    // return final_matches;
} 

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

g2o::RobustKernel* ORBmatcher::createRobustKernel()
{
    if (UseRobustKernel_)
    {
      g2o::RobustKernel *k = new g2o::RobustKernelCauchy();
      k->setDelta(RobustKernelDelta_);

      return k;
    }
    else
    {
      return 0;
    }
}

void ORBmatcher::CreateNewMapPoints(cvo_slam::MappointPtr& new_map_point, const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
                                    g2o::SparseOptimizer& graph, const int& id_interval_, int& mappoint_vertex_id_, int& projection_edge_id_, int& projection_num, const cv::Mat& F_c_wrt_r, \
                                    const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& r_inv_R, const cv::Mat& r_inv_t, const cv::Mat& r_inv_T, const cv::Mat& c_R, const cv::Mat& c_t, \
                                    const cv::Mat& c_inv_R, const cv::Mat& c_inv_t, const cv::Mat& c_inv_T, const float& fx, const float& fy, const float& cx, const float& cy)
{
    cv::KeyPoint r_kp = reference->keypoints[r_kp_idx];
    cv::KeyPoint c_kp = current->keypoints[c_kp_idx];
    
    float ratioFactor = 1.5f*reference->fScaleFactor;

    // Check first that baseline is not too short
    float baseline = cv::norm(r_t - c_t);

    if(baseline < reference->mbf / fx) return;

    // Check epipolar constraints
    // Compute epipole in second image
    cv::Mat P_r_in_c = c_inv_R * r_t + c_inv_t;
    float ex = fx * P_r_in_c.at<float>(0) / P_r_in_c.at<float>(2) + cx;
    float ey = fy * P_r_in_c.at<float>(1) / P_r_in_c.at<float>(2) + cy; 

    float distex = ex - c_kp.pt.x;    
    float distey = ey - c_kp.pt.y;
    if(distex*distex+distey*distey<100*reference->mvScaleFactor[c_kp.octave]) return;

    if(!CheckDistEpipolarLine(r_kp,c_kp,F_c_wrt_r,current)) return;

    // Triangulate matched features
    // Check parallax between rays
    cv::Mat xn1 = (cv::Mat_<float>(3,1) << (r_kp.pt.x-cx)/fx, (r_kp.pt.y-cy)/fy, 1.0);
    cv::Mat xn2 = (cv::Mat_<float>(3,1) << (c_kp.pt.x-cx)/fx, (c_kp.pt.y-cy)/fy, 1.0);

    cv::Mat ray1 = r_R*xn1;
    cv::Mat ray2 = c_R*xn2;
    float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

    cv::Mat x3D;
    if(cosParallaxRays>0 && cosParallaxRays<0.9998){
        // Linear Triangulation Method
        cv::Mat A(4,4,CV_32F);
        A.row(0) = xn1.at<float>(0)*r_inv_T.row(2)-r_inv_T.row(0);
        A.row(1) = xn1.at<float>(1)*r_inv_T.row(2)-r_inv_T.row(1);
        A.row(2) = xn2.at<float>(0)*c_inv_T.row(2)-c_inv_T.row(0);
        A.row(3) = xn2.at<float>(1)*c_inv_T.row(2)-c_inv_T.row(1);

        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

        x3D = vt.row(3).t();

        if(x3D.at<float>(3)==0) return;

        // Euclidean coordinates
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }
    else return;

    cv::Mat x3Dt = x3D.t();

    //Check triangulation in front of cameras
    float z1 = r_inv_R.row(2).dot(x3Dt)+r_inv_t.at<float>(2);
    if(z1<=0) return;

    float z2 = c_inv_R.row(2).dot(x3Dt)+c_inv_t.at<float>(2);
    if(z2<=0) return;


    //Check reprojection error in first keyframe
    // float sigmaSquare1 = reference->mvLevelSigma2[r_kp.octave];
    float x1 = r_inv_R.row(0).dot(x3Dt)+r_inv_t.at<float>(0);
    float y1 = r_inv_R.row(1).dot(x3Dt)+r_inv_t.at<float>(1);
    float invz1 = 1.0/z1;

    float u1 = fx*x1*invz1+cx;
    float v1 = fy*y1*invz1+cy;
    float errX1 = u1 - r_kp.pt.x;
    float errY1 = v1 - r_kp.pt.y;
    // if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1) return;
    if((errX1*errX1+errY1*errY1) > 9.0) return;

    //Check reprojection error in second keyframe
    // float sigmaSquare2 = current->mvLevelSigma2[c_kp.octave];
    float x2 = c_inv_R.row(0).dot(x3Dt)+c_inv_t.at<float>(0);
    float y2 = c_inv_R.row(1).dot(x3Dt)+c_inv_t.at<float>(1);
    float invz2 = 1.0/z2;
    
    float u2 = fx*x2*invz2+cx;
    float v2 = fy*y2*invz2+cy;
    float errX2 = u2 - c_kp.pt.x;
    float errY2 = v2 - c_kp.pt.y;
    // if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2) return;
    if((errX2*errX2+errY2*errY2) > 9.0) return;

    //Check scale consistency
    cv::Mat normal1 = x3D-r_t;
    float dist1 = cv::norm(normal1);

    cv::Mat normal2 = x3D-c_t;
    float dist2 = cv::norm(normal2);

    if(dist1==0 || dist2==0) return;

    const float ratioDist = dist2/dist1;
    const float ratioOctave = reference->mvScaleFactor[r_kp.octave]/current->mvScaleFactor[c_kp.octave];

    if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor) return;

    // Triangulation succeeds here, create a new map point, compute its mean viewing direction,
    // and update visibility between it and keyframes
    g2o::Vector3D position = Converter::toEigenVector(x3D);
    cv::Mat normal = normal1 / dist1  + normal2 / dist2;
    normal = normal/cv::norm(normal);

    new_map_point.reset(new cvo_slam::Mappoint(mappoint_vertex_id_, position, normal));
    new_map_point->keypoints_id.insert({reference->id, r_kp_idx});
    new_map_point->keypoints_id.insert({current->id, c_kp_idx});

    reference->mappoints_id.insert({r_kp_idx, mappoint_vertex_id_});
    current->mappoints_id.insert({c_kp_idx, mappoint_vertex_id_});

    mappoint_vertex_id_ += id_interval_;

    // // create a new map point vertex
    // g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
    // map_point_vertex->setEstimate(position);
    // map_point_vertex->setId(mappoint_vertex_id_);
    // map_point_vertex->setMarginalized(true);

    // mappoint_vertex_id_ += id_interval_;

    // graph.addVertex(map_point_vertex);


    // // create projection edges
    // g2o::EdgeSE3Projection* reference_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(reference->intrinsic));
    // g2o::EdgeSE3Projection* current_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(current->intrinsic));
    
    // g2o::Vector2D r_xy(r_kp.pt.x, r_kp.pt.y);
    // reference_edge->setId(projection_edge_id_);

    // projection_edge_id_ += id_interval_;
    // projection_num++;

    // reference_edge->setMeasurement(r_xy);
    // reference_edge->setRobustKernel(createRobustKernel());
    // reference_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * reference->mvInvLevelSigma2[r_kp.octave]);
    // reference_edge->resize(2);
    // reference_edge->setLevel(0);
    // reference_edge->setVertex(0,map_point_vertex);
    // reference_edge->setVertex(1,graph.vertex(reference->id));

    // graph.addEdge(reference_edge);

    
    // g2o::Vector2D c_xy(c_kp.pt.x, c_kp.pt.y);
    // current_edge->setId(projection_edge_id_);

    // projection_edge_id_ += id_interval_;
    // projection_num++;

    // current_edge->setMeasurement(c_xy);
    // current_edge->setRobustKernel(createRobustKernel());
    // current_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * current->mvInvLevelSigma2[c_kp.octave]);
    // current_edge->resize(2);
    // current_edge->setLevel(0);
    // current_edge->setVertex(0,map_point_vertex);
    // current_edge->setVertex(1,graph.vertex(current->id));

    // graph.addEdge(current_edge);
}

void ORBmatcher::CreateNewMapPoints(cvo_slam::MappointPtr& new_map_point, const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
                                    const int& id_interval_, int& mappoint_vertex_id_, const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& c_t, const g2o::Vector3D& reference_pc_inlier)
{
    // create a new map point, compute its mean viewing direction,
    // and update visibility between it and keyframes

    cv::Mat p_reference = (cv::Mat_<float>(3,1) << reference_pc_inlier(0),reference_pc_inlier(1),reference_pc_inlier(2));
    cv::Mat p_world = r_R * p_reference + r_t;

    cv::Mat normal1 = p_world-r_t;
    float dist1 = cv::norm(normal1);

    cv::Mat normal2 = p_world-c_t;
    float dist2 = cv::norm(normal2);

    g2o::Vector3D position = Converter::toEigenVector(p_world);
    cv::Mat normal = normal1 / dist1  + normal2 / dist2;
    normal = normal/cv::norm(normal);

    new_map_point.reset(new cvo_slam::Mappoint(mappoint_vertex_id_, position, normal));
    new_map_point->keypoints_id.insert({reference->id, r_kp_idx});
    new_map_point->keypoints_id.insert({current->id, c_kp_idx});

    reference->mappoints_id.insert({r_kp_idx, mappoint_vertex_id_});
    current->mappoints_id.insert({c_kp_idx, mappoint_vertex_id_});

    mappoint_vertex_id_ += id_interval_;
}

// bool ORBmatcher::CheckEpipolarConstraint(const int& r_kp_idx, const int& c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, const cv::Mat& F_c_wrt_r, \
//                                     const cv::Mat& r_R, const cv::Mat& r_t, const cv::Mat& r_inv_R, const cv::Mat& r_inv_t, const cv::Mat& r_inv_T, const cv::Mat& c_R, const cv::Mat& c_t, \
//                                     const cv::Mat& c_inv_R, const cv::Mat& c_inv_t, const cv::Mat& c_inv_T, const float& fx, const float& fy, const float& cx, const float& cy)
// {
//     cv::KeyPoint r_kp = reference->keypoints[r_kp_idx];
//     cv::KeyPoint c_kp = current->keypoints[c_kp_idx];
    
//     float ratioFactor = 1.5f*reference->fScaleFactor;

//     // Check first that baseline is not too short
//     float baseline = cv::norm(r_t - c_t);

//     if(baseline < reference->mbf / fx) return false;

//     // Check epipolar constraints
//     // Compute epipole in second image
//     cv::Mat P_r_in_c = c_inv_R * r_t + c_inv_t;
//     float ex = fx * P_r_in_c.at<float>(0) / P_r_in_c.at<float>(2) + cx;
//     float ey = fy * P_r_in_c.at<float>(1) / P_r_in_c.at<float>(2) + cy; 

//     float distex = ex - c_kp.pt.x;    
//     float distey = ey - c_kp.pt.y;
//     if(distex*distex+distey*distey<100*reference->mvScaleFactor[c_kp.octave]) return false;

//     if(!CheckDistEpipolarLine(r_kp,c_kp,F_c_wrt_r,current)) return false;

//     // Triangulate matched features
//     // Check parallax between rays
//     cv::Mat xn1 = (cv::Mat_<float>(3,1) << (r_kp.pt.x-cx)/fx, (r_kp.pt.y-cy)/fy, 1.0);
//     cv::Mat xn2 = (cv::Mat_<float>(3,1) << (c_kp.pt.x-cx)/fx, (c_kp.pt.y-cy)/fy, 1.0);

//     cv::Mat ray1 = r_R*xn1;
//     cv::Mat ray2 = c_R*xn2;
//     float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

//     cv::Mat x3D;
//     if(cosParallaxRays>0 && cosParallaxRays<0.9998){
//         // Linear Triangulation Method
//         cv::Mat A(4,4,CV_32F);
//         A.row(0) = xn1.at<float>(0)*r_inv_T.row(2)-r_inv_T.row(0);
//         A.row(1) = xn1.at<float>(1)*r_inv_T.row(2)-r_inv_T.row(1);
//         A.row(2) = xn2.at<float>(0)*c_inv_T.row(2)-c_inv_T.row(0);
//         A.row(3) = xn2.at<float>(1)*c_inv_T.row(2)-c_inv_T.row(1);

//         cv::Mat w,u,vt;
//         cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

//         x3D = vt.row(3).t();

//         if(x3D.at<float>(3)==0) return false;

//         // Euclidean coordinates
//         x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
//     }
//     else return false;

//     cv::Mat x3Dt = x3D.t();

//     //Check triangulation in front of cameras
//     float z1 = r_inv_R.row(2).dot(x3Dt)+r_inv_t.at<float>(2);
//     if(z1<=0) return false;

//     float z2 = c_inv_R.row(2).dot(x3Dt)+c_inv_t.at<float>(2);
//     if(z2<=0) return false;


//     //Check reprojection error in first keyframe
//     float sigmaSquare1 = reference->mvLevelSigma2[r_kp.octave];
//     float x1 = r_inv_R.row(0).dot(x3Dt)+r_inv_t.at<float>(0);
//     float y1 = r_inv_R.row(1).dot(x3Dt)+r_inv_t.at<float>(1);
//     float invz1 = 1.0/z1;

//     float u1 = fx*x1*invz1+cx;
//     float v1 = fy*y1*invz1+cy;
//     float errX1 = u1 - r_kp.pt.x;
//     float errY1 = v1 - r_kp.pt.y;
//     if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1) return false;

//     //Check reprojection error in second keyframe
//     float sigmaSquare2 = current->mvLevelSigma2[c_kp.octave];
//     float x2 = c_inv_R.row(0).dot(x3Dt)+c_inv_t.at<float>(0);
//     float y2 = c_inv_R.row(1).dot(x3Dt)+c_inv_t.at<float>(1);
//     float invz2 = 1.0/z2;
    
//     float u2 = fx*x2*invz2+cx;
//     float v2 = fy*y2*invz2+cy;
//     float errX2 = u2 - c_kp.pt.x;
//     float errY2 = v2 - c_kp.pt.y;
//     if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2) return false;

//     //Check scale consistency
//     cv::Mat normal1 = x3D-r_t;
//     float dist1 = cv::norm(normal1);

//     cv::Mat normal2 = x3D-c_t;
//     float dist2 = cv::norm(normal2);

//     if(dist1==0 || dist2==0) return false;

//     const float ratioDist = dist2/dist1;
//     const float ratioOctave = reference->mvScaleFactor[r_kp.octave]/current->mvScaleFactor[c_kp.octave];

//     if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor) return false;

//     // epipolar check succeeds here
//     return true;
// }

// void ORBmatcher::CreateNewMapPoint(cvo_slam::MappointVector& map_points, int r_kp_idx, int c_kp_idx,  const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current, \
//                                    int id_interval_, int& mappoint_vertex_id_)
// {
//     cvo_slam::MappointPtr new_map_point(new cvo_slam::Mappoint(mappoint_vertex_id_));
    
//     new_map_point->keypoints_id.insert({reference->id, r_kp_idx});
//     new_map_point->keypoints_id.insert({current->id, c_kp_idx});

//     reference->mappoints_id.insert({r_kp_idx, mappoint_vertex_id_});
//     current->mappoints_id.insert({c_kp_idx, mappoint_vertex_id_});

//     mappoint_vertex_id_ += id_interval_;

//     map_points.push_back(new_map_point);
// }

// void ORBmatcher::IncludeMappointIntoGraph(cvo_slam::MappointPtr& map_point, cvo_slam::KeyframeVector& keyframes, float fx, float fy, float cx, float cy, g2o::SparseOptimizer& graph, \
//                                           int id_interval_, int& projection_edge_id_, int& projection_num)
// {
//     assert(map_point->keypoints_id.size() == 2);

//     // create a new map point vertex
//     g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
//     map_point_vertex->setId(map_point->id);
//     map_point_vertex->setMarginalized(true);

//     for(std::map<int,int>::iterator it = map_point->keypoints_id.begin(); it != map_point->keypoints_id.end(); it++){
//         // std::cout << "keyframe id in IncludeMappointIntoGraph: " << it->first << std::endl;
        
//         cvo_slam::KeyframePtr kf = keyframes[it->first / 2];
//         assert(kf->id == it->first);

//         g2o::VertexSE3ExpmapInv* keyframe_vertex = (g2o::VertexSE3ExpmapInv*) graph.vertex(kf->id);
//         cv::KeyPoint kp = kf->keypoints[it->second];

//         if(it == map_point->keypoints_id.begin()){
//             float dep = kf->ImDepth.at<float>(kp.pt.y,kp.pt.x);
//             assert(dep > 0);
            
//             cv::Mat position_in_kf(3,1,CV_32F);
//             position_in_kf.at<float>(0,0) = (kp.pt.x - cx) * dep / fx;
//             position_in_kf.at<float>(1,0) = (kp.pt.y - cy) * dep / fy;
//             position_in_kf.at<float>(2,0) = dep;
            
//             cv::Mat pose = Converter::toCvMat(keyframe_vertex->estimateInv());
//             cv::Mat R = pose.rowRange(0,3).colRange(0,3);
//             cv::Mat t = pose.rowRange(0,3).col(3);
//             cv::Mat position_in_world = R * position_in_kf + t;

//             map_point->position = Converter::toEigenVector(position_in_world);
//             map_point_vertex->setEstimate(map_point->position);

//             graph.addVertex(map_point_vertex);
//         }

//         g2o::EdgeSE3Projection* projection_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(kf->intrinsic));
        
//         g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);
//         projection_edge->setId(projection_edge_id_);

//         projection_edge_id_ += id_interval_;
//         projection_num++;

//         projection_edge->setMeasurement(kp_xy);
//         projection_edge->setRobustKernel(createRobustKernel());
//         projection_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * kf->mvInvLevelSigma2[kp.octave]);
//         projection_edge->resize(2);
//         projection_edge->setLevel(0);
//         projection_edge->setVertex(0,map_point_vertex);
//         projection_edge->setVertex(1,graph.vertex(kf->id));

//         graph.addEdge(projection_edge);
//     }
//     // std::cout << std::endl;
// }

// void ORBmatcher::UpdateVisibility(cvo_slam::MappointPtr& map_point, cvo_slam::KeyframePtr& keyframe, int kp_idx, g2o::SparseOptimizer& graph, \
//                                   int id_interval_, int& projection_edge_id_, int& projection_num)
// {
//     g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) graph.vertex(map_point->id);
//     assert(map_point_vertex != nullptr);
//     assert(map_point_vertex->id() == map_point->id);

//     map_point->keypoints_id.insert({keyframe->id, kp_idx});
//     keyframe->mappoints_id.insert({kp_idx, map_point->id});

//     // create a new projection edge
//     g2o::EdgeSE3Projection* projection_edge = new g2o::EdgeSE3Projection(Converter::toEigenMat(keyframe->intrinsic));
    
//     cv::KeyPoint kp = keyframe->keypoints[kp_idx];
//     g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);
//     projection_edge->setId(projection_edge_id_);

//     projection_edge_id_ += id_interval_;
//     projection_num++;

//     projection_edge->setMeasurement(kp_xy);
//     projection_edge->setRobustKernel(createRobustKernel());
//     projection_edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * keyframe->mvInvLevelSigma2[kp.octave]);
//     projection_edge->resize(2);
//     projection_edge->setLevel(0);
//     projection_edge->setVertex(0,map_point_vertex);
//     projection_edge->setVertex(1,graph.vertex(keyframe->id));

//     graph.addEdge(projection_edge);
// }

bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const cvo_slam::KeyframePtr& pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}

void ORBmatcher::ResetPoseOptimizer()
{
    pose_optimizer.reset(new g2o::SparseOptimizer());
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    pose_optimizer->setAlgorithm(solver);
    pose_optimizer->setVerbose(false);
}

void ORBmatcher::AddPoseVertexToOptimizer(const g2o::VertexSE3ExpmapInv* frame_vertex)
{
    g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
    v->setId(0);
    v->setEstimate(frame_vertex->estimate());
    v->setHessianIndex(-1);

    pose_optimizer->addVertex(v);
}

void ORBmatcher::CheckExistingMappoints(cvo_slam::KeyframePtr& keyframe, cvo_slam::MappointVector& map_points)
{
    std::cout << "Number of potentially visible map points: " << map_point_kp_pairs.size() << std::endl;
    
    std::vector<bool> inliers(map_point_kp_pairs.size(),true);
    bool enough_inliers = true;
    
    for(size_t i = 0; i < 4; i++){
        pose_optimizer->initializeOptimization(0);
        pose_optimizer->optimize(5);

        int inliers_count = 0;
        for(size_t j = 0; j < map_point_kp_pairs.size(); j++){
            g2o::EdgeSE3ProjectionOnlyPose* edge = map_point_kp_pairs[j].first;
            if(!inliers[j]) edge->computeError();
            float chi2 = edge->chi2();
            if(chi2 > 5.991){
                inliers[j] = false;
                edge->setLevel(1);
            }
            else{
                inliers[j] = true;
                edge->setLevel(0);
                inliers_count++;
            }
        }

        // std::cout << "inliers count: " << inliers_count << std::endl;

        if(inliers_count < 10){
            enough_inliers = false;
            break;
        } 
    }

    int visible_map_points = 0;

    if(enough_inliers){
        for(size_t k = 0; k < map_point_kp_pairs.size(); k++){
            if(inliers[k]){
                int map_point_id = (map_point_kp_pairs[k].first)->id();
                int kp_id = map_point_kp_pairs[k].second;

                cvo_slam::MappointPtr map_point = map_points[(map_point_id - 1)/2];
                assert(map_point->id == map_point_id);

                // map point counter
                for(std::map<int,int>::iterator it = map_point->keypoints_id.begin(); it != map_point->keypoints_id.end(); it++){
                    std::map<int,int>::iterator kf_iter = keyframe_map_point_pairs.find(it->first);
                    if(kf_iter == keyframe_map_point_pairs.end()) keyframe_map_point_pairs.insert({it->first,1});
                    else keyframe_map_point_pairs[kf_iter->first]++;
                }

                map_point->keypoints_id.insert({keyframe->id, kp_id});
                keyframe->mappoints_id.insert({kp_id, map_point_id});

                visible_map_points++;
            }
        }
    }

    std::cout << "Number of actually visible map points: " << visible_map_points << std::endl << std::endl;
}

bool ORBmatcher::CheckExistingMappointByProjection(cvo_slam::KeyframePtr& keyframe, cvo_slam::MappointPtr& map_point, int keypoint_id, \
                                                   const cv::Mat& t, const cv::Mat& inv_R, const cv::Mat& inv_t, const cv::Mat& intrinsic)
{   
    cv::Mat position_w = ORB_SLAM2::Converter::toCvMat(map_point->position);
    cv::Mat position_c = inv_R * position_w + inv_t;

    // Check depth in camera frame
    if(position_c.at<float>(2) <= 0.0) return false;
    
    cv::Mat projection = intrinsic * position_c;
    
    float x = projection.at<float>(0) / projection.at<float>(2);
    float y = projection.at<float>(1) / projection.at<float>(2);

    cv::KeyPoint kp = keyframe->keypoints[keypoint_id];

    // Check reprojection error
    // float sigmaSquare = keyframe->mvLevelSigma2[kp.octave];
    float errX = x - kp.pt.x;
    float errY = y - kp.pt.y;
    // if((errX*errX+errY*errY)>5.991*sigmaSquare) return false;
    if((errX*errX+errY*errY) > 64.0) return false;

    // // View angle of the map point must be less than 60 degree
    // cv::Mat view_direction = position_w - t;
    // cv::Mat normal = map_point->GetNormal();
    // if(view_direction.dot(normal) < 0.5 * cv::norm(view_direction)) return false;

    // count covisible keyframes of the map point
    for(std::map<int,int>::iterator it = map_point->keypoints_id.begin(); it != map_point->keypoints_id.end(); it++){
        std::map<int,int>::iterator kf_iter = keyframe_map_point_pairs.find(it->first);
        if(kf_iter == keyframe_map_point_pairs.end()) keyframe_map_point_pairs.insert({it->first,1});
        else keyframe_map_point_pairs[kf_iter->first]++;
    }

    // Update visibility of the map point and the keyframe
    map_point->keypoints_id.insert({keyframe->id, keypoint_id});
    keyframe->mappoints_id.insert({keypoint_id, map_point->id}); 

    return true;
}

// void ORBmatcher::CheckExistingMappoints(cvo_slam::KeyframePtr& keyframe, cvo_slam::MappointVector& map_points)
// {
//     std::cout << "Number of potentially visible map points: " << map_point_kp_pairs.size() << std::endl;
    
//     std::vector<bool> inliers(map_point_kp_pairs.size(),true);
//     bool enough_inliers = true;
    
//     pose_optimizer->initializeOptimization(0);
//     pose_optimizer->optimize(5);

//     int inliers_count = 0;
//     for(size_t j = 0; j < map_point_kp_pairs.size(); j++){
//         g2o::EdgeSE3Projection* edge = map_point_kp_pairs[j].first;
//         // if(!inliers[j]) edge->computeError();
//         // float chi2 = edge->chi2();
//         if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
//             inliers[j] = false;
//             // edge->setLevel(1);
//         }
//         else{
//             inliers[j] = true;
//             // edge->setLevel(0);
//             inliers_count++;
//         }
//     }

//     std::cout << "inliers count: " << inliers_count << std::endl;

//     if(inliers_count < 10){
//         enough_inliers = false;
//     } 

//     int visible_map_points = 0;

//     if(enough_inliers){
//         for(size_t k = 0; k < map_point_kp_pairs.size(); k++){
//             if(inliers[k]){
//                 int map_point_id = (map_point_kp_pairs[k].first)->id();
//                 int kp_id = map_point_kp_pairs[k].second;

//                 cvo_slam::MappointPtr map_point = map_points[(map_point_id - 1)/2];
//                 assert(map_point->id == map_point_id);

//                 // map point counter
//                 for(std::map<int,int>::iterator it = map_point->keypoints_id.begin(); it != map_point->keypoints_id.end(); it++){
//                     std::map<int,int>::iterator kf_iter = keyframe_map_point_pairs.find(it->first);
//                     if(kf_iter == keyframe_map_point_pairs.end()) keyframe_map_point_pairs.insert({it->first,1});
//                     else keyframe_map_point_pairs[kf_iter->first]++;
//                 }

//                 map_point->keypoints_id.insert({keyframe->id, kp_id});
//                 keyframe->mappoints_id.insert({kp_id, map_point_id});

//                 visible_map_points++;
//             }
//         }
//     }

//     std::cout << "Number of actually visible map points: " << visible_map_points << std::endl << std::endl;
// }

void ORBmatcher::GetBestCovisibleKeyframeList(std::vector<int>& keyframe_list, int& farest_keyframe)
{
    if(!keyframe_list.empty()) keyframe_list.clear();

    std::vector<std::pair<int,int>> map_point_keyframe_pairs;
    for(std::map<int,int>::iterator it = keyframe_map_point_pairs.begin(); it != keyframe_map_point_pairs.end(); it++){
        if(it->second >= 15) map_point_keyframe_pairs.push_back({it->second, it->first});
    }

    if(map_point_keyframe_pairs.empty()) return;
    sort(map_point_keyframe_pairs.begin(),map_point_keyframe_pairs.end(),greater<>());
    farest_keyframe = map_point_keyframe_pairs.front().second;

    for(int i=0; i<min(10,(int)map_point_keyframe_pairs.size()); i++){
        std::cout << "keyframe: " << map_point_keyframe_pairs[i].second << " covisible map point number: " << map_point_keyframe_pairs[i].first << std::endl;
        keyframe_list.push_back(map_point_keyframe_pairs[i].second);
        if (map_point_keyframe_pairs[i].second < farest_keyframe) farest_keyframe = map_point_keyframe_pairs[i].second;
    }

    std::cout << "farest covisible keyframe: " << farest_keyframe << std::endl;

    return;
}

void ORBmatcher::GetBestCovisibleKeyframeList(cvo_slam::KeyframePtr& reference)
{
    std::vector<std::pair<int,int>> map_point_keyframe_pairs;
    for(std::map<int,int>::iterator it = keyframe_map_point_pairs.begin(); it != keyframe_map_point_pairs.end(); it++){
        if(it->second >= 15) map_point_keyframe_pairs.push_back({it->second, it->first});
    }

    if(map_point_keyframe_pairs.empty()) return;
    sort(map_point_keyframe_pairs.begin(),map_point_keyframe_pairs.end(),greater<>());
    for(int i=0; i<min(10,(int)map_point_keyframe_pairs.size()); i++){
        std::cout << "keyframe: " << map_point_keyframe_pairs[i].second << " covisible map point number: " << map_point_keyframe_pairs[i].first << std::endl;
        reference->bestCovisibleKeyframeList.insert(map_point_keyframe_pairs[i].second);
    }

    std::cout << "farest covisible keyframe: " << *(reference->bestCovisibleKeyframeList.cbegin()) << std::endl;

    return;
}

void ORBmatcher::ReleasePoseOptimizer()
{
    pose_optimizer.reset(nullptr);
    map_point_kp_pairs.clear();
    keyframe_map_point_pairs.clear();
}

g2o::EdgeSE3ProjectionOnlyPose* ORBmatcher::AddProjectionEdgeToOptimizer(cvo_slam::KeyframePtr& keyframe, int kp_id, cvo_slam::MappointPtr& map_point)
{
    cv::KeyPoint kp = keyframe->keypoints[kp_id];
    g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

    // create a new projection edge
    g2o::EdgeSE3ProjectionOnlyPose* edge = new g2o::EdgeSE3ProjectionOnlyPose(Converter::toEigenMat(keyframe->intrinsic), map_point->position);
    
    edge->setId(map_point->id);
    edge->setMeasurement(kp_xy);
    edge->setRobustKernel(createRobustKernel());
    edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * keyframe->mvInvLevelSigma2[kp.octave]);
    edge->setLevel(0);
    edge->setVertex(0,static_cast<g2o::VertexSE3ExpmapInv*>(pose_optimizer->vertex(0)));

    pose_optimizer->addEdge(edge);

    return edge;
}

// g2o::EdgeSE3Projection* ORBmatcher::AddMappointAndEdgeToOptimizer(cvo_slam::KeyframePtr& keyframe, int kp_id, cvo_slam::MappointPtr& map_point)
// {
//     cv::KeyPoint kp = keyframe->keypoints[kp_id];
//     g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

//     // create a map point vertex
//     g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
//     map_point_vertex->setEstimate(map_point->position);
//     map_point_vertex->setId(map_point->id);
//     map_point_vertex->setMarginalized(true);
//     pose_optimizer->addVertex(map_point_vertex);

//     // create a projection edge
//     g2o::EdgeSE3Projection* edge = new g2o::EdgeSE3Projection(ORB_SLAM2::Converter::toEigenMat(keyframe->intrinsic));
//     edge->setMeasurement(kp_xy);
//     edge->setRobustKernel(createRobustKernel());
//     edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * keyframe->mvInvLevelSigma2[kp.octave]);
//     edge->resize(2);
//     edge->setLevel(0);
//     edge->setVertex(0,map_point_vertex);
//     edge->setVertex(1,static_cast<g2o::VertexSE3ExpmapInv*>(pose_optimizer->vertex(0)));
//     pose_optimizer->addEdge(edge);

//     return edge;
// }

// g2o::EdgeSE3ProjectionOnlyPose* ORBmatcher::AddProjectionEdgeToOptimizer(const g2o::VertexPointXYZ* map_point_vertex, const cv::KeyPoint& kp, float info, const cv::Mat& intrinsic)
// {
//     g2o::EdgeSE3ProjectionOnlyPose* e = new g2o::EdgeSE3ProjectionOnlyPose(Converter::toEigenMat(intrinsic),map_point_vertex->estimate());
//     g2o::Vector2D measurement(kp.pt.x, kp.pt.y);
    
//     e->setId(map_point_vertex->id());
//     e->setMeasurement(measurement);
//     e->setRobustKernel(createRobustKernel());
//     e->setInformation(Eigen::Matrix<double,2,2>::Identity() * info);
//     e->setLevel(0);
//     e->setVertex(0,static_cast<g2o::VertexSE3ExpmapInv*>(pose_optimizer->vertex(0)));

//     pose_optimizer->addEdge(e);

//     return e;
// }

// void ORBmatcher::drawLoopClosure_debug(const std::vector<cv::KeyPoint>& r_kp, const std::vector<cv::KeyPoint>& c_kp, const std::vector<cv::DMatch>& r_to_c_match, const cvo_slam::KeyframePtr& reference, const cvo_slam::KeyframePtr& current)
// {
// cv::Mat r_image_out;
// cv::Mat c_image_out;
// cv::Mat r_to_c_match_out;

// // cv::drawKeypoints(reference->image->rgb, reference->keypoints, r_image_out);
// // cv::drawKeypoints(current->image->rgb, current->keypoints, c_image_out);

// // cv::drawMatches(reference->image->rgb, r_kp, current->image->rgb, c_kp, r_to_c_match, r_to_c_match_out);

// cv::drawMatches(current->ImGray, c_kp, reference->ImGray, r_kp, r_to_c_match, r_to_c_match_out);

// for(size_t i = 0; i < r_kp.size(); i++){
//     cv::Point2f r_p = r_kp[i].pt;
//     cv::Point2f c_p = c_kp[i].pt;

//     r_p.x += current->ImGray.cols; 
//     cv::line(r_to_c_match_out,r_p,c_p,cv::Scalar(255, 0, 0),1);
// }

// cv::namedWindow( "Superpoint matches inliers", cv::WINDOW_AUTOSIZE );// Create a window for display.
// cv::imshow( "Superpoint matches inliers", r_to_c_match_out);                   // Show our image inside it.

// cv::waitKey();

// // std::string save_folder = "/home/xi/ETH3D/training/camera_shake_1/loop_closure_images/"+std::to_string(reference->id)+"_"+std::to_string(current->id)+"/";
// // boost::filesystem::create_directory(save_folder);

// // std::string r_name = save_folder+std::to_string(reference->id)+".png";
// // std::string c_name = save_folder+std::to_string(current->id)+".png";
// // std::string r_to_c_name = save_folder+std::to_string(reference->id)+"_"+std::to_string(current->id)+".png";

// // cv::imwrite(r_name, r_image_out);
// // cv::imwrite(c_name, c_image_out);
// // cv::imwrite(r_to_c_name, r_to_c_match_out);
// }

Eigen::Matrix<double,4,4> ORBmatcher::computeRigidTransformSVD(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst)
{
    // This function is reference to https://gist.github.com/JiaxiangZheng/8168862

    assert(src.size() == dst.size());
	int pairSize = src.size();
	Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
	for (int i=0; i<pairSize; ++i)
	{
		center_src += src[i];
		center_dst += dst[i];
	}
	center_src /= (double)pairSize;
	center_dst /= (double)pairSize;

	Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);
	for (int i=0; i<pairSize; ++i)
	{
		for (int j=0; j<3; ++j)
			S(i, j) = src[i][j] - center_src[j];
		for (int j=0; j<3; ++j)
			D(i, j) = dst[i][j] - center_dst[j];
	}
	Eigen::MatrixXd Dt = D.transpose();
	Eigen::Matrix3d H = Dt*S;
	Eigen::Matrix3d W, U, V;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd H_(3, 3);
	for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) H_(i, j) = H(i, j);
	svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
	if (!svd.computeU() || !svd.computeV()) {
		std::cerr << "SVD decomposition error" << endl;
		return Eigen::Matrix<double,4,4>::Identity();
	}
	Eigen::Matrix3d Vt = svd.matrixV().transpose();
	Eigen::Matrix3d R = svd.matrixU()*Vt;
	Eigen::Vector3d t = center_dst - R*center_src;	
	
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            T(i,j) = R(i,j);
        }
        T(i,3) = t(i);
    }

    return T;
}

Eigen::Matrix<double,4,4> ORBmatcher::optimizeRelativeTransformation(const vector<int>& current_kp_idx_inliers, const vector<g2o::Vector3D>& reference_pc_inliers, const cvo_slam::KeyframePtr& current, const Eigen::Matrix<double,4,4>& T_cr)
{
    // Set up relative pose optimizer
    g2o::SparseOptimizer pose_optimizer;
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    
    pose_optimizer.setAlgorithm(solver);
    pose_optimizer.setVerbose(false);

    // Add pose vertex
    Eigen::Affine3d T_Affine = Converter::toAffine(T_cr);
    g2o::SE3Quat T_SE3Quat(T_Affine.rotation(), T_Affine.translation());

    g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
    v->setId(0);
    v->setEstimateInv(T_SE3Quat);
    v->setHessianIndex(-1);

    pose_optimizer.addVertex(v);

    // Add all inliers in projection edge
    for(int i=0; i<current_kp_idx_inliers.size(); i++){
        cv::KeyPoint kp = current->keypoints[current_kp_idx_inliers[i]];
        g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

        // create a new projection edge
        g2o::EdgeSE3ProjectionOnlyPose* edge = new g2o::EdgeSE3ProjectionOnlyPose(Converter::toEigenMat(current->intrinsic), reference_pc_inliers[i]);
        
        edge->setId(current_kp_idx_inliers[i]);
        edge->setMeasurement(kp_xy);
        edge->setRobustKernel(createRobustKernel());
        edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * current->mvInvLevelSigma2[kp.octave]);
        edge->setLevel(0);
        edge->setVertex(0,v);

        pose_optimizer.addEdge(edge);
    }

    // Optimize pose with all inliers
    pose_optimizer.initializeOptimization(0);
    pose_optimizer.optimize(20);

    return v->estimateInv().to_homogeneous_matrix(); 
}

} //namespace ORB_SLAM2
