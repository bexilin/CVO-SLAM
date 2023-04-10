/* ----------------------------------------------------------------------------
 * Copyright 2019, Tzu-yuan Lin <tzuyuan@umich.edu>, Maani Ghaffari <maanigj@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   cvo.hpp
 *  @author Tzu-yuan Lin, Maani Ghaffari 
 *  @brief  Header file for contineuous visual odometry
 *  @date   September 20, 2019
 **/

#ifndef CVO_H
#define CVO_H


// #include "DataType.h"
#include "LieGroup.h"
#include "pcd_generator.hpp"
#include "../thirdparty/nanoflann.hpp"
#include "../thirdparty/KDTreeVectorOfVectorsAdaptor.h"

#include <vector>
#include <string.h>
#include <iostream>
#include <memory>
#include <utility>
#include <future>
#include <thread>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky> 
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/StdVector>
#include <opencv2/core/mat.hpp>
#include <boost/timer/timer.hpp>
#include <tbb/tbb.h>
#include <numeric>
#include <complex>

using namespace std;
using namespace nanoflann;

namespace cvo
{

class inn_p
{
public:
    float value;
    int num;
    int num_e;  // excluded point number 

    void copy(const inn_p &result)
    {
    this->value = result.value;
    this->num = result.num;
    this->num_e = result.num_e;
    }

    inn_p(const inn_p &result) : value(result.value),
                                           num(result.num),
                                           num_e(result.num_e)
    {
    }
    inn_p(float v, int n, int n_e)
    {
        this->value = v;
        this->num = n;
        this->num_e = n_e;
    }
    inn_p()
    {
    }
};

class cvo
{    
    
    private:
        // private variables
        unique_ptr<frame> ptr_fixed_fr;
        unique_ptr<frame> ptr_moving_fr;
        unique_ptr<frame> ptr_previous_fr;

        unique_ptr<point_cloud> ptr_fixed_pcd;
        unique_ptr<point_cloud> ptr_moving_pcd;
        unique_ptr<point_cloud> ptr_previous_pcd; // retain the point cloud of last frame so that when a new keyframe is decided, in the CVO object for
                                               // keyframe-based tracking, ptr_fixed_pcd is updated as ptr_previous_pcd

        bool pre_pc_init;      // flag for indication that previous action is init new kf

        int num_fixed;    // target point cloud counts
        int num_moving;   // source point cloud counts
        cloud_t *cloud_x; // target points represented as a matrix (num_fixed,3)
        cloud_t *cloud_y; // source points represented as a matrix (num_moving,3)

        float ell;         // kernel characteristic length-scale
        float sigma;       // kernel signal variance (set as std)      
        float sp_thres;    // kernel sparsification threshold       
        float c;           // so(3) inner product scale     
        float d;           // R^3 inner product scale
        float color_scale; // color space inner product scale
        float c_ell;       // kernel characteristic length-scale for color kernel
        float c_sigma;     // kernel signal variance for color kernel
        float r_weight;
        float g_weight;
        float b_weight;
        float dx_weight;
        float dy_weight;
        int MAX_ITER;   // maximum number of iteration
        float eps;      // the program stops if norm(omega)+norm(v) < eps
        float eps_2;    // threshold for se3 distance
        float min_step; // minimum step size for integration
        float step;     // integration step size

        Eigen::Matrix3f R;                             // orientation
        Eigen::Vector3f T;                             // translation
        Eigen::SparseMatrix<float, Eigen::RowMajor> A; // coefficient matrix, represented in sparse
        Eigen::Vector3f omega;                         // so(3) part of twist
        Eigen::Vector3f v;                             // R^3 part of twist

        // variables for cloud manipulations
        typedef Eigen::Triplet<float> Trip_t;
        // std::vector<Trip> A_trip_concur;
        tbb::concurrent_vector<Trip_t> A_trip_concur;

        int A_nonzero;

        camera_info cam_info;

    public:
        // public variables
        /***new part***/ bool first_frame;
        bool init;                 // initialization indicator
        int iter;                  // final iteration for display
        Eigen::Affine3f transform; // transformation matrix
        Eigen::Affine3f prev_transform;
        Eigen::Affine3f accum_transform;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    private:
        // private functions
        
        /**
         * @brief a polynomial root finder
         * @param coef: coefficeints of the polynomial in descending order
         *              ex. [2,1,3] for 2x^2+x+3
         * @return roots: roots of the polynomial in complex number. ex. a+bi
         */
        inline Eigen::VectorXcf poly_solver(const Eigen::VectorXf &coef);

        /**
         * @brief calculate the se3 distance for giving R and T
         * @return d: se3 distance of given R and T
         */
        inline float dist_se3(const Eigen::Matrix3f &R, const Eigen::Vector3f &T);

        /**
         * @brief update transformation matrix
         */
        inline void update_tf();

        /**
         * @brief compute color inner product of ith row in fixed and jth row in moving
         * @param i: index of desired row in fixed
         * @param j: indxe of desired row in moving
         * @return CI: the inner product
         */
        inline float color_inner_product(const int i, const int j);
        
        /**
         * @brief compute color kernel
         */
        inline float color_kernel(const int i, const int j);

        /**
         * @brief isotropic (same length-scale for all dimensions) squared-exponential kernel
         * @param l: kernel characteristic length-scale, aka cvo.ell
         * @prarm s2: signal variance, square of cvo.sigma
         * @return k: n-by-m kernel matrix 
         */
        void se_kernel(const float l, const float s2);

        /**
         * @brief computes the Lie algebra transformation elements
         *        twist = [omega; v] will be updated in this function
         */
        void compute_flow();
        
        /**
         * @brief compute the integration step size
         *        step will be updated in  this function
         */
        void compute_step_size();

        /**
         * @brief transform cloud_y for current update
         */
        void transform_pcd();

        // /***new part***/
        // // computer ORB descriptors for corners
        // void ComputeDescriptor(frame* ptr_fr);

    public:
        // public funcitons

        // constructor and destructor
        cvo(const string& calib_file);
        ~cvo();

        /**
        * @brief compute function inner product and return a scalar
        */
        const inn_p function_inner_product(point_cloud *cloud_a, point_cloud *cloud_b);

        // Compute inner product in tracking part of CVO SLAM 
        void compute_innerproduct(inn_p& inn_pre, inn_p& inn_post, Eigen::Matrix<double, 6, 6>& post_hessian, \
                                Eigen::Affine3f& tran, int& inliers, inn_p& inn_fixed_pcd, inn_p& inn_moving_pcd, float& cos_angle);
        
        // Compute inner product in loop-closure detection part of CVO SLAM
        void compute_innerproduct_lc(inn_p& inn_prior, inn_p& inn_lc_prior, inn_p& inn_lc_pre, inn_p& inn_lc_post, \
                                       Eigen::Matrix<double, 6, 6>& post_hessian, Eigen::Affine3f& prior_tran, \
                                       Eigen::Affine3f& lc_prior_tran, Eigen::Affine3f& lc_prior_tran_2, Eigen::Affine3f& lc_tran, \
                                       int& inliers_svd, int& inliers_pnpransac, inn_p& inn_fixed_pcd, inn_p& inn_moving_pcd, float& cos_angle);

        /**
         * @brief initialize new point cloud and extract pcd as matrices
         */
        // Take in new RGB image and depth image and generate corresponding point clouds  
        void set_pcd(const cv::Mat &RGB_img, const cv::Mat &dep_img);

        // Compute transformation between last frame and current frame
        void match_odometry(const cv::Mat &RGB_img, const cv::Mat &dep_img, Eigen::Affine3d &transformd);
        
        // Compute transformation between current keyframe and current frame
        void match_keyframe(const cv::Mat &RGB_img, const cv::Mat &dep_img, Eigen::Affine3d &transformd);

        void update_fixed_pcd();

        void update_previous_pcd();

        void reset_keyframe(Eigen::Affine3f &odometry);

        void reset_transform(Eigen::Affine3f &odometry);
        
        // Reset the initial value of transformation between current keyframe and current frame, given transformation between last frame and current frame
        Eigen::Affine3f reset_initial(Eigen::Affine3f &odometry);

        /**
         * @brief compute Hessian of current transformation
         */
        Eigen::Matrix<double,6,6> se3_Hessian(point_cloud* cloud_a, point_cloud* cloud_b, int& inliers);

        /**
         * @brief align two rgbd pointcloud
         *        the function will iterate MAX_ITER times unless break conditions are met
         */
        void align();

        void get_fixed_and_moving_number(int& fixed_num, int& moving_num) {fixed_num = num_fixed; moving_num = num_moving;}
        void get_iteration_number(int& iteration) {iteration = iter;}
        void get_A_nonzero(int& nonzero) {nonzero = A_nonzero;}

        // void get_fixed_frame_corners(std::vector<cv::KeyPoint>& corners_) {corners_ = ptr_fixed_fr->corners;}
        // void get_moving_frame_corners(std::vector<cv::KeyPoint>& corners_) {corners_ = ptr_moving_fr->corners;}

        void get_fixed_frame_selected_points(std::vector<cv::Point2f>& selected_points_) {selected_points_ = ptr_fixed_fr->selected_points;}
        void get_moving_frame_selected_points(std::vector<cv::Point2f>& selected_points_) {selected_points_ = ptr_moving_fr->selected_points;}

        /**
        * @brief run cvo
        */ 
        // void run_cvo(const int dataset_seq,const cv::Mat& RGB_img,const cv::Mat& dep_img, string pcd_pth, string pcd_dso_pth);
};
}
#endif  // RKHS_SE3_H
