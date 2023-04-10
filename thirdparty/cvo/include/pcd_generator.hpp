/* ----------------------------------------------------------------------------
 * Copyright 2019, Tzu-yuan Lin <tzuyuan@umich.edu>, Maani Ghaffari <maanigj@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   pcd_generator.hpp
 *  @author Tzu-yuan Lin, Maani Ghaffari 
 *  @brief  Header file for point cloud generator
 *  @date   September 20, 2019
 **/

#ifndef PCD_GENERATOR_H
#define PCD_GENERATOR_H

#include "data_type.h"
#include "../thirdparty/PixelSelector2.h"

#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/visualization/cloud_viewer.h>

// #include "nanoflann.hpp"
// #include "utils.h"

using namespace std;
// using namespace nanoflann;

namespace cvo{

struct compare_score{
  bool operator()(const std::pair<int,float>& a, const std::pair<int,float>& b){
    return a.second < b.second;
  }
};

// typedef std::priority_queue<std::pair<int,float>,std::vector<std::pair<int,float>>,compare_score> score_priority_queue; // pixel index, ShiTomasiScore pairs queue

// typedef KDTreeSingleIndexDynamicAdaptor<L2_Simple_Adaptor<float, PointCloud<float>> ,PointCloud<float>, 3 /*actually 2, z is always set to 0*/> keypoint_kd_tree;

class pcd_generator{
    private:
        // private variables

        int num_want;
        int num_selected;
        int dep_thres;

        int w_pyr[PYR_LEVELS];  // width for each pyramid
        int h_pyr[PYR_LEVELS];

        float* map; // map for selected pixels

        // pcl::visualization::PCLVisualizer::Ptr ds_viewer;
        int img_idx;

        camera_info cam_info;

    public:
        // public variables

        const int HALF_PATCH_SIZE = 15; // half patch size for computing ORB descriptor

    private:
        // private functions

        /**
         * @brief make image pyramid and calculate gradient for each level
         */
        void make_pyramid(frame* ptr_fr);

        /**
         * @brief select point using PixelSelector from DSO
         */
        void select_point(frame* ptr_fr);

        /**
         * @brief select point using FeatureDetector::DetectCorners from LDSO
         */
        void select_point_LDSO(frame* ptr_fr);

        /**
         * @brief visualize selected pixels in the image
         */
        void visualize_selected_pixels(frame* ptr_fr);

        /**
         * @brief retrieve points x,y,z positions from pixels
         **/
        void get_points_from_pixels(frame* ptr_fr, point_cloud* ptr_pcd);

        /**
         * @brief get features from pixels (R,G,B,dx,dy)
         **/
        void get_features(const int feature_type, frame* ptr_fr, point_cloud* ptr_pcd);

        /**
         * shi-tomasi score
         * @param frame, must have frame hessian since we need dI
         * @param u
         * @param v
         * @param halfbox
         * @return
         */
        inline float
        ShiTomasiScore(const frame* ptr_fr, const float &u, const float &v, int halfbox = 4) {

            float dXX = 0.0;
            float dYY = 0.0;
            float dXY = 0.0;

            const int box_size = 2 * halfbox;
            const int box_area = box_size * box_size;
            const int x_min = u - halfbox;
            const int x_max = u + halfbox;
            const int y_min = v - halfbox;
            const int y_max = v + halfbox;

            if (x_min < 1 || x_max >= ptr_fr->w - 1 || y_min < 1 || y_max >= ptr_fr->h - 1)
                return 0.0; // patch is too close to the boundary
            const int stride = ptr_fr->w;

            for (int y = y_min; y < y_max; ++y) {
                for (int x = x_min; x < x_max; ++x) {
                    float dx = ptr_fr->dI[y * stride + x][1];
                    float dy = ptr_fr->dI[y * stride + x][2];
                    dXX += dx * dx;
                    dYY += dy * dy;
                    dXY += dx * dy;
                }
            }

            // Find and return smaller eigenvalue:
            dXX = dXX / (2.0 * box_area);
            dYY = dYY / (2.0 * box_area);
            dXY = dXY / (2.0 * box_area);
            return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
        }

        /**
         * compute the rotation of a feature point
         * @param image the image stored in FrameHessian
         * @param pt keypoint position
         * @param u_max
         * @return
         */
        inline float IC_Angle(const frame* ptr_fr, const cv::Point2f &pt) {

            float m_01 = 0, m_10 = 0;
            const Eigen::Vector3f *center = ptr_fr->dI + int(pt.y) * ptr_fr->w + int(pt.x);

            // Treat the center line differently, v=0
            for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
                m_10 += u * center[u][0];

            // Go line by line in the circular patch
            int step = ptr_fr->w;
            for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
                // Proceed over the two lines
                float v_sum = 0;
                int d = umax[v];
                for (int u = -d; u <= d; ++u) {
                    float val_plus = center[u + v * step][0], val_minus = center[u - v * step][0];
                    v_sum += (val_plus - val_minus);
                    m_10 += u * (val_plus + val_minus);
                }
                m_01 += v * v_sum;
            }
            return atan2f(m_01, m_10);
        }

        std::vector<int> umax;  // used to compute rotation

    public:
        // public functions

        // constructor and destructor
        pcd_generator();
        ~pcd_generator();

        void set_calib(const camera_info& calib) {cam_info = calib;};

        /**
         * @brief load image and preprocess for point selector
         */
        void load_image(const cv::Mat& RGB_img,const cv::Mat& dep_img,\
                        frame* ptr_fr);

        /**
         * @brief select points and generate point clouds
         * @param feature_type: 0: HSV+gradient and normalized to 0~1
         *                      1: RGB+gradient without normalization
         */
        void create_pointcloud(const int feature_type,frame* ptr_fr, point_cloud* ptr_pcd);

        /**
         * @brief write point cloud as pcd files. this function requires pcl library
         */
        // void create_pcl_pointcloud(frame* ptr_fr, point_cloud* ptr_pcd);


};
}
#endif //PCD_GENERATOR_H