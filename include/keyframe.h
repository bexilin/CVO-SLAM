/* ----------------------------------------------------------------------------
 * Copyright 2019, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include "cfg.h"
#include "cvo_image.h"

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
// #include "local_tracker.h"
#include "frame.h"

// #include <ros/time.h>
#include <Eigen/Geometry>
#include <vector>
#include <map>

namespace cvo_slam
{

typedef boost::shared_ptr<ORB_SLAM2::ORBextractor> OrbExtractorPtr;

class Keyframe
{
public:

  Keyframe(const Image_Ptr& image_, const Transform& pose_, const cv::Mat& intrinsic_, bool mbRGB, float mDepthMapFactor, const OrbExtractorPtr& ORBextractor, float mbf_, std::vector<cv::Point2f>& selected_points):
  image(image_), ImGray(image_->rgb), ImDepth(image_->depth), timestamp(image_->timestamp), pose(pose_), mbf(mbf_), rgb_path(image_->rgb_path)
  {
    /** 
    * In this function we reference to ORB-SLAM2 by Raúl et al.
    * https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc#L212-232
    **/
    
    if(ImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(ImGray,ImGray,CV_RGB2GRAY);
        else
            cvtColor(ImGray,ImGray,CV_BGR2GRAY);
    }
    else if(ImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(ImGray,ImGray,CV_RGBA2GRAY);
        else
            cvtColor(ImGray,ImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || ImDepth.type()!=CV_32F)
        ImDepth.convertTo(ImDepth,CV_32F,mDepthMapFactor);

    fScaleFactor = ORBextractor->GetScaleFactor();
    mvScaleFactor = ORBextractor->GetScaleFactors();
    mvLevelSigma2 = ORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = ORBextractor->GetInverseScaleSigmaSquares();

    intrinsic = intrinsic_.clone();

    CVO_selected_points.reserve(selected_points.size());
    for(std::vector<cv::Point2f>::iterator it = selected_points.begin(); it != selected_points.end(); it++){
      cv::KeyPoint kp(*it,1.0);
      CVO_selected_points.push_back(kp);
    }
  }

  ~Keyframe() {};

  int EraseObservation(int keypoint_id){
      std::map<int,int>::iterator corres = mappoints_id.find(keypoint_id);
      if(corres == mappoints_id.end()){
        // std::cout << "The keyframe observation doesn't exist" << std::endl;
        return -1;
      } 
      else{
        int map_point_id = corres->second;
        mappoints_id.erase(corres);
        return map_point_id;
      } 
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // For SuperPoint implementation
  std::string rgb_path;

  int id;
  cv::Mat ImGray;
  cv::Mat ImDepth;
  std::string timestamp;
  Transform pose;

  // Used to compute cvo inner product in loop closure
  Image_Ptr image;

  // Keypoints, descriptors 
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  // Feature and BoW vector
  DBoW2::BowVector BowVec;
  DBoW2::FeatureVector FeatVec;

  // Camera intrinsic
  cv::Mat intrinsic;

  // Correspondence of key points and map points ID 
  std::map<int,int> mappoints_id;

  // Best covisible keyframes
  std::set<int> bestCovisibleKeyframeList;

  // Scale factor between layers of image pyramid when extracting ORB features
  float fScaleFactor;

  // IR projector baseline * fx (approx)
  float mbf;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  // frames in the local map
  FrameVector frameLists; 

  // CVO selected points for visualization 
  std::vector<cv::KeyPoint> CVO_selected_points;

  // // For SuperPoint implementation
  // PyObject* SP_keypoints_and_descriptors;
};

typedef boost::shared_ptr<Keyframe> KeyframePtr;
typedef std::vector<KeyframePtr> KeyframeVector;

} // namespace cvo_slam

#endif // KEYFRAME_H_
