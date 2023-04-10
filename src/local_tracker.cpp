/**
 * This is a modified version of local_tracker.cpp from dvo (see below).
 * Changes: 1) add definition of new functions in class LocalTracker; 
 *          2) remove some original member functions and variables in class LocalTrackerImpl; 
 *          3) In class LocalTrackerImpl, claim new member functions and variables, and add function definitions; 
 *          4) change the namespace from dvo_slam to cvo_slam
 * Date: Nov 2019
 * Xi Lin, Dingyi Sun
 */

/**
 *  This file is part of dvo.
 *
 *  Copyright 2013 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "local_tracker.h"

namespace cvo_slam
{

namespace internal
{

struct LocalTrackerImpl
{
  friend class LocalTracker;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalTrackerImpl(OrbVocabularyPtr& OrbVoc, const string& strSettingsFile)
  {
    // std::cout << "here 1" << std::endl;
    cvo_keyframe.reset(new cvo::cvo(strSettingsFile));
    cvo_odometry.reset(new cvo::cvo(strSettingsFile));
    new_map = false;
    // std::cout << "here 2" << std::endl;
    // last_keyframe_pose_.setIdentity();
    // std::cout << "here 3" << std::endl;
    force_ = false;

    ORBvocabulary = OrbVoc;

    /** 
    * Here we reference to ORB-SLAM2 by Raúl et al.
    * https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc#L53-150
    **/

    // Read the configuration file for ORB features
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    intrinsic = cv::Mat::eye(3,3,CV_32F);
    intrinsic.at<float>(0,0) = fx;
    intrinsic.at<float>(1,1) = fy;
    intrinsic.at<float>(0,2) = cx;
    intrinsic.at<float>(1,2) = cy;
    
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    mbf = fSettings["Camera.bf"];

    mDepthMapFactor = fSettings["DepthMapFactor"];
    if(fabs(mDepthMapFactor)<1e-5)
        mDepthMapFactor=1;
    else
        mDepthMapFactor = 1.0f/mDepthMapFactor;

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    float keypoint_distance = fSettings["ORBextractor.keypoint_distance"];

    ORBextractor.reset(new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST,keypoint_distance));

    reference.reset(new tracking_result());

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
    //     std::cout << "Fail to intialize SPF_object" << std::endl;
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
  }

  cvo_slam::CvoPtr cvo_keyframe, cvo_odometry;

  cvo_slam::OrbVocabularyPtr ORBvocabulary;

  cvo_slam::OrbExtractorPtr ORBextractor;

  cv::Mat intrinsic;

  cvo_slam::cfg cfg_;

  bool mbRGB;

  float mbf;

  float mDepthMapFactor;

  bool new_map;

  // Transform last_keyframe_pose_;

  // Store tracking result of current reference transformation
  tracking_result_Ptr reference;

  // Store tracking result of transformation from keyframe to last frame (use multiplication of consecutive transformations)
  Transform keyframe_to_last_frame;

  bool force_;

  // PyObject*SPF_object, *read_img_function;

  LocalTracker::AcceptSignal accept_;
  LocalTracker::MapInitializedSignal map_initialized_;
  LocalTracker::MapCompleteSignal map_complete_;

};
}

LocalTracker::LocalTracker(OrbVocabularyPtr& OrbVoc, const string& strSettingsFile) :
    impl_(new internal::LocalTrackerImpl(OrbVoc, strSettingsFile))
{
  // pcd_num_file.open("/home/xi/pcd_num_temp.txt");
  // pcd_num_file_kf.open("/home/xi/pcd_num_kf_temp.txt");
  // cos_angle_file.open("/home/xi/cos_angle_file.txt");
  // cos_angle_file_kf.open("/home/xi/cos_angle_file_kf.txt");
}

LocalTracker::~LocalTracker()
{
  // pcd_num_file.close();
  // pcd_num_file_kf.close();
  // Py_Finalize();
  // cos_angle_file.close();
  // cos_angle_file_kf.close();
}

LocalMap::Ptr LocalTracker::getLocalMap() const
{
  return local_map_;
}

void LocalTracker::getCurrentPose(Transform& pose)
{
  local_map_->getCurrentFramePose(pose);
}

boost::signals2::connection LocalTracker::addAcceptCallback(const AcceptCallback& callback)
{
  return impl_->accept_.connect(callback);
}

boost::signals2::connection LocalTracker::addMapCompleteCallback(const MapCompleteCallback& callback)
{
  return impl_->map_complete_.connect(callback);
}

boost::signals2::connection LocalTracker::addMapInitializedCallback(const MapInitializedCallback& callback)
{
  return impl_->map_initialized_.connect(callback);
}

void LocalTracker::initNewLocalMap(const Image_Ptr& keyframe, const Image_Ptr& frame, const Transform& keyframe_pose)
{
  tracking_result r_odometry;
  
  // Initialize the CVO object for consecutive frame tracking with the first frame
  impl_->cvo_odometry->set_pcd(keyframe->rgb, keyframe->depth);

  // Initialize the CVO object for keyframe based tracking with the first frame
  impl_->cvo_keyframe->set_pcd(keyframe->rgb, keyframe->depth);

  impl_->cvo_odometry->match_odometry(frame->rgb, frame->depth, r_odometry.transform);
  // impl_->last_keyframe_pose_ = r_odometry.transform;

  // record number of points in fixed and moving point clouds, number of inliers paris, and Hessian 
  int num_fixed = 0;
  int num_moving = 0;
  int A_nonzeros = 0;
  int inliers = 0;
  int iteration = 0;
  impl_->cvo_odometry->get_fixed_and_moving_number(num_fixed, num_moving);
  impl_->cvo_odometry->get_A_nonzero(A_nonzeros);
  impl_->cvo_odometry->get_iteration_number(iteration);
  // pcd_num_file << keyframe->timestamp << " " << frame->timestamp << " " << num_fixed << " " << num_moving << " " << A_nonzeros << " ";

  // // Eigen::Quaterniond q2(r_odometry.transform.rotation());
  // // pcd_num_file << r_odometry.transform.translation()(0) << " " << r_odometry.transform.translation()(1) << " " << r_odometry.transform.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " " << std::endl;

  Eigen::Affine3f tran = r_odometry.transform.cast<float>();
  impl_->cvo_odometry->compute_innerproduct(r_odometry.inn_pre, r_odometry.inn_post, r_odometry.post_hessian, tran, inliers, r_odometry.inn_fixed_pcd, r_odometry.inn_moving_pcd, r_odometry.cos_angle);

  // pcd_num_file << inliers << " " << iteration << " ";
  // for(int i = 0; i < 6; i++){
  //   for(int j = 0; j < 6; j++){
  //     pcd_num_file << r_odometry.post_hessian(i,j) << " ";
  //   }
  // }
  // pcd_num_file << std::endl; 

  r_odometry.information = r_odometry.post_hessian;

  // printout to cos_angle_file
  cos_angle_file << keyframe->timestamp << " " << frame->timestamp << " ";
  Eigen::Quaterniond q2(r_odometry.transform.rotation());
  cos_angle_file << r_odometry.transform.translation()(0) << " " << r_odometry.transform.translation()(1) << " " << r_odometry.transform.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " ";
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      cos_angle_file << r_odometry.post_hessian(i,j) << " ";
    }
  }
  cos_angle_file << r_odometry.inn_post.value << " " << r_odometry.inn_fixed_pcd.value << " " << r_odometry.inn_moving_pcd.value << " " << r_odometry.cos_angle << std::endl;

  /***new part***/
  std::vector<cv::Point2f> selected_points;
  impl_->cvo_odometry->get_fixed_frame_selected_points(selected_points);
  impl_->cvo_odometry->update_fixed_pcd();

  // Currently we do not have effective way to estimate the information of a given transformation. Here we use a rough estimation based on the CVO inner
  // product values before and after registration.  
  // r_odometry.information = (r_odometry.inn_post.value > 100 || r_odometry.inn_pre.value > 25) ? 500000 * Information::Identity() : 50000 * Information::Identity();

  initNewLocalMap(keyframe, frame, r_odometry, keyframe_pose, selected_points);
}

void LocalTracker::initNewLocalMap(const Image_Ptr& keyframe, const Image_Ptr& frame, const tracking_result& r_odometry, const Transform& keyframe_pose, std::vector<cv::Point2f>& selected_points)
{
  // Create new keyframe
  KeyframePtr kf(new Keyframe(keyframe, keyframe_pose, impl_->intrinsic, impl_->mbRGB, impl_->mDepthMapFactor, impl_->ORBextractor, impl_->mbf, selected_points));

  // Extract ORB features
  impl_->ORBextractor->ExtractOrb(selected_points,kf->ImGray,kf->ImDepth,kf->keypoints,kf->descriptors);

  /***new part***/
  // Compute ORB descriptors of corners detected in CVO
  // if(kf->keypoints.size()>0) impl_->ORBextractor->ComputeDescriptorForKeypoints(kf->ImGray,kf->keypoints,kf->descriptors);

  // Compute ORB feature vector and BoW vector
  std::vector<cv::Mat> DesVec = ORB_SLAM2::Converter::toDescriptorVector(kf->descriptors);
  impl_->ORBvocabulary->transform(DesVec,kf->BowVec, kf->FeatVec,4);

  // // For SuperPoint implementation
  // // Read image
  // PyObject* img_path = Py_BuildValue("(s)", keyframe->rgb_path.c_str());
  // if(img_path == NULL){
  //     std::cout << "Fail to get img_path" << std::endl;
  //     PyErr_Print();
  //     exit(1); 
  // }

  // PyObject* img = PyObject_CallObject(impl_->read_img_function,img_path);
  // if(img == NULL){
  //     std::cout << "Fail to get img" << std::endl;
  //     PyErr_Print();
  //     exit(1); 
  // }

  // // Call method run_2 in SPF class to compute keypoints and descriptors
  // kf->SP_keypoints_and_descriptors = PyObject_CallMethod(impl_->SPF_object,"run_2","(O)",img);
  // if(kf->SP_keypoints_and_descriptors == NULL){
  //     std::cout << "Fail to get SP_keypoints_and_descriptors" << std::endl;
  //     PyErr_Print();
  //     exit(1); 
  // }

  local_map_ = LocalMap::create(kf,keyframe_pose,impl_->cfg_);
  local_map_->addFrame(frame);

  std::cout << "Initialize a new local map" << std::endl << std::endl; 
  if(impl_->cvo_keyframe->first_frame){
      impl_->cvo_keyframe->first_frame = false;
      Eigen::Affine3f transform = r_odometry.transform.cast<float>();
      impl_->cvo_keyframe->reset_transform(transform);
  }
  else {
      Eigen::Affine3f transform = r_odometry.transform.cast<float>();
      impl_->cvo_keyframe->reset_keyframe(transform);
      impl_->new_map = true;
  }

  local_map_->addKeyframeMeasurement(r_odometry);

  impl_->reference.reset(new tracking_result(r_odometry));
  impl_->keyframe_to_last_frame = r_odometry.transform;

  impl_->map_initialized_(*this, local_map_, r_odometry);
}

void LocalTracker::update(const Image_Ptr& image, Transform& pose)
{
  if(impl_->new_map) impl_->new_map = false;
  
  tracking_result r_odometry, r_keyframe;

  // Perform consecutive tracking
  impl_->cvo_odometry->match_odometry(image->rgb, image->depth, r_odometry.transform);
  Eigen::Affine3f tran = r_odometry.transform.cast<float>();

  // record number of points in fixed and moving point clouds, number of inliers paris, and Hessian 
  int num_fixed = 0;
  int num_moving = 0;
  int A_nonzeros = 0;
  int inliers = 0;
  int iteration = 0;
  impl_->cvo_odometry->get_fixed_and_moving_number(num_fixed, num_moving);
  impl_->cvo_odometry->get_A_nonzero(A_nonzeros);
  impl_->cvo_odometry->get_iteration_number(iteration);
  
  Image_Ptr last_frame = local_map_->getCurrentFrame();
  // pcd_num_file << last_frame->timestamp << " " << image->timestamp << " " << num_fixed << " " << num_moving << " " << A_nonzeros << " ";

  // // Eigen::Quaterniond q2(r_odometry.transform.rotation());
  // // pcd_num_file << r_odometry.transform.translation()(0) << " " << r_odometry.transform.translation()(1) << " " << r_odometry.transform.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " " << std::endl;

  impl_->cvo_odometry->compute_innerproduct(r_odometry.inn_pre, r_odometry.inn_post, r_odometry.post_hessian, tran, inliers, r_odometry.inn_fixed_pcd, r_odometry.inn_moving_pcd, r_odometry.cos_angle);

  // pcd_num_file << inliers << " " << iteration << " ";
  // for(int i = 0; i < 6; i++){
  //   for(int j = 0; j < 6; j++){
  //     pcd_num_file << r_odometry.post_hessian(i,j) << " ";
  //   }
  // }
  // pcd_num_file << std::endl;

  r_odometry.information = r_odometry.post_hessian;

  // printout to cos_angle_file
  cos_angle_file << last_frame->timestamp << " " << image->timestamp << " ";
  Eigen::Quaterniond q2(r_odometry.transform.rotation());
  cos_angle_file << r_odometry.transform.translation()(0) << " " << r_odometry.transform.translation()(1) << " " << r_odometry.transform.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " ";
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      cos_angle_file << r_odometry.post_hessian(i,j) << " ";
    }
  }
  cos_angle_file << r_odometry.inn_post.value << " " << r_odometry.inn_fixed_pcd.value << " " << r_odometry.inn_moving_pcd.value << " " << r_odometry.cos_angle << std::endl;

  /***new part***/
  std::vector<cv::Point2f> last_selected_points;
  std::vector<cv::Point2f> current_selected_points;
  impl_->cvo_odometry->get_fixed_frame_selected_points(last_selected_points);
  impl_->cvo_odometry->get_moving_frame_selected_points(current_selected_points);
  impl_->cvo_odometry->update_fixed_pcd();

  // Perform keyframe-based tracking
  Eigen::Affine3f transform = r_odometry.transform.cast<float>();
  Eigen::Affine3f initial_guess = impl_->cvo_keyframe->reset_initial(transform);
  Transform initial = impl_->keyframe_to_last_frame * r_odometry.transform;

  // Eigen::Quaterniond qx(initial.rotation());
  // std::cout << "intial: " << initial.translation()(0) << " " << initial.translation()(1) << " " << initial.translation()(2) << " " << qx.x() << " " << qx.y() << " " << qx.z() << " " << qx.w() << std::endl << std::endl;

  // Eigen::Affine3f initial_cast = initial.cast<float>();
  // impl_->cvo_keyframe->reset_transform(initial_cast);
  impl_->cvo_keyframe->match_keyframe(image->rgb, image->depth, r_keyframe.transform);

  // record number of points in fixed and moving point clouds, number of inliers paris, and Hessian 
  int num_fixed_2 = 0;
  int num_moving_2 = 0;
  int A_nonzeros_2 = 0;
  int inliers_2 = 0;
  int iteration_2 = 0;
  impl_->cvo_keyframe->get_fixed_and_moving_number(num_fixed_2, num_moving_2);
  impl_->cvo_keyframe->get_A_nonzero(A_nonzeros_2);
  impl_->cvo_keyframe->get_iteration_number(iteration_2);
  
  KeyframePtr current_kf = local_map_->getKeyframe();
  // pcd_num_file_kf << current_kf->timestamp << " " << image->timestamp << " " << num_fixed_2 << " " << num_moving_2 << " " << A_nonzeros_2 << " ";
  
  tran = r_keyframe.transform.cast<float>();
  impl_->cvo_keyframe->compute_innerproduct(r_keyframe.inn_pre, r_keyframe.inn_post, r_keyframe.post_hessian, tran, inliers_2, r_keyframe.inn_fixed_pcd, r_keyframe.inn_moving_pcd, r_keyframe.cos_angle);

  // pcd_num_file_kf << inliers_2 << " " << iteration_2 << " ";
  // for(int i = 0; i < 6; i++){
  //   for(int j = 0; j < 6; j++){
  //     pcd_num_file_kf << r_keyframe.post_hessian(i,j) << " ";
  //   }
  // }
  // pcd_num_file_kf << std::endl;

  r_keyframe.information = r_keyframe.post_hessian;

  r_keyframe.dis_to_keyframe = local_map_->getFrameNumber();

  // printout to cos_angle_file_kf
  cos_angle_file_kf << current_kf->timestamp << " " << image->timestamp << " ";
  Eigen::Quaterniond q3(r_keyframe.transform.rotation());
  cos_angle_file_kf << r_keyframe.transform.translation()(0) << " " << r_keyframe.transform.translation()(1) << " " << r_keyframe.transform.translation()(2) << " " << q3.x() << " " << q3.y() << " " << q3.z() << " " << q3.w() << " ";
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      cos_angle_file_kf << r_keyframe.post_hessian(i,j) << " ";
    }
  }
  cos_angle_file_kf << r_keyframe.inn_post.value << " " << r_keyframe.inn_fixed_pcd.value << " " << r_keyframe.inn_moving_pcd.value << " " << r_keyframe.cos_angle << " ";
  
  // printout of reference transformation
  Eigen::Quaterniond q4(impl_->reference->transform.rotation());
  cos_angle_file_kf << impl_->reference->transform.translation()(0) << " " << impl_->reference->transform.translation()(1) << " " << impl_->reference->transform.translation()(2) << " " << q4.x() << " " << q4.y() << " " << q4.z() << " " << q4.w() << " ";
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      cos_angle_file_kf << impl_->reference->post_hessian(i,j) << " ";
    }
  }
  cos_angle_file_kf << impl_->reference->inn_post.value << " " << impl_->reference->inn_fixed_pcd.value << " " << impl_->reference->inn_moving_pcd.value << " " << impl_->reference->cos_angle << " ";

  // print initial guess and multiplication of odometry transformation
  Eigen::Quaternionf q5(initial_guess.rotation());
  cos_angle_file_kf << initial_guess.translation()(0) << " " << initial_guess.translation()(1) << " " << initial_guess.translation()(2) << " " << q5.x() << " " << q5.y() << " " << q5.z() << " " << q5.w() << " ";
  
  Eigen::Quaterniond q6(initial.rotation());
  cos_angle_file_kf << initial.translation()(0) << " " << initial.translation()(1) << " " << initial.translation()(2) << " " << q6.x() << " " << q6.y() << " " << q6.z() << " " << q6.w() << std::endl;


  // Eigen::Quaterniond q(r_keyframe.transform.rotation());
  // pcd_num_file << r_keyframe.transform.translation()(0) << " " << r_keyframe.transform.translation()(1) << " " << r_keyframe.transform.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
  
  // std::cout << "\nr_odometry information: " << std::endl;
  // std::cout << (r_odometry.information)(0,0) << "," << (r_odometry.information)(0,1) << "," << (r_odometry.information)(0,2) << "," << (r_odometry.information)(0,3) << "," << (r_odometry.information)(0,4) << "," << (r_odometry.information)(0,5) << std::endl;
  // std::cout << (r_odometry.information)(1,0) << "," << (r_odometry.information)(1,1) << "," << (r_odometry.information)(1,2) << "," << (r_odometry.information)(1,3) << "," << (r_odometry.information)(1,4) << "," << (r_odometry.information)(1,5) << std::endl;
  // std::cout << (r_odometry.information)(2,0) << "," << (r_odometry.information)(2,1) << "," << (r_odometry.information)(2,2) << "," << (r_odometry.information)(2,3) << "," << (r_odometry.information)(2,4) << "," << (r_odometry.information)(2,5) << std::endl;
  // std::cout << (r_odometry.information)(3,0) << "," << (r_odometry.information)(3,1) << "," << (r_odometry.information)(3,2) << "," << (r_odometry.information)(3,3) << "," << (r_odometry.information)(3,4) << "," << (r_odometry.information)(3,5) << std::endl;
  // std::cout << (r_odometry.information)(4,0) << "," << (r_odometry.information)(4,1) << "," << (r_odometry.information)(4,2) << "," << (r_odometry.information)(4,3) << "," << (r_odometry.information)(4,4) << "," << (r_odometry.information)(4,5) << std::endl;
  // std::cout << (r_odometry.information)(5,0) << "," << (r_odometry.information)(5,1) << "," << (r_odometry.information)(5,2) << "," << (r_odometry.information)(5,3) << "," << (r_odometry.information)(5,4) << "," << (r_odometry.information)(5,5) << std::endl << std::endl;

  // std::cout << "\nr_keyframe information: " << std::endl;
  // std::cout << (r_keyframe.information)(0,0) << "," << (r_keyframe.information)(0,1) << "," << (r_keyframe.information)(0,2) << "," << (r_keyframe.information)(0,3) << "," << (r_keyframe.information)(0,4) << "," << (r_keyframe.information)(0,5) << std::endl;
  // std::cout << (r_keyframe.information)(1,0) << "," << (r_keyframe.information)(1,1) << "," << (r_keyframe.information)(1,2) << "," << (r_keyframe.information)(1,3) << "," << (r_keyframe.information)(1,4) << "," << (r_keyframe.information)(1,5) << std::endl;
  // std::cout << (r_keyframe.information)(2,0) << "," << (r_keyframe.information)(2,1) << "," << (r_keyframe.information)(2,2) << "," << (r_keyframe.information)(2,3) << "," << (r_keyframe.information)(2,4) << "," << (r_keyframe.information)(2,5) << std::endl;
  // std::cout << (r_keyframe.information)(3,0) << "," << (r_keyframe.information)(3,1) << "," << (r_keyframe.information)(3,2) << "," << (r_keyframe.information)(3,3) << "," << (r_keyframe.information)(3,4) << "," << (r_keyframe.information)(3,5) << std::endl;
  // std::cout << (r_keyframe.information)(4,0) << "," << (r_keyframe.information)(4,1) << "," << (r_keyframe.information)(4,2) << "," << (r_keyframe.information)(4,3) << "," << (r_keyframe.information)(4,4) << "," << (r_keyframe.information)(4,5) << std::endl;
  // std::cout << (r_keyframe.information)(5,0) << "," << (r_keyframe.information)(5,1) << "," << (r_keyframe.information)(5,2) << "," << (r_keyframe.information)(5,3) << "," << (r_keyframe.information)(5,4) << "," << (r_keyframe.information)(5,5) << std::endl << std::endl;


  // r_odometry.information = (r_odometry.inn_post.value > 100 || r_odometry.inn_pre.value > 25) ? 500000 * Information::Identity() : 50000 * Information::Identity();
  // r_keyframe.information = (r_keyframe.inn_post.value > 100 || r_keyframe.inn_pre.value > 25) ? 500000 * Information::Identity() : 50000 * Information::Identity();
 
  // Check if a new keyframe is needed
  std::cout << "Check whether a new keyframe is needed" << std::endl;  
  if(impl_->accept_(*this, r_odometry, r_keyframe) && !impl_->force_)
  {
    std::cout << "Update current local pose graph" << std::endl << std::endl;
    local_map_->addFrame(image);
    local_map_->addOdometryMeasurement(r_odometry);
    local_map_->addKeyframeMeasurement(r_keyframe);

    impl_->cvo_keyframe->update_previous_pcd();

    // impl_->last_keyframe_pose_ = r_keyframe.transform;
    impl_->keyframe_to_last_frame = initial;
  }
  else
  {
    std::cout << "Current local pose graph completes" << std::endl << std::endl;
    Transform current_pose = local_map_->getCurrentFramePose();
    
    impl_->map_complete_(*this, local_map_);

    initNewLocalMap(local_map_->getCurrentFrame(), image, r_odometry, current_pose, last_selected_points);

    // impl_->last_keyframe_pose_ = r_odometry.transform;

    // If the current frame is the final frame, force the current local pose graph to complete
    if(impl_->force_){
      local_map_->setLastMap();

      KeyframePtr kf(new Keyframe(image, local_map_->getCurrentFramePose(), impl_->intrinsic, impl_->mbRGB, impl_->mDepthMapFactor, impl_->ORBextractor, impl_->mbf, current_selected_points));

      impl_->ORBextractor->ExtractOrb(current_selected_points,kf->ImGray,kf->ImDepth,kf->keypoints,kf->descriptors);
      // /***new part***/
      // // Compute ORB descriptors of corners detected in CVO
      // if(kf->keypoints.size()>0) impl_->ORBextractor->ComputeDescriptorForKeypoints(kf->ImGray,kf->keypoints,kf->descriptors);

      std::vector<cv::Mat> DesVec = ORB_SLAM2::Converter::toDescriptorVector(kf->descriptors);
      impl_->ORBvocabulary->transform(DesVec,kf->BowVec, kf->FeatVec,4);

      // // For SuperPoint implementation
      // // Read image
      // PyObject* img_path = Py_BuildValue("(s)", image->rgb_path.c_str());
      // if(img_path == NULL){
      //     std::cout << "Fail to get img_path" << std::endl;
      //     PyErr_Print();
      //     exit(1); 
      // }

      // PyObject* img = PyObject_CallObject(impl_->read_img_function,img_path);
      // if(img == NULL){
      //     std::cout << "Fail to get img" << std::endl;
      //     PyErr_Print();
      //     exit(1); 
      // }

      // // Call method run_2 in SPF class to compute keypoints and descriptors
      // kf->SP_keypoints_and_descriptors = PyObject_CallMethod(impl_->SPF_object,"run_2","(O)",img);
      // if(kf->SP_keypoints_and_descriptors == NULL){
      //     std::cout << "Fail to get SP_keypoints_and_descriptors" << std::endl;
      //     PyErr_Print();
      //     exit(1); 
      // }

      local_map_->setLastKeyframe(kf);

      local_map_->getCurrentFramePose(pose);

      impl_->map_complete_(*this, local_map_);

      return;
    }   

  }

  local_map_->getCurrentFramePose(pose);    
}

bool LocalTracker::checkNewMap()
{
  return impl_->new_map;
}

void LocalTracker::forceCompleteCurrentLocalMap()
{
  impl_->force_ = true;
}

void LocalTracker::configure(const cfg& config)
{
  impl_->cfg_ = config;
}

}
