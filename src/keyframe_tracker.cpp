/**
 * This is a modified version of keyframe_tracker.cpp from dvo (see below).
 * Changes: 1) add definition of new functions in class KeyframeTracker; 
 *          2) remove some original member functions and variables in class KeyframeTracker::Impl; 
 *          3) In class KeyframeTracker::Impl, claim new member functions and variables, and add function definitions; 
 *          4) change the namespace from dvo_slam to cvo_slam
 * Date: Nov 2019
 * Xi Lin, Dingyi Sun
 */

/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
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

#include "keyframe_tracker.h"

namespace cvo_slam
{

class KeyframeTracker::Impl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class ::cvo_slam::KeyframeTracker;

  Impl(const string& strVocFile, const string& strSettingsFile, const string& folder){

    std::cout << std::endl << "loading visual words, it may take some time" << std::endl;
    ORBvoc.reset(new ORB_SLAM2::ORBVocabulary());
    ORBvoc->loadFromTextFile(strVocFile);

    lt_.reset(new LocalTracker(ORBvoc, strSettingsFile));
    graph_.reset(new KeyframeGraph(ORBvoc, folder, strSettingsFile));

    // Function called when a new local pose graph is initialized
    lt_->addMapInitializedCallback(boost::bind(&KeyframeTracker::Impl::onMapInitialized, this, _1, _2, _3));

    // Function called when the current local pose graph finishes
    lt_->addMapCompleteCallback(boost::bind(&KeyframeTracker::Impl::onMapComplete, this, _1, _2));
    
    // Check if translation norm of a transformation is smaller than the given threshold
    lt_->addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionDistance, this, _1, _2, _3));

    // Check if absolute rotation angle of a transformation is smaller than the given threshold
    lt_->addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionAngle, this, _1, _2, _3));

    // Check if the CVO inner product ratio of a transformation is larger than the given threshold
    lt_->addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionInnerProductRatio, this, _1, _2, _3));

    // Check if the number of frames in the current map is larger than the given threshold
    lt_->addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionFrameNumber, this, _1, _2, _3));
  }

  tracking_result evaluation;
  // Transform last_transform_to_keyframe_;

  void onMapInitialized(const LocalTracker& lt, const LocalMap::Ptr& m, const tracking_result& r_odometry)
  {
    // last_transform_to_keyframe_ = r_odometry.transform;

    evaluation = r_odometry;
  }

  void onMapComplete(const cvo_slam::LocalTracker& lt, const cvo_slam::LocalMap::Ptr& m)
  {
    if(!cfg_.OnlyTracking)  graph_->add(m);
  }

  bool onAcceptCriterionDistance(const LocalTracker& lt, const tracking_result& r_odometry, const tracking_result& r_keyframe)
  {
    std::cout << "Translation norm (m): " << r_keyframe.transform.translation().norm() << std::endl;
    // std::cout << "Below translation threshold: " << (r_keyframe.transform.translation().norm() < cfg_.KFS_Distance) << std::endl;

    return r_keyframe.transform.translation().norm() < cfg_.KFS_Distance;
  }

  bool onAcceptCriterionAngle(const LocalTracker& lt, const tracking_result& r_odometry, const tracking_result& r_keyframe)
  {
    std::cout << "Rotation angle (degree): " << (abs(acos(0.5*(r_keyframe.transform.rotation().trace() - 1))) * 180.0 / 3.14159265) << std::endl;
    // std::cout << "Below rotation angle threshold: " << ((abs(acos(0.5*(r_keyframe.transform.rotation().trace() - 1))) * 180.0 / 3.14159265) < cfg_.KFS_Angle) << std::endl;

    return (abs(acos(0.5*(r_keyframe.transform.rotation().trace() - 1))) * 180.0 / 3.14159265) < cfg_.KFS_Angle;
  }

  bool onAcceptCriterionInnerProductRatio(const LocalTracker& lt, const tracking_result& r_odometry, const tracking_result& r_keyframe)
  {
    std::cout << "r_keyframe inn_post: " << r_keyframe.inn_post.value << std::endl;
    std::cout << "Evaluation inn_post: " << evaluation.inn_post.value << std::endl;
    std::cout << "Inner product ratio: " << (r_keyframe.inn_post.value / evaluation.inn_post.value) << std::endl;
    // std::cout << "onAcceptCriterionInnerProductRatio: " << ((r_keyframe.inn_post.value / evaluation.inn_post.value) > cfg_.FE_InnpThreshold) << std::endl;

    return (r_keyframe.inn_post.value / evaluation.inn_post.value) > cfg_.FE_InnpThreshold;
  }

  bool onAcceptCriterionFrameNumber(const LocalTracker& lt, const tracking_result& r_odometry, const tracking_result& r_keyframe)
  {
    std::cout << "Number of frames in the current local map: " << r_keyframe.dis_to_keyframe << std::endl;
    return r_keyframe.dis_to_keyframe <= cfg_.Max_KF_interval; 
  }

  void forceKeyframe()
  {
    lt_->forceCompleteCurrentLocalMap();
  }

  void init(const Transform& initial_transformation)
  {
    initial_transformation_ = initial_transformation;
    relative_transformation_.setIdentity();
  }

  void update(const Image_Ptr& current, Transform& absolute_transformation)
  {

    if(!previous_)
    {
      std::cout << "first pose" << std::endl;
      previous_ = current;
      absolute_transformation = initial_transformation_;
      return;
    }

    if(!lt_->getLocalMap())
    {
      std::cout << "second pose" << std::endl;
      lt_->initNewLocalMap(previous_, current, initial_transformation_);
      lt_->getCurrentPose(absolute_transformation);
      return;
    }

    lt_->update(current, absolute_transformation);
  }

  // void finish()
  // {
  //   graph_->finalOptimization();
  // }

  bool checkNewMap()
  {
    return lt_->checkNewMap();
  }


private:
  KeyframeGraph::Ptr graph_;
  LocalTracker::Ptr lt_;
  Transform initial_transformation_, relative_transformation_, last_absolute_transformation_;
  
  OrbVocabularyPtr ORBvoc;

  Image_Ptr previous_;

  cfg cfg_;
};

KeyframeTracker::KeyframeTracker(const string& strVocFile, const string& strSettingsFile, const string& folder) : 
impl_(new KeyframeTracker::Impl(strVocFile, strSettingsFile,folder))
{
}

KeyframeTracker::~KeyframeTracker()
{
}

void KeyframeTracker::configure(const cfg& cfg)
{
  impl_->cfg_ = cfg;
}

void KeyframeTracker::configureGraph(const cfg& cfg)
{
  impl_->graph_->configure(cfg);
}

void KeyframeTracker::configureTracker(const cfg& cfg)
{
  impl_->lt_->configure(cfg);
}

void KeyframeTracker::init()
{
  init(Transform::Identity());
}

void KeyframeTracker::init(const Transform& initial_transformation)
{
  impl_->init(initial_transformation);
}

void KeyframeTracker::update(const Image_Ptr& current, Transform& absolute_transformation)
{
  impl_->update(current, absolute_transformation);
}

bool KeyframeTracker::checkNewMap()
{
  return impl_->checkNewMap();
}

void KeyframeTracker::forceKeyframe()
{
  impl_->forceKeyframe();
}

// void KeyframeTracker::finish()
// {
//   impl_->finish();
// }

// void KeyframeTracker::serializeMap(MapSerializerInterface& serializer)
// {
//   serializer.serialize(impl_->graph_);
// }

void KeyframeTracker::writeSLAMTrajectoryAndLoopClosureFile(string& SLAM_file_, string& lc_file_)
{
  std::ofstream SLAM_file;
  SLAM_file.open(SLAM_file_.c_str());
  
  KeyframeVector keyframes = impl_->graph_->keyframes();
  
  for (size_t i = 0; i < keyframes.size(); i++){
    KeyframePtr keyframe = keyframes[i];
    Eigen::Quaterniond q(keyframe->pose.rotation());
    SLAM_file << keyframe->timestamp << " " << keyframe->pose.translation()(0) << " " << keyframe->pose.translation()(1) << " " << keyframe->pose.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
    if(keyframe->frameLists.size()>0){
      FrameVector frames = keyframe->frameLists;

      for (size_t j = 0; j < frames.size(); j++){
        FramePtr frame = frames[j];
        Transform global_pose = keyframe->pose * frame->relative_pose;
        Eigen::Quaterniond q2(global_pose.rotation());
        SLAM_file << frame->timestamp << " " << global_pose.translation()(0) << " " << global_pose.translation()(1) << " " << global_pose.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " " << std::endl;
      }
    }
  }

  SLAM_file.close();

  std::ofstream lc_file;
  lc_file.open(lc_file_.c_str());

  g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  for (g2o::OptimizableGraph::EdgeSet::iterator iter = impl_->graph_->graph().edges().begin(); iter != impl_->graph_->graph().edges().end(); ++iter)
  {
    // std::cout << "checking" << std::endl;
    if ((*iter)->id() % 2 != 0) continue;
    g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);

    g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
    g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);

    // std::cout << "get vertices" << std::endl;
    if (std::abs(temp_v1->id() - temp_v2->id()) == 2) continue;

    // std::cout << temp_v1->id() << " " << temp_v2->id() << std::endl;
    // std::cout << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << std::endl;
    lc_file << temp_v1->id() << " " << temp_v2->id() << " ";
    // std::cout << "complete id printing" << std::endl;
    lc_file << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";

    g2o::Vector7d measurement;
    it->getMeasurementData(measurement);
    for (size_t i = 0; i < 7; ++i)
    {
      lc_file << measurement(i) << " ";
    }
    
    Information post = ((cvo_slam::tracking_result *)(it->userData()))->post_hessian;
    for (size_t i = 0; i < 6; ++i)
      for (size_t j = 0; j < 6; ++j)
        lc_file << post(i,j) << " ";

    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->score << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->matches << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_prior.value << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_lc_prior.value << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value << " ";

    Transform lc_prior = ((cvo_slam::tracking_result *)(it->userData()))->lc_prior;
    Eigen::Quaterniond q(lc_prior.rotation());
    lc_file << lc_prior.translation()(0) << " " << lc_prior.translation()(1) << " " << lc_prior.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";

    // Transform lc_prior_pnpransac = ((cvo_slam::tracking_result *)(it->userData()))->lc_prior_pnpransac;
    // Eigen::Quaterniond q2(lc_prior_pnpransac.rotation());
    // lc_file << lc_prior_pnpransac.translation()(0) << " " << lc_prior_pnpransac.translation()(1) << " " << lc_prior_pnpransac.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " ";

    // lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inliers_svd << " ";
    // lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inliers_pnpransac << std::endl;

    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_fixed_pcd.value << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_moving_pcd.value << " ";
    lc_file << ((cvo_slam::tracking_result *)(it->userData()))->cos_angle << std::endl;
  }

  lc_file.close();

}

}
