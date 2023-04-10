/**
 * This is a modified version of keyframe_graph.cpp from dvo (see below).
 * Changes: 1) add definition of new functions in class KeyframeGraph; 
 *          2) remove some original member functions and variables in class KeyframeGraphImpl; 
 *          3) In class KeyframeGraphImpl, claim new member functions and variables, and add function definitions; 
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

#include "keyframe_graph.h"


namespace cvo_slam
{

namespace internal
{

struct compare_BoW_score{
  bool operator()(const std::pair<int,float>& a, const std::pair<int,float>& b){
    return a.second < b.second;
  }
};

// struct FindEdgeById
// {
// public:
//   FindEdgeById(int id) : id_(id)
//   {
//   }

//   bool operator()(g2o::HyperGraph::Edge *e)
//   {
//     return e->id() == id_;
//   }

// private:
//   int id_;
// };

// static inline int combine(const short &left, const short &right)
// {
//   int lower, upper;

//   if (left < right)
//   {
//     lower = right;
//     upper = left;
//   }
//   else
//   {
//     lower = left;
//     upper = right;
//   }

//   return upper << 16 | lower;
// }

class KeyframeGraphImpl
{
public:
  friend class ::cvo_slam::KeyframeGraph;
  // static const int FirstOdometryId = 1 << 30;
  KeyframeGraphImpl(OrbVocabularyPtr& OrbVoc_, const std::string& folder_, const std::string& calib_file_) :
                        OrbVoc(OrbVoc_),
                        lc_num(0),
                        projection_num(0),
                        optimization_thread_shutdown_(false),
                        optimization_thread_(boost::bind(&KeyframeGraphImpl::execOptimization, this)),
                        id_interval_(2),
                        keyframe_vertex_id_(0),
                        // frame_vertex_id_(1),
                        mappoint_vertex_id_(1),
                        keyframe_edge_id_(0),
                        // odometry_edge_id_(1),
                        projection_edge_id_(1),
                        last_to_current_kf_transform_result(nullptr),
                        current_to_next_kf_transform_result(nullptr),
                        folder(folder_),
                        current_kf_dist(0),
                        calib_file(calib_file_)
  {
    // configure(cfg_);
    
    // g2o version used here is 20170730 (https://github.com/RainerKuemmerle/g2o/releases), can not use a newer version 
    // keyframegraph_.setAlgorithm(
    //     new g2o::OptimizationAlgorithmDogleg(
    //         new BlockSolver(
    //             new LinearSolver())));

    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    keyframegraph_.setAlgorithm(solver);

    keyframegraph_.setVerbose(false);

    // boost::filesystem::create_directory(folder_+"loop_closure_images");

    mappoint_num_file.open("/home/xi/mappoint_num_file.txt");
  }

  ~KeyframeGraphImpl()
  {
    optimization_thread_shutdown_ = true;
    new_keyframes_.push(LocalMap::Ptr());
    optimization_thread_.join();

    mappoint_num_file.close();
  }

  void configure(const cvo_slam::cfg &cfg)
  {
    cfg_ = cfg;

    // initialize orb matcher
    // std::cout << "kfg LC_MatchThreshold: " << cfg_.LC_MatchThreshold << std::endl;
    // std::cout << "kfg UseRobustKernel: " << cfg_.UseRobustKernel << std::endl;
    // std::cout << "kfg RobustKernelDelta: " << cfg_.RobustKernelDelta << std::endl;
    // std::cout << "kfg LC_MinMatch: " << cfg_.LC_MinMatch << std::endl;
    OrbMatcher.reset(new ORB_SLAM2::ORBmatcher(cfg_.LC_MatchThreshold,true,cfg_.UseRobustKernel,cfg_.RobustKernelDelta,cfg_.LC_MinMatch));
  }

  void add(const LocalMap::Ptr &keyframe)
  {
    if (cfg_.UseMultiThreading)
    {
      new_keyframes_.push(keyframe);
    }
    else
    {
      // wait until queue is empty
      tbb::mutex::scoped_lock l(queue_empty_sync_);

      newKeyframe(keyframe);
    }
  }

  // void finalOptimization()
  // {

  //   tbb::mutex::scoped_lock l;
  //   std::cerr << "final optimization, waiting for all keyframes";
  //   while (!l.try_acquire(queue_empty_sync_))
  //   {
  //     std::cerr << ".";
  //     tbb::this_tbb_thread::sleep(tbb::tick_count::interval_t(0.1));
  //   }
  //   std::cerr << std::endl;

  //   std::cerr << keyframes_.size() << " keyframes" << std::endl;

  //   // include all edges in the optimization
  //   if (cfg_.FinalOptimizationUseDenseGraph && !cfg_.OptimizationUseDenseGraph)
  //   {
  //     for (g2o::OptimizableGraph::EdgeSet::iterator e_it = keyframegraph_.edges().begin(); e_it != keyframegraph_.edges().end(); ++e_it)
  //     {
  //       // if((*e_it)->id() % 3 == 2) continue;
  //       g2o::EdgeSE3ExpmapInv *e = (g2o::EdgeSE3ExpmapInv *)(*e_it);
  //       e->setLevel(0);
  //     }
  //   }

  //   std::cerr << "optimizing..." << std::endl;

  //   keyframegraph_.setVerbose(true);
  //   keyframegraph_.initializeOptimization(0);
  //   keyframegraph_.optimize(cfg_.FinalOptimizationIterations);


  //   std::cerr << "done" << std::endl;

  //   // update keyframe database
  //   updateKeyframePosesFromGraph();

  //   map_changed_(*me_);

  //   // If you want to print edge data, enable the following two lines
  //   G2oPrint();
  //   KeyframeEdgesPrint();
  // }

private:
  typedef g2o::BlockSolver_6_3 BlockSolver;
  typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;

  void execOptimization()
  {

    bool is_locked = false;

    while (!optimization_thread_shutdown_)
    {
      LocalMap::Ptr new_keyframe;

      new_keyframes_.pop(new_keyframe);

      if (new_keyframe)
      {
        if (!is_locked)
        {
          queue_empty_sync_.lock();
          is_locked = true;
        }

        newKeyframe(new_keyframe);

        if (is_locked && new_keyframes_.empty())
        {
          queue_empty_sync_.unlock();
          is_locked = false;
        }
      }
    }
  }

  void newKeyframe(const LocalMap::Ptr &map)
  {
    tbb::mutex::scoped_lock l(new_keyframe_sync_);

    // Insert current local pose graph into the global pose graph
    KeyframePtr keyframe = insertNewKeyframe(map);

    // Early abort
    if(keyframes_.size() <= 2){
      current_kf_dist += map->getFrameNumber();
      return;
    }

    // Abort if current keyframe is too close to the last keyframe that has performed loop-closure detection
    if(!map->lastMap()){
      if(current_kf_dist < cfg_.Min_KF_interval){
        std::cout << "\n\n\n" << "Avoid performing too frequent loop-closure" << "\n\n\n";
        current_kf_dist += map->getFrameNumber();
        return;
      }
      else current_kf_dist = map->getFrameNumber();
    }
    
    // 1. Perform indirect loop-closure detection via ORB features and direct transformation estimation with CVO 
    // 2. Update visibility between map points and keyframes
    int farest_lc_keyframe = keyframe->id;
    // std::vector<int> bestCovisibleKeyframeList;
    int new_lc = detectLoopClousure_top10(keyframe, farest_lc_keyframe);

    lc_num += new_lc;

    // int new_lc = 0;

    std::cout << "\n\n\n" << "Number of loop closure constraints: " << lc_num << "\n\n\n"; 
    // std::cout << "Number of projection constraints: " << projection_num << "\n\n\n";

    // if(new_lc >= cfg_.MinConstraintDistance)
    // {
    //   // Perform keyframe pose graph optimization
    //   std::cout << "Pose graph optimization start" << std::endl;
    //   keyframegraph_.initializeOptimization(0);
    //   keyframegraph_.optimize(cfg_.OptimizationIterations);
    //   std::cout << "Pose graph optimization end" << std::endl << std::endl;

    //   // Update keyframe poses after optimization
    //   updateKeyframePosesFromGraph();
    // }

    // if(bestCovisibleKeyframeList.size() >= 3)
    // {
    //   // Perform bundle adjustment
    //   std::cout << "Bundle adjustment start" << std::endl;
    //   bundleAdjustmentForCurrentKeyframe(keyframe, bestCovisibleKeyframeList);
    //   std::cout << "Bundle adjustment end" << std::endl << std::endl;
    // }

    // if(farest_keyframe < keyframe->id){
    //   // Perform bundle adjustment
    //   std::cout << "Bundle adjustment start" << std::endl;
    //   bundleAdjustmentForCurrentKeyframe(keyframe, farest_lc_keyframe);
    //   std::cout << "Bundle adjustment end" << std::endl << std::endl;
    // }

    // Perform bundle adjustment
    std::cout << "Bundle adjustment start" << std::endl;
    bundleAdjustmentForCurrentKeyframe(keyframe, farest_lc_keyframe);
    std::cout << "Bundle adjustment end" << std::endl << std::endl;

    // if this is the last local map, set last frame to keyframe, and do loop-closure detection and keyframe pose graph optimization
    if(map->lastMap()){
      KeyframePtr keyframe_last = insertLastKeyframe(map);

      int farest_lc_keyframe_last = keyframe_last->id;
      // std::vector<int> bestCovisibleKeyframeList_last;
      int new_lc = detectLoopClousure_top10(keyframe_last, farest_lc_keyframe_last);

      lc_num += new_lc;

      // int new_lc = 0;

      std::cout << "\n\n\n" << "Number of loop closure constraints: " << lc_num << "\n\n\n"; 
      // std::cout << "Number of projection constraints: " << projection_num << "\n\n\n";

      // if(new_lc >= cfg_.MinConstraintDistance)
      // {
      //   std::cout << "Pose graph optimization start" << std::endl;
      //   keyframegraph_.initializeOptimization(0);
      //   keyframegraph_.optimize(cfg_.OptimizationIterations);
      //   std::cout << "Pose graph optimization end" << std::endl << std::endl;

      //   updateKeyframePosesFromGraph();
      // }

      // if(bestCovisibleKeyframeList.size() >= 3)
      // {
      //   // Perform bundle adjustment
      //   std::cout << "Bundle adjustment start" << std::endl;
      //   bundleAdjustmentForCurrentKeyframe(keyframe_last, bestCovisibleKeyframeList_last);
      //   std::cout << "Bundle adjustment end" << std::endl << std::endl;
      // }

      // if(farest_keyframe_last < keyframe_last->id){
      //   // Perform bundle adjustment
      //   std::cout << "Bundle adjustment start" << std::endl;
      //   bundleAdjustmentForCurrentKeyframe(keyframe_last, bestCovisibleKeyframeList_last, farest_keyframe_last);
      //   std::cout << "Bundle adjustment end" << std::endl << std::endl;
      // }

      // Perform bundle adjustment
      std::cout << "Bundle adjustment start" << std::endl;
      bundleAdjustmentForCurrentKeyframe(keyframe_last, farest_lc_keyframe_last);
      std::cout << "Bundle adjustment end" << std::endl << std::endl;

      // Perform bundle adjustment for all keyframes
      std::cout << "Final bundle adjustment start" << std::endl;
      bundleAdjustmentForAllKeyframes();
      std::cout << "Final bundle adjustment end" << std::endl;
    }

    map_changed_(*me_);
  }

  // void insertConstraint(const KeyframePtr &keyframe, const KeyframePtr &constraint, const cvo_slam::tracking_result &result)
  // {
  //   // int edge_id = combine(constraint->id, keyframe->id);
  //   int edge_id = keyframe_edge_id_;
  //   keyframe_edge_id_ += id_interval_;

  //   g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
  //   e->setId(edge_id);

  //   e->setMeasurement(internal::toSE3Quat(result.transform));

  //   e->setRobustKernel(createRobustKernel());
  //   e->setInformation(result.information);
  //   e->resize(2);

  //   e->setLevel(0);
  //   e->setUserData(new cvo_slam::tracking_result(result));

  //   e->setVertex(0, keyframegraph_.vertex(keyframe->id));
  //   e->setVertex(1, keyframegraph_.vertex(constraint->id));

  //   keyframegraph_.addEdge(e);
  // }

  void updateKeyframePosesFromGraph()
  {
    for (KeyframeVector::iterator it = keyframes_.begin(); it != keyframes_.end(); ++it)
    {
      KeyframePtr keyframe = *it;

      g2o::VertexSE3ExpmapInv *vertex = (g2o::VertexSE3ExpmapInv *)keyframegraph_.vertex(keyframe->id);

      keyframe->pose = internal::toAffine(vertex->estimateInv());
    }
  }

  // struct FindEdge
  // {
  //   int id1, id2;

  //   FindEdge(int id1, int id2) : id1(id1), id2(id2)
  //   {
  //   }

  //   bool operator()(const g2o::HyperGraph::Edge *e) const
  //   {
  //     return e->vertices().size() == 2 && ((e->vertex(0)->id() == id1 && e->vertex(1)->id() == id2) || (e->vertex(1)->id() == id1 && e->vertex(0)->id() == id2));
  //   }
  // };

  // void addGraph(g2o::OptimizableGraph *g)
  // {
  //   // Add vertices in a local pose graph to the global pose graph
  //   for (g2o::HyperGraph::VertexIDMap::iterator it = g->vertices().begin(); it != g->vertices().end(); ++it)
  //   {
  //     g2o::OptimizableGraph::Vertex *v = (g2o::OptimizableGraph::Vertex *)(it->second);
  //     if (keyframegraph_.vertex(v->id()))
  //       continue;

  //     g2o::VertexSE3ExpmapInv *v1 = (g2o::VertexSE3ExpmapInv *)v;
  //     g2o::VertexSE3ExpmapInv *v2 = new g2o::VertexSE3ExpmapInv();
  //     v2->setId(v1->id());
  //     v2->setEstimate(v1->estimate());
  //     v2->setMarginalized(v1->marginalized());
  //     v2->setUserData(v1->userData());
  //     v1->setUserData(0);
  //     v2->setHessianIndex(-1);

  //     if (!keyframegraph_.addVertex(v2))
  //     {
  //       throw std::runtime_error("failed to add vertex to g2o graph!");
  //     }
  //   }

  //   // Add edges in a local pose graph to the global pose graph, and apply Cauchy robust kernel to them 
  //   for (g2o::HyperGraph::EdgeSet::iterator it = g->edges().begin(); it != g->edges().end(); ++it)
  //   {
  //     g2o::EdgeSE3ExpmapInv *e = (g2o::EdgeSE3ExpmapInv *)(*it);
  //     g2o::EdgeSE3ExpmapInv *en = new g2o::EdgeSE3ExpmapInv();

  //     en->setId(e->id());
  //     en->setLevel(e->level());
  //     en->setRobustKernel(createRobustKernel());
  //     en->setMeasurement(e->measurement());
  //     en->setInformation(e->information());
  //     en->setUserData(e->userData());
  //     e->setUserData(0);
  //     en->resize(e->vertices().size());
  //     int cnt = 0;
  //     for (std::vector<g2o::HyperGraph::Vertex *>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it)
  //     {
  //       g2o::OptimizableGraph::Vertex *v = (g2o::OptimizableGraph::Vertex *)keyframegraph_.vertex((*it)->id());
  //       assert(v);
  //       en->setVertex(cnt++, v);
  //     }
  //     keyframegraph_.addEdge(en);
  //   }
  // }

  void addVertexToGraph(g2o::VertexSE3ExpmapInv*& v, bool first)
  {
    g2o::VertexSE3ExpmapInv* v2 = new g2o::VertexSE3ExpmapInv();
    v2->setId(keyframe_vertex_id_);
    v2->setEstimate(v->estimate());
    // v2->setMarginalized(v->marginalized());
    v2->setUserData(v->userData());
    v->setUserData(0);
    v2->setHessianIndex(-1);

    if(first) v2->setFixed(true);

    keyframegraph_.addVertex(v2);

    keyframe_vertex_id_ += id_interval_;
  }

  void addEdgeToGraph(cvo_slam::tracking_result_Ptr& result, int from, int to)
  {
    g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
    
    e->setId(keyframe_edge_id_);
    keyframe_edge_id_ += id_interval_;

    e->setMeasurement(internal::toSE3Quat(result->transform));

    // Add Cauchy robust kernel to loop-closure edges
    e->setRobustKernel(createRobustKernel());
    
    e->setInformation(result->information);
    e->resize(2);

    e->setLevel(0);
    // e->setLevel(2);

    e->setUserData(new cvo_slam::tracking_result(*result));

    e->setVertex(0, keyframegraph_.vertex(from));
    e->setVertex(1, keyframegraph_.vertex(to));

    keyframegraph_.addEdge(e);

  }

  // int detectLoopClousure(KeyframePtr& reference)
  // {
  //   int new_lc_num = 0;
  //   std::cout << std::endl << "Performing loop-closure detection" << std::endl << std::endl;
  //   KeyframePtr last_keyframe = keyframes_[keyframes_.size()-2];

  //   // The given threshold of normalized similarity score is applied here to get the minimum acceptable similarity score  
  //   float min_score = cfg_.LC_MinScoreRatio * OrbVoc->score(reference->BowVec, last_keyframe->BowVec);

  //   for(KeyframeVector::iterator it = keyframes_.begin(); it != keyframes_.end(); it++)
  //   {
  //     if((*it)->id == reference->id || std::abs((*it)->id - reference->id) == id_interval_) continue;

  //     tracking_result result;

  //     result.score = OrbVoc->score(reference->BowVec, (*it)->BowVec);

  //     // Reject loop-closure if similarity score is lower than minimum acceptable value
  //     if(result.score < min_score){
  //       // std::cout << std::endl;
  //       continue;
  //     }

  //     // std::cout << "\nkeyframe " << (*it)->id << " score: " << result.score << std::endl;
      
  //     // cv::Mat reference_pose = ORB_SLAM2::Converter::toCvMat(reference->pose);
  //     // cv::Mat current_pose = ORB_SLAM2::Converter::toCvMat((*it)->pose);

  //     pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc(new pcl::PointCloud<pcl::PointXYZ>());
  //     pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc(new pcl::PointCloud<pcl::PointXYZ>());

  //     // Match ORB features and get corresponding point clouds 
  //     // int matches = OrbMatcher->GetMatches(reference->keypoints, reference->FeatVec, reference->descriptors, reference->ImDepth, \
  //     //                                      (*it)->keypoints, (*it)->FeatVec, (*it)->descriptors, (*it)->ImDepth, reference->intrinsic, \
  //     //                                      reference->mappoints, (*it)->mappoints, reference_pc, current_pc);

  //     int matches = OrbMatcher->GetMatches(reference, (*it), keyframegraph_, id_interval_, mappoint_vertex_id_, projection_edge_id_, \
  //                                          projection_num, reference_pc, current_pc);

  //     // Reject loop-closure if number of matched features is lower than the given threshold
  //     if(matches < cfg_.LC_MinMatch){
  //       // std::cout << std::endl;
  //       continue;
  //     }

  //     result.matches = matches;
      
  //     Eigen::Matrix<double,4,4> T_cr;

  //     // Compute initial transformation estimation through SVD
  //     PointCloudRegistrator.estimateRigidTransformation(*current_pc, *reference_pc, T_cr);

  //     Eigen::Affine3f prior = ((reference->pose).inverse() * ((*it)->pose)).cast<float>();

  //     Transform lc_constraint = ORB_SLAM2::Converter::toAffine(T_cr);
  //     Eigen::Affine3f lc_prior = lc_constraint.cast<float>();


  //     cvo::rkhs_se3 cvo;

  //     // Set the initial value of CVO to be initial transformation estimation
  //     cvo.reset_initial(lc_prior);

  //     cvo.set_pcd(cfg_.dataset_seq, reference->image->rgb, reference->image->depth);

  //     cvo.match_keyframe(cfg_.dataset_seq, (*it)->image->rgb, (*it)->image->depth, result.transform);
  //     Eigen::Affine3f lc_post = result.transform.cast<float>();

  //     cvo.compute_innerproduct_lc(result.inn_prior, result.inn_lc_prior, result.inn_pre, result.inn_post, \
  //                                 result.post_hessian, prior, lc_prior, lc_post);
  //     result.information = result.post_hessian;

  //     // Reject loop-closure if the innerproduct of transformation estimation of CVO is not larger than that of any other transformations  
  //     if ((result.inn_post.value <= result.inn_pre.value) || (result.inn_post.value <= result.inn_lc_prior.value) 
  //         || (result.inn_post.value <= result.inn_prior.value)){
  //       // std::cout << std::endl;
  //       continue;
  //     }

  //     std::cout << "Accept loop-closure between keyframe " << reference->id << " and " << (*it)->id << std::endl;

  //     // Currently we do not have effective way to estimate information of a loop-closure measurement, so set the information as a large and uniform value 
  //     // result.information = ((result.post_hessian)(4,4)>1000000000)? 500000 * Information::Identity() : 50000 * Information::Identity();

  //     insertLoopClosureConstraint(reference, *it, result);

  //     new_lc_num++;
      
  //     std::cout << std::endl;
  //   }
  //   std::cout << std::endl;
  //   return new_lc_num;
  // }

  int detectLoopClousure_top10(KeyframePtr& reference, int& farest_lc_keyframe)
  {
    int new_lc_num = 0;
    // int farest_lc_keyframe = reference->id;
    // int farest_covisible_keyframe = reference->id;

    std::cout << std::endl << "Performing loop-closure detection top 10" << std::endl << std::endl;

    std::priority_queue<std::pair<int,float>,std::vector<std::pair<int,float>>,compare_BoW_score> BoW_score_sort; 

    for(size_t i = 0; i < keyframes_.size()-2; i++){
      float BoW_score = OrbVoc->score(reference->BowVec,keyframes_[i]->BowVec);
      BoW_score_sort.push(std::make_pair(i,BoW_score));
    }

    // std::cout << "check keyframe id and BoW score" << std::endl;

    OrbMatcher->ResetPoseOptimizer();
    // g2o::VertexSE3ExpmapInv* reference_vertex = (g2o::VertexSE3ExpmapInv*) keyframegraph_.vertex(reference->id);
    // OrbMatcher->AddPoseVertexToOptimizer(reference_vertex);

    for(size_t i = 0; i < 10; i++){
      if(BoW_score_sort.empty()) break;
      std::pair<int,float> front = BoW_score_sort.top();
      int keyframe_id = front.first * 2;
      std::cout << "Checking keyframe " << keyframe_id << " with BoW score " << front.second << std::endl;
      BoW_score_sort.pop();

      tracking_result result;

      result.score = front.second;

      // inliers point clouds
      // pcl::PointCloud<pcl::PointXYZ>::Ptr reference_inliers_pc(new pcl::PointCloud<pcl::PointXYZ>());
      // pcl::PointCloud<pcl::PointXYZ>::Ptr current_inliers_pc(new pcl::PointCloud<pcl::PointXYZ>());

      // keypoints for loop-closure visualization
      std::vector<cv::KeyPoint> reference_final_kp;
      std::vector<cv::KeyPoint> current_final_kp;
      std::vector<cv::DMatch> r_to_c_match;

      Eigen::Matrix<double,4,4> T_cr;

      bool check_map = false;
      // if(i < 3) check_map = true;
      
      // Match ORB features and solve the relative transformation with a PnP RANSAC solver
      Transform lc_constraint_pnpransac;
      int success = OrbMatcher->GetInitialTransformation(check_map, reference, keyframes_[front.first], keyframes_, map_points_, keyframegraph_, id_interval_, mappoint_vertex_id_, projection_edge_id_, \
                                                         projection_num, reference_final_kp, current_final_kp, r_to_c_match, lc_constraint_pnpransac, result.matches, T_cr);

      // Reject loop-closure if failing to get enough number of corresponding inliers point clouds
      if(!success){
        // std::cout << "Reject loop-closure because of too few matched features" << std::endl << std::endl;
        std::cout << std::endl;
        continue;
      }

      // result.matches = matches;
      
      // Eigen::Matrix<double,4,4> T_cr;

      // // Compute initial transformation estimation through SVD
      // // PointCloudRegistrator.estimateRigidTransformation(*current_pc, *reference_pc, T_cr);
      // PointCloudRegistrator.estimateRigidTransformation(*current_inliers_pc, *reference_inliers_pc, T_cr);

      Eigen::Affine3f prior = ((reference->pose).inverse() * (keyframes_[front.first]->pose)).cast<float>();

      result.lc_prior = ORB_SLAM2::Converter::toAffine(T_cr);
      Eigen::Affine3f lc_prior = result.lc_prior.cast<float>();

      result.lc_prior_pnpransac = lc_constraint_pnpransac;
      Eigen::Affine3f lc_prior_2 = result.lc_prior_pnpransac.cast<float>();

      // std::cout << "prior:" << std::endl;
      // for(int row=0; row<4; row++){
      //   for(int col=0; col<4; col++){
      //     std::cout << prior(row,col) << " ";
      //   }
      //   std::cout << std::endl;
      // }
      // std::cout << std::endl;
      
      // std::cout << "lc prior:" << std::endl;
      // for(int row=0; row<4; row++){
      //   for(int col=0; col<4; col++){
      //     std::cout << lc_prior(row,col) << " ";
      //   }
      //   std::cout << std::endl;
      // }
      // std::cout << std::endl;

      cvo::cvo cvo(calib_file);

      // Set the initial value of CVO to be initial transformation estimation
      cvo.reset_initial(lc_prior);

      cvo.set_pcd(reference->image->rgb, reference->image->depth);

      cvo.match_keyframe(keyframes_[front.first]->image->rgb, keyframes_[front.first]->image->depth, result.transform);
      Eigen::Affine3f lc_post = result.transform.cast<float>();

      cvo.compute_innerproduct_lc(result.inn_prior, result.inn_lc_prior, result.inn_pre, result.inn_post, \
                                  result.post_hessian, prior, lc_prior, lc_prior_2, lc_post, result.inliers_svd, result.inliers_pnpransac, \
                                  result.inn_fixed_pcd, result.inn_moving_pcd, result.cos_angle);
      result.information = result.post_hessian;

      // Reject loop-closure if the innerproduct of transformation estimation of CVO is not larger than that of all other transformations  
      // std::cout << "prior inn: " << result.inn_prior.value << " lc_prior inn: " << result.inn_lc_prior.value << " post inn: " << result.inn_post.value << std::endl;
      if ((result.inn_post.value <= result.inn_pre.value) || (result.inn_post.value <= result.inn_lc_prior.value) 
          || (result.inn_post.value <= result.inn_prior.value) || result.cos_angle < 0.1){
        std::cout << "Final transformation: Reject, CVO result does not have the largest inner product value or have too low cosine of angle" << std::endl << std::endl;
        continue;
      }
      else{
        std::cout << "Final transformation: Accept" << std::endl;
      }

      std::cout << "Accept loop-closure between keyframe " << reference->id << " and " << keyframes_[front.first]->id << std::endl;

      if(keyframes_[front.first]->id < farest_lc_keyframe) farest_lc_keyframe = keyframes_[front.first]->id;

      // if(matches <= 20) drawLoopClosure(reference_final_kp, current_final_kp, r_to_c_match, reference, keyframes_[front.first]);
      // drawLoopClosure(reference_final_kp, current_final_kp, r_to_c_match, reference, keyframes_[front.first]);

      insertLoopClosureConstraint(reference, keyframes_[front.first], result);

      new_lc_num++;
      
      std::cout << std::endl;
    }
    std::cout << "farest lc keyframe: " << farest_lc_keyframe << std::endl;
    std::cout << std::endl;

    // if(OrbMatcher->ExistingMappoints() >= 15) OrbMatcher->CheckExistingMappoints(reference, map_points_);
    // // OrbMatcher->GetBestCovisibleKeyframeList(bestCovisibleKeyframeList, farest_covisible_keyframe);
    OrbMatcher->GetBestCovisibleKeyframeList(reference);
    OrbMatcher->ReleasePoseOptimizer();

    // farest_keyframe = min(farest_lc_keyframe, farest_covisible_keyframe);
    // // farest_keyframe = farest_lc_keyframe;
    // std::cout << "farest　keyframe: " << farest_keyframe << std::endl;
    // std::cout << std::endl;

    return new_lc_num;
  }

  // void bundleAdjustmentForCurrentKeyframe(KeyframePtr& reference, std::vector<int>& bestCovisibleKeyframeList, int farest_keyframe)
  // {
  //   std::cout << "Keyframes for optimization in BA : From " << (farest_keyframe + id_interval_) << " to " << reference->id << std::endl << std::endl; 
    
  //   // initialize an optimizer
  //   g2o::SparseOptimizer optimizer;
  //   g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

  //   linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  //   g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  //   g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  //   optimizer.setAlgorithm(solver);

  //   // set for storing map point id
  //   std::set<int> map_point_list;

  //   // set for storing fixed keyframe id
  //   std::set<int> fixed_keyframe_list;

  //   // Set farest keyframe as a fixed keyframe
  //   KeyframePtr farest_kf = keyframes_[farest_keyframe / 2];
  //   assert(farest_kf->id == farest_keyframe);

  //   g2o::VertexSE3ExpmapInv* farest_kf_vertex = new g2o::VertexSE3ExpmapInv();
  //   farest_kf_vertex->setId(farest_kf->id);
  //   farest_kf_vertex->setEstimateInv(internal::toSE3Quat(farest_kf->pose));
  //   farest_kf_vertex->setHessianIndex(-1);
  //   farest_kf_vertex->setFixed(true);
  //   optimizer.addVertex(farest_kf_vertex);
  //   fixed_keyframe_list.insert(farest_kf->id);

  //   // All keyframes between farest keyframe (not included) and reference keyframe are movable
  //   for(int i = farest_keyframe + id_interval_; i <= reference->id; i += id_interval_){
  //     KeyframePtr kf = keyframes_[i / 2];
  //     assert(kf->id == i);

  //     g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
  //     v->setId(kf->id);
  //     v->setEstimateInv(internal::toSE3Quat(kf->pose));
  //     v->setHessianIndex(-1);
  //     optimizer.addVertex(v);
  //   }

  //   // Find all relative pose constraints that link two keyframes between farest keyframe (included) and reference keyframe
  //   int relative_pose_edge = 0;
  //   for(g2o::OptimizableGraph::EdgeSet::iterator edge = keyframegraph_.edges().begin(); edge != keyframegraph_.edges().end(); edge++){
  //     g2o::EdgeSE3ExpmapInv *edge_type = (g2o::EdgeSE3ExpmapInv *)(*edge);

  //     g2o::VertexSE3ExpmapInv *v1 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(0);
  //     g2o::VertexSE3ExpmapInv *v2 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(1);

  //     if(v1->id() < farest_keyframe || v2->id() < farest_keyframe) continue;

  //     g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
  //     e->setId(edge_type->id());
  //     e->setLevel(0);
  //     e->setRobustKernel(createRobustKernel());
  //     e->setMeasurement(edge_type->measurement());
  //     e->setInformation(edge_type->information());
  //     e->resize(2);
  //     e->setVertex(0, optimizer.vertex(v1->id()));
  //     e->setVertex(1, optimizer.vertex(v2->id()));
  //     optimizer.addEdge(e);

  //     relative_pose_edge++;
  //   }

  //   // // Includes all map points seen by the farest keyframe and movable keyframes, find and include fixed keyframes
  //   // int projection_edge = 0;
  //   // int projection_edge_id = 1;
  //   // for(int i = farest_keyframe; i <= reference->id; i += id_interval_){
  //   //   KeyframePtr kf = keyframes_[i / 2];
  //   //   assert(kf->id == i);

  //   //   for(std::map<int,int>::iterator map_point_id = kf->mappoints_id.begin(); map_point_id != kf->mappoints_id.end(); map_point_id++){
  //   //     // check if the map point is already included into the graph
  //   //     if(map_point_list.find(map_point_id->second) != map_point_list.end()) continue;
        
  //   //     // create a map point vertex
  //   //     MappointPtr map_point = map_points_[(map_point_id->second - 1)/2];
  //   //     assert(map_point->id == map_point_id->second);
  //   //     map_point_list.insert(map_point->id);

  //   //     g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
  //   //     map_point_vertex->setEstimate(map_point->position);
  //   //     map_point_vertex->setId(map_point->id);
  //   //     map_point_vertex->setMarginalized(true);
  //   //     optimizer.addVertex(map_point_vertex);

  //   //     // Include all visible keyframes
  //   //     for(std::map<int,int>::iterator visible_kf_id = map_point->keypoints_id.begin(); visible_kf_id != map_point->keypoints_id.end(); visible_kf_id++){
  //   //       KeyframePtr visible_kf = keyframes_[visible_kf_id->first / 2];
  //   //       assert(visible_kf->id == visible_kf_id->first);

  //   //       cv::KeyPoint kp = visible_kf->keypoints[visible_kf_id->second];
  //   //       g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

  //   //       // create a projection edge
  //   //       g2o::EdgeSE3Projection* edge = new g2o::EdgeSE3Projection(ORB_SLAM2::Converter::toEigenMat(visible_kf->intrinsic));
  //   //       edge->setId(projection_edge_id);
  //   //       projection_edge_id += id_interval_;

  //   //       edge->setMeasurement(kp_xy);
  //   //       edge->setRobustKernel(createRobustKernel());
  //   //       edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * visible_kf->mvInvLevelSigma2[kp.octave]);
  //   //       edge->resize(2);
  //   //       edge->setLevel(0);
  //   //       edge->setVertex(0,map_point_vertex);

  //   //       if(visible_kf->id < farest_keyframe && fixed_keyframe_list.find(visible_kf->id) == fixed_keyframe_list.end()){
  //   //         // The keyframe is a fixed keyframe and has not been included into the graph
  //   //         g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
  //   //         v->setId(visible_kf->id);
  //   //         v->setEstimateInv(internal::toSE3Quat(visible_kf->pose));
  //   //         v->setHessianIndex(-1);
  //   //         v->setFixed(true);
  //   //         optimizer.addVertex(v);
  //   //         fixed_keyframe_list.insert(visible_kf->id);
  //   //       }

  //   //       edge->setVertex(1,optimizer.vertex(visible_kf->id));
  //   //       optimizer.addEdge(edge);

  //   //       projection_edge++;
  //   //     }
  //   //   }
  //   // }

  //   std::cout << "Number of map points in BA: " << map_point_list.size() << std::endl;
  //   std::cout << "Number of fixed keyframes in BA: " << fixed_keyframe_list.size() << std::endl;
  //   std::cout << "Number of movable keyframes in BA: " << (reference->id - farest_keyframe)/2 << std::endl;
  //   std::cout << "Number of relative pose edge in BA: " << relative_pose_edge << std::endl;
  //   // std::cout << "Number of projection edge in BA: " << projection_edge << std::endl << std::endl;

  //   // // optimize with all edges
  //   // optimizer.initializeOptimization();
  //   // optimizer.optimize(5);

  //   // // remove outlier edge from optimization
  //   // int outlier_edge = 0;
  //   // for (g2o::OptimizableGraph::EdgeSet::iterator e = optimizer.edges().begin(); e != optimizer.edges().end(); e++){
  //   //   if((*e)->id() % 2 == 0) continue;
      
  //   //   g2o::EdgeSE3Projection *edge = (g2o::EdgeSE3Projection *)(*e);
  //   //   if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
  //   //     edge->setLevel(1);
  //   //     outlier_edge++;
  //   //   }
  //   // }

  //   // std::cout << "Number of outlier projection edges: " << outlier_edge << std::endl << std::endl;

  //   // optimize without outliers
  //   optimizer.initializeOptimization(0);
  //   optimizer.optimize(cfg_.OptimizationIterations);

  //   // update pose of movable keyframes
  //   for(int i = farest_keyframe + id_interval_; i <= reference->id; i += id_interval_){
  //     KeyframePtr kf = keyframes_[i / 2];
  //     assert(kf->id == i);
      
  //     g2o::VertexSE3ExpmapInv* v_in_current_graph = (g2o::VertexSE3ExpmapInv*) optimizer.vertex(kf->id);
  //     g2o::VertexSE3ExpmapInv* v_in_kf_graph = (g2o::VertexSE3ExpmapInv*) keyframegraph_.vertex(kf->id);

  //     kf->pose = internal::toAffine(v_in_current_graph->estimateInv());
  //     v_in_kf_graph->setEstimate(v_in_current_graph->estimate());
  //   }

  //   // // update position of map points
  //   // for(std::set<int>::iterator map_point_id = map_point_list.cbegin(); map_point_id != map_point_list.cend(); map_point_id++){
  //   //   MappointPtr map_point = map_points_[((*map_point_id) - 1)/2];
  //   //   assert(map_point->id == (*map_point_id));

  //   //   g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) optimizer.vertex(map_point->id);
  //   //   map_point->position = map_point_vertex->estimate();
  //   // }
  // }

  void bundleAdjustmentForCurrentKeyframe(KeyframePtr& reference, int farest_lc_keyframe)
  {
    std::set<int> CovisibleKeyframeList(reference->bestCovisibleKeyframeList);
    if(CovisibleKeyframeList.size()>0) CovisibleKeyframeList.insert(reference->id);

    // // include the best covisible keyframes of reference keyframe and 4 keyframes ahead of it 
    // for(int i = reference->id; i >= max(0, reference->id - 4 * id_interval_); i -= id_interval_){
    //   KeyframePtr kf = keyframes_[i/2];
    //   assert(kf->id == i);
      
    //   if(kf->bestCovisibleKeyframeList.size()==0) continue; 
      
    //   if(CovisibleKeyframeList.find(kf->id) == CovisibleKeyframeList.end()){
    //     CovisibleKeyframeList.insert(kf->id);
    //   } 

    //   for(std::set<int>::iterator j = kf->bestCovisibleKeyframeList.begin(); j != kf->bestCovisibleKeyframeList.end(); j++){ 
    //     if(CovisibleKeyframeList.find(*j) != CovisibleKeyframeList.end()) continue;
    //     CovisibleKeyframeList.insert(*j);
    //   }
    // }

    // // Include all best covisibile keyframes of the best covisible keyframes of current keyframe
    // if(reference->bestCovisibleKeyframeList.size()>0){
    //   CovisibleKeyframeList.insert(reference->id);
    //   for(std::set<int>::iterator i = reference->bestCovisibleKeyframeList.begin(); i != reference->bestCovisibleKeyframeList.end(); i++){
    //     KeyframePtr kf = keyframes_[*i/2];
    //     assert(kf->id == *i); 
        
    //     if(CovisibleKeyframeList.find(kf->id) == CovisibleKeyframeList.end()){
    //       CovisibleKeyframeList.insert(kf->id);
    //     } 

    //     if(kf->bestCovisibleKeyframeList.size()>0){
    //       for(std::set<int>::iterator j = kf->bestCovisibleKeyframeList.begin(); j != kf->bestCovisibleKeyframeList.end(); j++){ 
    //         if(CovisibleKeyframeList.find(*j) != CovisibleKeyframeList.end()) continue;
    //         CovisibleKeyframeList.insert(*j);
    //       }
    //     }
    //   }
    // }

    

    int farest_keyframe;

    std::cout << "Number of covisible keyframes: " << CovisibleKeyframeList.size() << std::endl << std::endl;
    if(CovisibleKeyframeList.size() > 0){
      std::cout << "farest covisible keyframe: " << *(CovisibleKeyframeList.begin()) << std::endl;
      farest_keyframe = min(*(CovisibleKeyframeList.begin()),farest_lc_keyframe);
    }
    else farest_keyframe = farest_lc_keyframe;

    if(farest_keyframe == reference->id){
      std::cout << "No loop closure constraint and covisible keyframe, abort" << std::endl;
      return;
    }
    
    std::cout << "Keyframes for optimization in BA : From " << (farest_keyframe + id_interval_) << " to " << reference->id << std::endl << std::endl; 
    
    // initialize an optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // set for storing map point id
    std::set<int> map_point_list;

    // set for storing fixed keyframe id
    std::set<int> fixed_keyframe_list;

    // Set farest keyframe as a fixed keyframe
    KeyframePtr farest_kf = keyframes_[farest_keyframe / 2];
    assert(farest_kf->id == farest_keyframe);

    g2o::VertexSE3ExpmapInv* farest_kf_vertex = new g2o::VertexSE3ExpmapInv();
    farest_kf_vertex->setId(farest_kf->id);
    farest_kf_vertex->setEstimateInv(internal::toSE3Quat(farest_kf->pose));
    farest_kf_vertex->setHessianIndex(-1);
    farest_kf_vertex->setFixed(true);
    optimizer.addVertex(farest_kf_vertex);
    fixed_keyframe_list.insert(farest_kf->id);

    // All keyframes between farest keyframe (not included) and reference keyframe are movable
    for(int i = farest_keyframe + id_interval_; i <= reference->id; i += id_interval_){
      KeyframePtr kf = keyframes_[i / 2];
      assert(kf->id == i);

      g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
      v->setId(kf->id);
      v->setEstimateInv(internal::toSE3Quat(kf->pose));
      v->setHessianIndex(-1);
      optimizer.addVertex(v);
    }

    // Find all relative pose constraints that link two keyframes between farest keyframe (included) and reference keyframe
    int relative_pose_edge = 0;
    for(g2o::OptimizableGraph::EdgeSet::iterator edge = keyframegraph_.edges().begin(); edge != keyframegraph_.edges().end(); edge++){
      g2o::EdgeSE3ExpmapInv *edge_type = (g2o::EdgeSE3ExpmapInv *)(*edge);

      g2o::VertexSE3ExpmapInv *v1 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(0);
      g2o::VertexSE3ExpmapInv *v2 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(1);

      if(v1->id() < farest_keyframe || v2->id() < farest_keyframe) continue;

      g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
      e->setId(edge_type->id());
      e->setLevel(0);
      e->setRobustKernel(createRobustKernel());
      e->setMeasurement(edge_type->measurement());
      e->setInformation(edge_type->information());
      e->resize(2);
      e->setVertex(0, optimizer.vertex(v1->id()));
      e->setVertex(1, optimizer.vertex(v2->id()));
      optimizer.addEdge(e);

      relative_pose_edge++;
    }

    // Includes all map points seen by covisible keyframes in the list, find and include fixed keyframes
    int projection_edge = 0;
    int projection_edge_id = 1;

    for(std::set<int>::iterator i = CovisibleKeyframeList.begin(); i != CovisibleKeyframeList.end(); i++){
      KeyframePtr kf = keyframes_[*i / 2];
      assert(kf->id == *i);

      for(std::map<int,int>::iterator map_point_id = kf->mappoints_id.begin(); map_point_id != kf->mappoints_id.end(); map_point_id++){
        // check if the map point is already included into the graph
        if(map_point_list.find(map_point_id->second) != map_point_list.end()) continue;
        
        // create a map point vertex
        MappointPtr map_point = map_points_[(map_point_id->second - 1)/2];
        assert(map_point->id == map_point_id->second);
        map_point_list.insert(map_point->id);

        g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
        map_point_vertex->setEstimate(map_point->position);
        map_point_vertex->setId(map_point->id);
        map_point_vertex->setMarginalized(true);
        optimizer.addVertex(map_point_vertex);

        // Include all visible keyframes
        for(std::map<int,int>::iterator visible_kf_id = map_point->keypoints_id.begin(); visible_kf_id != map_point->keypoints_id.end(); visible_kf_id++){
          KeyframePtr visible_kf = keyframes_[visible_kf_id->first / 2];
          assert(visible_kf->id == visible_kf_id->first);

          cv::KeyPoint kp = visible_kf->keypoints[visible_kf_id->second];
          g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

          // create a projection edge
          g2o::EdgeSE3Projection* edge = new g2o::EdgeSE3Projection(ORB_SLAM2::Converter::toEigenMat(visible_kf->intrinsic));
          edge->setId(projection_edge_id);
          projection_edge_id += id_interval_;

          edge->setMeasurement(kp_xy);
          edge->setRobustKernel(createRobustKernel());
          edge->setInformation(100 * Eigen::Matrix<double,2,2>::Identity() * visible_kf->mvInvLevelSigma2[kp.octave]);
          edge->resize(2);
          edge->setLevel(0);
          edge->setVertex(0,map_point_vertex);

          if(visible_kf->id < farest_keyframe && fixed_keyframe_list.find(visible_kf->id) == fixed_keyframe_list.end()){
            // The keyframe is a fixed keyframe and has not been included into the graph
            g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
            v->setId(visible_kf->id);
            v->setEstimateInv(internal::toSE3Quat(visible_kf->pose));
            v->setHessianIndex(-1);
            v->setFixed(true);
            optimizer.addVertex(v);
            fixed_keyframe_list.insert(visible_kf->id);
          }

          edge->setVertex(1,optimizer.vertex(visible_kf->id));
          optimizer.addEdge(edge);

          projection_edge++;
        }
      }
    }

    std::cout << "Number of map points in BA: " << map_point_list.size() << std::endl;
    std::cout << "Number of fixed keyframes in BA: " << fixed_keyframe_list.size() << std::endl;
    std::cout << "Number of movable keyframes in BA: " << (reference->id - farest_keyframe)/2 << std::endl;
    std::cout << "Number of relative pose edge in BA: " << relative_pose_edge << std::endl;
    std::cout << "Number of projection edge in BA: " << projection_edge << std::endl << std::endl;

    mappoint_num_file << map_point_list.size() << " " << projection_edge << std::endl;

    // optimize with all edges
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // remove outlier edge from optimization
    int outlier_edge = 0;
    for (g2o::OptimizableGraph::EdgeSet::iterator e = optimizer.edges().begin(); e != optimizer.edges().end(); e++){
      if((*e)->id() % 2 == 0) continue;
      
      g2o::EdgeSE3Projection *edge = (g2o::EdgeSE3Projection *)(*e);
      // if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
      if(edge->squaredError() > 9.0 || !edge->isDepthPositive()){
        edge->setLevel(1);
        outlier_edge++;

        // remove observation
        int keyframe_id = -1;
        int map_point_id = -1;
        int edge_size = edge->vertices().size();
        assert(edge_size == 2);
        for (std::vector<g2o::HyperGraph::Vertex *>::const_iterator v = edge->vertices().begin(); v != edge->vertices().end(); ++v){
          int id = (*v)->id();
          if(id % 2 == 0) keyframe_id = id;
          else map_point_id = id;
        }
        assert(keyframe_id != -1);
        assert(map_point_id != -1);

        KeyframePtr kf = keyframes_[keyframe_id / 2];
        assert(kf->id == keyframe_id);
        MappointPtr map_point = map_points_[(map_point_id - 1)/2];
        assert(map_point->id == map_point_id);

        int kp_id = map_point->EraseObservation(kf->id);
        kf->EraseObservation(kp_id);

        if(map_point->keypoints_id.size() == 1){
          int only_kf_id = map_point->keypoints_id.begin()->first;
          int only_kp_id = map_point->EraseObservation(only_kf_id);
          KeyframePtr only_kf = keyframes_[only_kf_id / 2];
          assert(only_kf->id == only_kf_id);
          only_kf->EraseObservation(only_kp_id);
        }
      }
    }

    std::cout << "Number of outlier projection edges: " << outlier_edge << std::endl << std::endl;

    // optimize without outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(cfg_.OptimizationIterations);

    int outlier_edge_2 = 0;
    for (g2o::OptimizableGraph::EdgeSet::iterator e = optimizer.edges().begin(); e != optimizer.edges().end(); e++){
      if((*e)->id() % 2 == 0) continue;
      
      g2o::EdgeSE3Projection *edge = (g2o::EdgeSE3Projection *)(*e);

      if(edge->level() > 0) continue;

      // if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
      if(edge->squaredError() > 9.0 || !edge->isDepthPositive()){
        edge->setLevel(1);
        outlier_edge_2++;

        // remove observation
        int keyframe_id = -1;
        int map_point_id = -1;
        int edge_size = edge->vertices().size();
        assert(edge_size == 2);
        for (std::vector<g2o::HyperGraph::Vertex *>::const_iterator v = edge->vertices().begin(); v != edge->vertices().end(); ++v){
          int id = (*v)->id();
          if(id % 2 == 0) keyframe_id = id;
          else map_point_id = id;
        }
        assert(keyframe_id != -1);
        assert(map_point_id != -1);

        KeyframePtr kf = keyframes_[keyframe_id / 2];
        assert(kf->id == keyframe_id);
        MappointPtr map_point = map_points_[(map_point_id - 1)/2];
        assert(map_point->id == map_point_id);

        int kp_id = map_point->EraseObservation(kf->id);
        kf->EraseObservation(kp_id);

        if(map_point->keypoints_id.size() == 1){
          int only_kf_id = map_point->keypoints_id.begin()->first;
          int only_kp_id = map_point->EraseObservation(only_kf_id);
          KeyframePtr only_kf = keyframes_[only_kf_id / 2];
          assert(only_kf->id == only_kf_id);
          only_kf->EraseObservation(only_kp_id);
        }
      }
    }

    std::cout << "Number of outlier projection edges after second optimization: " << outlier_edge_2 << std::endl << std::endl;

    // update pose of movable keyframes
    for(int i = farest_keyframe + id_interval_; i <= reference->id; i += id_interval_){
      KeyframePtr kf = keyframes_[i / 2];
      assert(kf->id == i);
      
      g2o::VertexSE3ExpmapInv* v_in_current_graph = (g2o::VertexSE3ExpmapInv*) optimizer.vertex(kf->id);
      g2o::VertexSE3ExpmapInv* v_in_kf_graph = (g2o::VertexSE3ExpmapInv*) keyframegraph_.vertex(kf->id);

      kf->pose = internal::toAffine(v_in_current_graph->estimateInv());
      v_in_kf_graph->setEstimate(v_in_current_graph->estimate());
    }

    // update position and normal of map points
    for(std::set<int>::iterator map_point_id = map_point_list.cbegin(); map_point_id != map_point_list.cend(); map_point_id++){
      MappointPtr map_point = map_points_[((*map_point_id) - 1)/2];
      assert(map_point->id == (*map_point_id));

      g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) optimizer.vertex(map_point->id);
      map_point->position = map_point_vertex->estimate();

      UpdateMapPointNormal(map_point);
    }
  }

  void UpdateMapPointNormal(MappointPtr& map_point)
  {
    if(map_point->keypoints_id.size() == 0){
      // std::cout << "The map point is not observed by any keyframe, could not update normal" << std::endl;
      return;
    }
    
    cv::Mat normal;
    cv::Mat position = ORB_SLAM2::Converter::toCvMat(map_point->position);
    for(std::map<int,int>::iterator i = map_point->keypoints_id.begin(); i != map_point->keypoints_id.end(); i++){
      KeyframePtr kf = keyframes_[i->first / 2];
      assert(kf->id == i->first);

      cv::Mat kf_pose = ORB_SLAM2::Converter::toCvMat(kf->pose);
      cv::Mat kf_center = kf_pose.rowRange(0,3).col(3);
      cv::Mat viewing_direction = position - kf_center;
      normal = normal + viewing_direction/cv::norm(viewing_direction);
    }
    normal = normal/cv::norm(normal);
    map_point->normal = normal.clone();
  }

  void bundleAdjustmentForAllKeyframes()
  {
    // initialize an optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // set for storing map point id
    std::set<int> map_point_list;

    // set for storing fixed keyframe id
    std::set<int> fixed_keyframe_list;

    // Include all keyframes, and set the first as fixed
    for(KeyframeVector::iterator i = keyframes_.begin(); i != keyframes_.end(); i++){
      KeyframePtr kf = *i;

      g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
      v->setId(kf->id);
      v->setEstimateInv(internal::toSE3Quat(kf->pose));
      v->setHessianIndex(-1);
      if(kf->id == 0){
        v->setFixed(true);
        fixed_keyframe_list.insert(kf->id);
      }
      optimizer.addVertex(v);
    }

    // Include all relative pose constraints
    int relative_pose_edge = 0;
    for(g2o::OptimizableGraph::EdgeSet::iterator edge = keyframegraph_.edges().begin(); edge != keyframegraph_.edges().end(); edge++){
      g2o::EdgeSE3ExpmapInv *edge_type = (g2o::EdgeSE3ExpmapInv *)(*edge);
      g2o::VertexSE3ExpmapInv *v1 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(0);
      g2o::VertexSE3ExpmapInv *v2 = (g2o::VertexSE3ExpmapInv *)edge_type->vertex(1);

      g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
      e->setId(edge_type->id());
      e->setLevel(0);
      e->setRobustKernel(createRobustKernel());
      e->setMeasurement(edge_type->measurement());
      e->setInformation(edge_type->information());
      e->resize(2);
      e->setVertex(0, optimizer.vertex(v1->id()));
      e->setVertex(1, optimizer.vertex(v2->id()));
      optimizer.addEdge(e);

      relative_pose_edge++;
    }

    // // Includes all map points seen by the farest keyframe and movable keyframes, find and include fixed keyframes
    // int projection_edge = 0;
    // int projection_edge_id = 1;
    // for(int i = farest_keyframe; i <= reference->id; i += id_interval_){
    //   KeyframePtr kf = keyframes_[i / 2];
    //   assert(kf->id == i);

    //   for(std::map<int,int>::iterator map_point_id = kf->mappoints_id.begin(); map_point_id != kf->mappoints_id.end(); map_point_id++){
    //     // check if the map point is already included into the graph
    //     if(map_point_list.find(map_point_id->second) != map_point_list.end()) continue;
        
    //     // create a map point vertex
    //     MappointPtr map_point = map_points_[(map_point_id->second - 1)/2];
    //     assert(map_point->id == map_point_id->second);
    //     map_point_list.insert(map_point->id);

    //     g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
    //     map_point_vertex->setEstimate(map_point->position);
    //     map_point_vertex->setId(map_point->id);
    //     map_point_vertex->setMarginalized(true);
    //     optimizer.addVertex(map_point_vertex);

    //     // Include all visible keyframes
    //     for(std::map<int,int>::iterator visible_kf_id = map_point->keypoints_id.begin(); visible_kf_id != map_point->keypoints_id.end(); visible_kf_id++){
    //       KeyframePtr visible_kf = keyframes_[visible_kf_id->first / 2];
    //       assert(visible_kf->id == visible_kf_id->first);

    //       cv::KeyPoint kp = visible_kf->keypoints[visible_kf_id->second];
    //       g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

    //       // create a projection edge
    //       g2o::EdgeSE3Projection* edge = new g2o::EdgeSE3Projection(ORB_SLAM2::Converter::toEigenMat(visible_kf->intrinsic));
    //       edge->setId(projection_edge_id);
    //       projection_edge_id += id_interval_;

    //       edge->setMeasurement(kp_xy);
    //       edge->setRobustKernel(createRobustKernel());
    //       edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * visible_kf->mvInvLevelSigma2[kp.octave]);
    //       edge->resize(2);
    //       edge->setLevel(0);
    //       edge->setVertex(0,map_point_vertex);

    //       if(visible_kf->id < farest_keyframe && fixed_keyframe_list.find(visible_kf->id) == fixed_keyframe_list.end()){
    //         // The keyframe is a fixed keyframe and has not been included into the graph
    //         g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
    //         v->setId(visible_kf->id);
    //         v->setEstimateInv(internal::toSE3Quat(visible_kf->pose));
    //         v->setHessianIndex(-1);
    //         v->setFixed(true);
    //         optimizer.addVertex(v);
    //         fixed_keyframe_list.insert(visible_kf->id);
    //       }

    //       edge->setVertex(1,optimizer.vertex(visible_kf->id));
    //       optimizer.addEdge(edge);

    //       projection_edge++;
    //     }
    //   }
    // }

    std::cout << "Number of map points in BA: " << map_point_list.size() << std::endl;
    std::cout << "Number of fixed keyframes in BA: " << fixed_keyframe_list.size() << std::endl;
    std::cout << "Number of movable keyframes in BA: " << keyframes_.size() << std::endl;
    std::cout << "Number of relative pose edge in BA: " << relative_pose_edge << std::endl;
    // std::cout << "Number of projection edge in BA: " << projection_edge << std::endl << std::endl;

    // // optimize with all edges
    // optimizer.initializeOptimization();
    // optimizer.optimize(5);

    // // remove outlier edge from optimization
    // int outlier_edge = 0;
    // for (g2o::OptimizableGraph::EdgeSet::iterator e = optimizer.edges().begin(); e != optimizer.edges().end(); e++){
    //   if((*e)->id() % 2 == 0) continue;
      
    //   g2o::EdgeSE3Projection *edge = (g2o::EdgeSE3Projection *)(*e);
    //   if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
    //     edge->setLevel(1);
    //     outlier_edge++;
    //   }
    // }

    // std::cout << "Number of outlier projection edges: " << outlier_edge << std::endl << std::endl;

    // optimize without outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(cfg_.FinalOptimizationIterations);

    // update pose of all keyframes except the first one
    for(KeyframeVector::iterator i = keyframes_.begin(); i != keyframes_.end(); i++){
      KeyframePtr kf = *i;
      if(kf->id == 0) continue;
      
      g2o::VertexSE3ExpmapInv* v_in_current_graph = (g2o::VertexSE3ExpmapInv*) optimizer.vertex(kf->id);
      g2o::VertexSE3ExpmapInv* v_in_kf_graph = (g2o::VertexSE3ExpmapInv*) keyframegraph_.vertex(kf->id);

      kf->pose = internal::toAffine(v_in_current_graph->estimateInv());
      v_in_kf_graph->setEstimate(v_in_current_graph->estimate());
    }

    // // update position of map points
    // for(std::set<int>::iterator map_point_id = map_point_list.cbegin(); map_point_id != map_point_list.cend(); map_point_id++){
    //   MappointPtr map_point = map_points_[((*map_point_id) - 1)/2];
    //   assert(map_point->id == (*map_point_id));

    //   g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) optimizer.vertex(map_point->id);
    //   map_point->position = map_point_vertex->estimate();
    // }
  }

  // void bundleAdjustmentForCurrentKeyframe(KeyframePtr& reference, std::vector<int>& bestCovisibleKeyframeList)
  // {
  //   // initialize an optimizer
  //   g2o::SparseOptimizer optimizer;
  //   g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

  //   linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  //   g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  //   g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  //   optimizer.setAlgorithm(solver);

  //   // set for storing map point id
  //   std::set<int> map_point_list;

  //   // set for storing fixed keyframe id
  //   std::set<int> fixed_keyframe_list;

  //   // set for storing movable keyframe id
  //   std::set<int> movable_keyframe_list;

  //   // include reference keyframe into the graph
  //   g2o::VertexSE3ExpmapInv* reference_vertex = new g2o::VertexSE3ExpmapInv();
  //   reference_vertex->setId(reference->id);
  //   reference_vertex->setEstimateInv(internal::toSE3Quat(reference->pose));
  //   reference_vertex->setHessianIndex(-1);
  //   optimizer.addVertex(reference_vertex);
  //   movable_keyframe_list.insert(reference->id);

  //   // include best covisible keyframes into the graph
  //   for(int i=0; i<bestCovisibleKeyframeList.size(); i++){
  //     cvo_slam::KeyframePtr kf = keyframes_[bestCovisibleKeyframeList[i] / 2];
  //     assert(kf->id == bestCovisibleKeyframeList[i]);

  //     g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
  //     v->setId(kf->id);
  //     v->setEstimateInv(internal::toSE3Quat(kf->pose));
  //     v->setHessianIndex(-1);
  //     v->setFixed(kf->id == 0);
  //     optimizer.addVertex(v);
  //     movable_keyframe_list.insert(kf->id);
  //   }

  //   // Includes all map points seen by the movable keyframes, find and include fixed keyframes
  //   for(std::set<int>::iterator movable_kf_id = movable_keyframe_list.cbegin(); movable_kf_id != movable_keyframe_list.cend(); movable_kf_id++){
  //     KeyframePtr movable_kf = keyframes_[(*movable_kf_id) / 2];
  //     assert(movable_kf->id == (*movable_kf_id));
     
  //     for(std::map<int,int>::iterator map_point_id = movable_kf->mappoints_id.begin(); map_point_id != movable_kf->mappoints_id.end(); map_point_id++){
  //       // check if the map point is already included into the graph
  //       if(map_point_list.find(map_point_id->second) != map_point_list.end()) continue;
        
  //       // create a map point vertex
  //       MappointPtr map_point = map_points_[(map_point_id->second - 1)/2];
  //       assert(map_point->id == map_point_id->second);
  //       map_point_list.insert(map_point->id);

  //       g2o::VertexPointXYZ * map_point_vertex = new g2o::VertexPointXYZ();
  //       map_point_vertex->setEstimate(map_point->position);
  //       map_point_vertex->setId(map_point->id);
  //       map_point_vertex->setMarginalized(true);
  //       optimizer.addVertex(map_point_vertex);

  //       // Include all visible keyframes
  //       for(std::map<int,int>::iterator visible_kf_id = map_point->keypoints_id.begin(); visible_kf_id != map_point->keypoints_id.end(); visible_kf_id++){
  //         KeyframePtr kf = keyframes_[visible_kf_id->first / 2];
  //         assert(kf->id == visible_kf_id->first);

  //         cv::KeyPoint kp = kf->keypoints[visible_kf_id->second];
  //         g2o::Vector2D kp_xy(kp.pt.x, kp.pt.y);

  //         // create a projection edge
  //         g2o::EdgeSE3Projection* edge = new g2o::EdgeSE3Projection(ORB_SLAM2::Converter::toEigenMat(kf->intrinsic));
  //         edge->setMeasurement(kp_xy);
  //         edge->setRobustKernel(createRobustKernel());
  //         edge->setInformation(Eigen::Matrix<double,2,2>::Identity() * kf->mvInvLevelSigma2[kp.octave]);
  //         edge->resize(2);
  //         edge->setLevel(0);
  //         edge->setVertex(0,map_point_vertex);

  //         if(movable_keyframe_list.find(kf->id) == movable_keyframe_list.end() && fixed_keyframe_list.find(kf->id) == fixed_keyframe_list.end()){
  //           // The keyframe is a fixed keyframe and has not been included into the graph
  //           g2o::VertexSE3ExpmapInv* v = new g2o::VertexSE3ExpmapInv();
  //           v->setId(kf->id);
  //           v->setEstimateInv(internal::toSE3Quat(kf->pose));
  //           v->setHessianIndex(-1);
  //           v->setFixed(true);
  //           optimizer.addVertex(v);
  //           fixed_keyframe_list.insert(kf->id);
  //         }

  //         edge->setVertex(1,optimizer.vertex(kf->id));
  //         optimizer.addEdge(edge);
  //       }
  //     }
  //   }

  //   std::cout << "Number of map points in BA: " << map_point_list.size() << std::endl;
  //   std::cout << "Number of fixed keyframes in BA: " << fixed_keyframe_list.size() << std::endl;
  //   std::cout << "Number of movable keyframes in BA: " << movable_keyframe_list.size() << std::endl << std::endl;

  //   // optimize with all edges
  //   optimizer.initializeOptimization();
  //   optimizer.optimize(5);

  //   int all_edge = 0;
  //   int outlier_edge = 0;

  //   // remove outlier edge from optimization
  //   for (g2o::OptimizableGraph::EdgeSet::iterator e = optimizer.edges().begin(); e != optimizer.edges().end(); e++){
  //     g2o::EdgeSE3Projection *edge = (g2o::EdgeSE3Projection *)(*e);
  //     if(edge->chi2() > 5.991 || !edge->isDepthPositive()){
  //       edge->setLevel(1);
  //       outlier_edge++;
  //     }
  //     all_edge++; 
  //   }

  //   std::cout << "Number of all edges: " << all_edge << std::endl;
  //   std::cout << "Number of outlier edges: " << outlier_edge << std::endl << std::endl;
    
  //   // optimize without outliers
  //   optimizer.initializeOptimization(0);
  //   optimizer.optimize(10);

  //   // update pose of movable keyframes
  //   for(std::set<int>::iterator kf_id = movable_keyframe_list.cbegin(); kf_id != movable_keyframe_list.cend(); kf_id++){
  //     KeyframePtr kf = keyframes_[(*kf_id) / 2];
  //     assert(kf->id == (*kf_id));
      
  //     g2o::VertexSE3ExpmapInv* v_in_current_graph = (g2o::VertexSE3ExpmapInv*) optimizer.vertex(kf->id);
  //     g2o::VertexSE3ExpmapInv* v_in_kf_graph = (g2o::VertexSE3ExpmapInv*) keyframegraph_.vertex(kf->id);

  //     kf->pose = internal::toAffine(v_in_current_graph->estimateInv());
  //     v_in_kf_graph->setEstimate(v_in_current_graph->estimate());
  //   }

  //   // update position of map points
  //   for(std::set<int>::iterator map_point_id = map_point_list.cbegin(); map_point_id != map_point_list.cend(); map_point_id++){
  //     MappointPtr map_point = map_points_[((*map_point_id) - 1)/2];
  //     assert(map_point->id == (*map_point_id));

  //     g2o::VertexPointXYZ* map_point_vertex = (g2o::VertexPointXYZ*) optimizer.vertex(map_point->id);
  //     map_point->position = map_point_vertex->estimate();
  //   }
  // }

  void insertLoopClosureConstraint(const KeyframePtr &reference, const KeyframePtr &current, const tracking_result &result)
  {
    // int edge_id = combine(current->id, reference->id);

    g2o::EdgeSE3ExpmapInv *e = new g2o::EdgeSE3ExpmapInv();
    
    e->setId(keyframe_edge_id_);
    keyframe_edge_id_ += id_interval_;

    e->setMeasurement(internal::toSE3Quat(result.transform));

    // Add Cauchy robust kernel to loop-closure edges
    e->setRobustKernel(createRobustKernel());
    
    e->setInformation(result.information);
    e->resize(2);

    e->setLevel(0);
    // e->setLevel(2);

    e->setUserData(new cvo_slam::tracking_result(result));

    e->setVertex(0, keyframegraph_.vertex(reference->id));
    e->setVertex(1, keyframegraph_.vertex(current->id));

    keyframegraph_.addEdge(e);
  }

  // void drawLoopClosure(const std::vector<cv::KeyPoint>& r_kp, const std::vector<cv::KeyPoint>& c_kp, const std::vector<cv::DMatch>& r_to_c_match, const KeyframePtr& reference, const KeyframePtr& current)
  // {
  //   cv::Mat r_image_out;
  //   cv::Mat c_image_out;
  //   cv::Mat r_to_c_match_out;

  //   // draw CVO selected points
  //   cv::drawKeypoints(reference->image->rgb, reference->CVO_selected_points, r_image_out,cv::Scalar(255,255,255));
  //   cv::drawKeypoints(current->image->rgb, current->CVO_selected_points, c_image_out,cv::Scalar(255,255,255));

  //   // draw key points
  //   cv::drawKeypoints(r_image_out, reference->keypoints, r_image_out,cv::Scalar(255,0,0));
  //   cv::drawKeypoints(c_image_out, current->keypoints, c_image_out,cv::Scalar(255,0,0));

  //   // draw matches
  //   cv::drawMatches(reference->image->rgb, r_kp, current->image->rgb, c_kp, r_to_c_match, r_to_c_match_out);

  //   for(size_t i = 0; i < r_kp.size(); i++){
  //     cv::Point2f r_p = r_kp[i].pt;
  //     cv::Point2f c_p = c_kp[i].pt;

  //     c_p.x += reference->image->rgb.cols; 
  //     cv::line(r_to_c_match_out,r_p,c_p,cv::Scalar(255,255,255));
  //   }

  //   std::string save_folder = folder+"loop_closure_images/"+std::to_string(reference->id)+"_"+std::to_string(current->id)+"/";
  //   boost::filesystem::create_directory(save_folder);
    
  //   std::string r_name = save_folder+std::to_string(reference->id)+".png";
  //   std::string c_name = save_folder+std::to_string(current->id)+".png";
  //   std::string r_to_c_name = save_folder+std::to_string(reference->id)+"_"+std::to_string(current->id)+".png";

  //   cv::imwrite(r_name, r_image_out);
  //   cv::imwrite(c_name, c_image_out);
  //   cv::imwrite(r_to_c_name, r_to_c_match_out);
  // }

  // KeyframePtr insertNewKeyframe(const LocalMap::Ptr &m)
  // {
  //   // Set the pose of current keyframe vertex, which is also the last frame in the last local pose graph, with 
  //   // that of last keyframe vertex and transformation between them 
  //   if (!keyframes_.empty())
  //   {
  //     g2o::HyperGraph::Vertex *last_kv_tmp = keyframegraph_.vertex(keyframe_vertex_id_ - id_interval_);

  //     if (last_kv_tmp == 0)
  //     {
  //       throw std::runtime_error("last_kv_tmp == nullptr");
  //     }

  //     g2o::VertexSE3ExpmapInv *last_kv = dynamic_cast<g2o::VertexSE3ExpmapInv *>(last_kv_tmp);
  //     g2o::OptimizableGraph::EdgeSet::iterator e = std::find_if(last_kv->edges().begin(), last_kv->edges().end(), FindEdge(keyframe_vertex_id_ - id_interval_, frame_vertex_id_));

  //     assert(e != last_kv->edges().end());

  //     g2o::EdgeSE3ExpmapInv *e_se3 = (g2o::EdgeSE3ExpmapInv *)(*e);

  //     m->setKeyframePose(internal::toAffine(last_kv->estimateInv() * e_se3->measurement()));
  //   }

  //   // Perform local pose graph optimization
  //   m->optimize();

  //   g2o::SparseOptimizer &g = m->getGraph();

  //   // int max_id = g.vertices().size();
  //   int last_local_vertex_id = id_interval_ * (g.vertices().size()-1);

  //   g2o::OptimizableGraph::VertexIDMap vertices = g.vertices();
  //   for (g2o::OptimizableGraph::VertexIDMap::iterator v_it = vertices.begin(); v_it != vertices.end(); ++v_it)
  //   {
  //     g.changeId(v_it->second, frame_vertex_id_ + v_it->second->id());
  //   }
    
  //   int last_local_edge_id = id_interval_ * (g.edges().size()-1);

  //   for (g2o::OptimizableGraph::EdgeSet::iterator e_it = g.edges().begin(); e_it != g.edges().end(); ++e_it)
  //   {
  //     g2o::EdgeSE3ExpmapInv *e = (g2o::EdgeSE3ExpmapInv *)(*e_it);
  //     int local_id = e->id();
  //     e->setId(odometry_edge_id_ + local_id);
  //     e->setLevel(cfg_.OptimizationUseDenseGraph ? 0 : 2);
  //   }

  //   // Add current local pose graph to global pose graph 
  //   addGraph(&g);

  //   // Change the vertex type of current keyframe vertex from a normal frame vertex to a keyframe vertex
  //   g2o::VertexSE3ExpmapInv *kv = (g2o::VertexSE3ExpmapInv *)keyframegraph_.vertex(frame_vertex_id_);
  //   if (kv == 0)
  //   {
  //     throw std::runtime_error("kv == nullptr");
  //   }
  //   if (!keyframegraph_.changeId(kv, keyframe_vertex_id_))
  //   {
  //     throw std::runtime_error("keyframegraph_.changeId(kv, keyframe_vertex_id_) failed!");
  //   }

  //   if (!keyframes_.empty())
  //   {
  //     // Find the edge between last keyframe vertex and current keyframe vertex 
  //     g2o::OptimizableGraph::EdgeSet::iterator ke = std::find_if(kv->edges().begin(), kv->edges().end(), FindEdge(keyframe_vertex_id_ - id_interval_, keyframe_vertex_id_));

  //     assert(ke != kv->edges().end());

  //     // Change the edge type from a normal edge to a keyframe edge
  //     g2o::OptimizableGraph::Edge *e = (g2o::OptimizableGraph::Edge *)(*ke);
  //     e->setId(keyframe_edge_id_);
  //     e->setLevel(0);
  //     // e->setLevel(2);
  //   }
  //   else
  //   {
  //     kv->setFixed(true);
  //   }

  //   KeyframePtr keyframe = m->getKeyframe();

  //   keyframe->id = keyframe_vertex_id_;

  //   kv->setUserData(new cvo_slam::Timestamped(keyframe->timestamp));

  //   keyframes_.push_back(keyframe);

  //   // Increment ids
  //   frame_vertex_id_ += last_local_vertex_id;
  //   keyframe_vertex_id_ += id_interval_;
  //   odometry_edge_id_ += (last_local_edge_id + id_interval_);
  //   keyframe_edge_id_ += id_interval_;

  //   return keyframe;
  // }

  KeyframePtr insertNewKeyframe(const LocalMap::Ptr &m)
  { 
    // local map optimization
    if(!m->lastMap()) m->optimize();

    g2o::SparseOptimizer &g = m->getGraph();
    
    g2o::VertexSE3ExpmapInv* first_vertex = (g2o::VertexSE3ExpmapInv*) g.vertex(0);
    g2o::VertexSE3ExpmapInv* last_vertex = (g2o::VertexSE3ExpmapInv*) g.vertex(g.vertices().size()-1);

    // find the edge linking the first and last frame in the current local map, which is also the edge between the current and next keyframe 
    for(g2o::OptimizableGraph::EdgeSet::iterator e = last_vertex->edges().begin(); e != last_vertex->edges().end(); e++)
    {
      if ((*e)->vertices().size() == 2 && ((*e)->vertex(0)->id() == 0 && (*e)->vertex(1)->id() == g.vertices().size()-1)){
        g2o::EdgeSE3ExpmapInv *e_se3 = (g2o::EdgeSE3ExpmapInv *)(*e);
        // std::cout << "here 1" << std::endl;
        current_to_next_kf_transform_result.reset(new cvo_slam::tracking_result(*((cvo_slam::tracking_result *)(e_se3->userData()))));
        // std::cout << "here 2" << std::endl;
      }
    }

    assert(current_to_next_kf_transform_result.get() != nullptr);

    KeyframePtr keyframe = m->getKeyframe();
    keyframe->id = keyframe_vertex_id_;

    // retrieve the relative pose between the current keyframe and each frame after local map optimization 
    if(g.vertices().size()>2){
      for(size_t i = 1; i < g.vertices().size()-1; i++){
        g2o::VertexSE3ExpmapInv* intermediate_frame = (g2o::VertexSE3ExpmapInv*) g.vertex(i);
        Transform relative_pose = internal::toAffine(first_vertex->estimate() * intermediate_frame->estimateInv());
        std::string timestamp = ((cvo_slam::Timestamped *)((intermediate_frame)->userData()))->timestamp;
        FramePtr frame(new Frame(timestamp, relative_pose));
        keyframe->frameLists.push_back(frame);
      }
    }

    if(keyframes_.empty()){
      keyframe->pose = internal::toAffine(first_vertex->estimateInv());
      addVertexToGraph(first_vertex,true);
    }
    // reset the current keyframe pose wiht the optimized last keyframe pose, add vertex of current keyframe
    // and edge between last and current keyframe to keyframegraph
    else{      
      Transform current_pose = keyframes_.back()->pose * last_to_current_kf_transform_result->transform;
      keyframe->pose = current_pose;
      first_vertex->setEstimateInv(internal::toSE3Quat(current_pose));
      addVertexToGraph(first_vertex,false);
      addEdgeToGraph(last_to_current_kf_transform_result, keyframe_vertex_id_-2*id_interval_, keyframe_vertex_id_-id_interval_);
    }

    keyframes_.push_back(keyframe);

    last_to_current_kf_transform_result = std::move(current_to_next_kf_transform_result);

    return keyframe;
  }

  KeyframePtr insertLastKeyframe(const LocalMap::Ptr &m)
  { 
    KeyframePtr last_keyframe = m->getLastKeyframe();
    last_keyframe->id = keyframe_vertex_id_;

    g2o::SparseOptimizer &g = m->getGraph();
    g2o::VertexSE3ExpmapInv* last_vertex = (g2o::VertexSE3ExpmapInv*) g.vertex(g.vertices().size()-1);

    // reset the current keyframe pose wiht the optimized last keyframe pose, add vertex of current keyframe
    // and edge between last and current keyframe to keyframegraph
     
    Transform current_pose = keyframes_.back()->pose * last_to_current_kf_transform_result->transform;
    last_keyframe->pose = current_pose;
    last_vertex->setEstimateInv(internal::toSE3Quat(current_pose));
    addVertexToGraph(last_vertex,false);
    addEdgeToGraph(last_to_current_kf_transform_result, keyframe_vertex_id_-2*id_interval_, keyframe_vertex_id_-id_interval_);

    keyframes_.push_back(last_keyframe);

    return last_keyframe;
  }

  // KeyframePtr insertLastKeyframe(const LocalMap::Ptr &m)
  // {
  //   // Change the vertex type of final frame vertex from a normal frame vertex to a keyframe vertex
  //   g2o::VertexSE3ExpmapInv *kv = (g2o::VertexSE3ExpmapInv *)keyframegraph_.vertex(frame_vertex_id_);
  //   if (kv == 0)
  //   {
  //     throw std::runtime_error("kv == nullptr");
  //   }
  //   if (!keyframegraph_.changeId(kv, keyframe_vertex_id_))
  //   {
  //     throw std::runtime_error("keyframegraph_.changeId(kv, keyframe_vertex_id_) failed!");
  //   }

  //   // Find the edge between final frame vertex and current keyframe vertex
  //   g2o::OptimizableGraph::EdgeSet::iterator ke = std::find_if(kv->edges().begin(), kv->edges().end(), FindEdge(keyframe_vertex_id_ - id_interval_, keyframe_vertex_id_));

  //   assert(ke != kv->edges().end());

  //   // Change the edge type from a normal edge to a keyframe edge
  //   g2o::OptimizableGraph::Edge *e = (g2o::OptimizableGraph::Edge *)(*ke);
  //   e->setId(keyframe_edge_id_);
  //   e->setLevel(0);
  //   // e->setLevel(2);

  //   KeyframePtr last_keyframe = m->getLastKeyframe();

  //   last_keyframe->id = keyframe_vertex_id_;

  //   kv->setUserData(new cvo_slam::Timestamped(last_keyframe->timestamp));

  //   keyframes_.push_back(last_keyframe);

  //   return last_keyframe;
  // }

  g2o::RobustKernel *createRobustKernel()
  {
    if (cfg_.UseRobustKernel)
    {
      g2o::RobustKernel *k = new g2o::RobustKernelCauchy();
      k->setDelta(cfg_.RobustKernelDelta);

      return k;
    }
    else
    {
      return 0;
    }
  }

  // Debug function for printing the data of all non loop-closure edges in the global pose graph, not used by default 
  // void G2oPrint()
  // {
  //   std::ofstream g2o_file;

  //   // Modify the address to your local address
  //   g2o_file.open("/home/xi/edges_no_lc.txt");

  //   g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  //   for (g2o::OptimizableGraph::EdgeSet::iterator iter = keyframegraph_.edges().begin(); iter != keyframegraph_.edges().end(); ++iter)
  //   { 
  //     if ((*iter)->id() % id_interval_ == 2) continue;
  //     g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);

  //     g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
  //     g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);

  //     if (it->id() % id_interval_ == 0 && std::abs(temp_v1->id() - temp_v2->id()) != id_interval_) continue;

  //     g2o_file << temp_v1->id() << " " << temp_v2->id() << " ";
  //     g2o_file << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";
      
  //     g2o::Vector7d measurement;
  //     it->getMeasurementData(measurement);
  //     for (size_t i = 0; i < 7; ++i)
  //     {
  //       g2o_file << measurement(i) << " ";
  //     }

  //     // edge_information = it->information();
  //     // for (size_t i = 0; i < edge_information.rows(); ++i)
  //     //   for (size_t j = i; j < edge_information.cols(); ++j)
  //     //     g2o_file << edge_information(i, j) << " ";

  //     // Information pre = ((cvo_slam::tracking_result *)(it->userData()))->pre_hessian;
  //     // for (size_t i = 0; i < 6; ++i)
  //     // {  
  //     //   for (size_t j = 0; j < 6; ++j)
  //     //   { 
  //     //     // std::cout << pre(i,j) << " ";
  //     //     g2o_file << pre(i,j) << " ";
  //     //   }
  //     //   // std::cout << std::endl;
  //     // }
  //     // // std::cout << std::endl;
      
  //     Information post = ((cvo_slam::tracking_result *)(it->userData()))->post_hessian;
  //     for (size_t i = 0; i < 6; ++i)
  //       for (size_t j = 0; j < 6; ++j)
  //         g2o_file << post(i,j) << " ";

  //     g2o_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.value << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value << std::endl;    
  //   }
  //   g2o_file.close();
  // }

  // Debug function for printing the data of all loop-closure edges in the global pose graph, not used by default 
  // void KeyframeEdgesPrint()
  // {
  //   std::ofstream g2o_file;

  //   // Modify the address to your local address
  //   g2o_file.open("/home/xi/edges_lc.txt");

  //   g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  //   for (g2o::OptimizableGraph::EdgeSet::iterator iter = keyframegraph_.edges().begin(); iter != keyframegraph_.edges().end(); ++iter)
  //   {
  //     if ((*iter)->id() % id_interval_ == 2) continue;
  //     g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);

  //     g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
  //     g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);

  //     if (!(it->id() % id_interval_ == 0 && std::abs(temp_v1->id() - temp_v2->id()) != id_interval_)) continue;

  //     g2o_file << temp_v1->id() << " " << temp_v2->id() << " ";
  //     g2o_file << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";

  //     g2o::Vector7d measurement;
  //     it->getMeasurementData(measurement);
  //     for (size_t i = 0; i < 7; ++i)
  //     {
  //       g2o_file << measurement(i) << " ";
  //     }

  //     // edge_information = it->information();
  //     // for (size_t i = 0; i < edge_information.rows(); ++i)
  //     //   for (size_t j = i; j < edge_information.cols(); ++j)
  //     //     g2o_file << edge_information(i, j) << " ";

  //     // Information pre = ((cvo_slam::tracking_result *)(it->userData()))->pre_hessian;
  //     // for (size_t i = 0; i < 6; ++i)
  //     //   for (size_t j = 0; j < 6; ++j)
  //     //     g2o_file << pre(i,j) << " ";
      
  //     Information post = ((cvo_slam::tracking_result *)(it->userData()))->post_hessian;
  //     for (size_t i = 0; i < 6; ++i)
  //       for (size_t j = 0; j < 6; ++j)
  //         g2o_file << post(i,j) << " ";

  //     g2o_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.value << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value << std::endl;
  //   }
  //   g2o_file.close();
  // }

  // // Debug function for printing the data of all non loop-closure edges in the global pose graph, not used by default 
  // void G2oPrint()
  // {
  //   std::ofstream g2o_file;

  //   // Modify the address to your local address
  //   g2o_file.open("/home/xi/edges_no_lc.txt");

  //   double vertex_pose[7];
  //   double edge_pose[7];
  //   bool abandon = false;
  //   int pre_id;

  //   g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  //   for (g2o::OptimizableGraph::EdgeSet::iterator iter = keyframegraph_.edges().begin(); iter != keyframegraph_.edges().end(); ++iter)
  //   {
  //     g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);
  //     std::stringstream vertices_id;

  //     pre_id = 0;
  //     g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
  //     g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);

  //     if ((temp_v1->id() < FirstOdometryId) && (temp_v2->id() < FirstOdometryId) && (abs(temp_v1->id() - temp_v2->id()) != 1))
  //     {
  //         continue;
  //     }

  //     for (g2o::HyperGraph::VertexContainer::iterator i = (it->vertices()).begin(); i != (it->vertices()).end(); ++i)
  //     {
  //       pre_id = (*i)->id();
  //       vertices_id << (*i)->id() - 1 << " ";
  //     }

  //     vertices_id << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";


  //     g2o_file << vertices_id.str();
  //     it->getMeasurementData(edge_pose);
  //     for (size_t i = 0; i < 7; ++i)
  //     {
  //       g2o_file << edge_pose[i] << " ";
  //     }
  //     edge_information = it->information();
  //     for (size_t i = 0; i < edge_information.rows(); ++i)
  //       for (size_t j = i; j < edge_information.cols(); ++j)
  //         g2o_file << edge_information(i, j) << " ";
  //     g2o_file << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.value << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value  
  //     << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.num << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.num 
  //      << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.num_e << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.num_e << std::endl;    
  //   }
  //   g2o_file.close();
  // }

  // // Debug function for printing the data of all loop-closure edges in the global pose graph, not used by default 
  // void KeyframeEdgesPrint()
  // {
  //   std::ofstream g2o_file;

  //   // Modify the address to your local address
  //   g2o_file.open("/home/xi/edges_lc.txt");

  //   double vertex_pose[7];
  //   double edge_pose[7];
  //   int pre_id;
  //   bool abandon = false;

  //   g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  //   for (g2o::OptimizableGraph::EdgeSet::iterator iter = keyframegraph_.edges().begin(); iter != keyframegraph_.edges().end(); ++iter)
  //   {
  //     g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);
  //     std::stringstream vertices_id;
  //     pre_id = 0;
  //     for (g2o::HyperGraph::VertexContainer::iterator i = (it->vertices()).begin(); i != (it->vertices()).end(); ++i)
  //     {
        
  //       if ((*i)->id() > FirstOdometryId || ((*i)->id() - pre_id) == 1)
  //       {
  //         abandon = true;
  //         break;
  //       }
  //       vertices_id << (*i)->id() - 1 << " ";

  //       pre_id = (*i)->id();
  //     }
  //     g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
  //     g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);
  //     vertices_id << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";

  //     if (abandon)
  //     {
  //       abandon = false;
  //       continue;
  //     }

  //     g2o_file << vertices_id.str();
  //     it->getMeasurementData(edge_pose);
  //     for (size_t i = 0; i < 7; ++i)
  //     {
  //       g2o_file << edge_pose[i] << " ";
  //     }

  //     g2o_file << ((cvo_slam::tracking_result *)(it->userData()))->matches << " " << ((cvo_slam::tracking_result *)(it->userData()))->score 
  //     << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_prior.value << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_lc_prior.value
  //     << " " << ((cvo_slam::tracking_result *)(it->userData()))->inn_pre.value << " "  << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value << std::endl;
  //   }
  //   g2o_file.close();
  // }


  // Variables in tbb implementation for parallel computing, remaining furthur development (Please do not enable multithreading in config.txt)   
  tbb::concurrent_bounded_queue<LocalMap::Ptr> new_keyframes_;
  bool optimization_thread_shutdown_;
  tbb::tbb_thread optimization_thread_;
  tbb::mutex new_keyframe_sync_, queue_empty_sync_;
  

  KeyframeVector keyframes_; 
  MappointVector map_points_;
  // int id_interval_, keyframe_vertex_id_, frame_vertex_id_, mappoint_vertex_id_, keyframe_edge_id_, odometry_edge_id_, projection_edge_id_;
  int id_interval_, keyframe_vertex_id_, mappoint_vertex_id_, keyframe_edge_id_, projection_edge_id_;
  int lc_num, projection_num;

  g2o::SparseOptimizer keyframegraph_;
  
  cvo_slam::cfg cfg_;

  KeyframeGraph *me_;

  boost::signals2::signal<KeyframeGraph::MapChangedCallbackSignature> map_changed_;

  cvo_slam::OrbVocabularyPtr OrbVoc;

  cvo_slam::OrbMatcherPtr OrbMatcher;

  // pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ,double> PointCloudRegistrator;

  cvo_slam::tracking_result_Ptr last_to_current_kf_transform_result, current_to_next_kf_transform_result;

  std::string folder;

  std::string calib_file;

  int current_kf_dist;

  std::ofstream mappoint_num_file;
};
} 

KeyframeGraph::KeyframeGraph(OrbVocabularyPtr& OrbVoc_, const std::string& folder, const std::string& calib_file) : impl_(new internal::KeyframeGraphImpl(OrbVoc_, folder, calib_file))
{
  impl_->me_ = this;
}

KeyframeGraph::~KeyframeGraph()
{
}

void KeyframeGraph::configure(const cvo_slam::cfg &config)
{
  impl_->configure(config);
}

void KeyframeGraph::add(const LocalMap::Ptr &keyframe)
{
  impl_->add(keyframe);
}

// void KeyframeGraph::finalOptimization()
// {
//   impl_->finalOptimization();
// }

void KeyframeGraph::addMapChangedCallback(const KeyframeGraph::MapChangedCallback &callback)
{
  impl_->map_changed_.connect(callback);
}

const g2o::SparseOptimizer &KeyframeGraph::graph() const
{
  return impl_->keyframegraph_;
}

const KeyframeVector &KeyframeGraph::keyframes() const
{
  return impl_->keyframes_;
}

}
