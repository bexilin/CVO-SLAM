/**
 * This is a modified version of local_map.cpp from dvo (see below).
 * Changes: 1) add definition of new functions in class LocalMap; 
 *          2) remove some original member functions and variables in class LocalMapImpl; 
 *          3) In class LocalMapImpl, claim new member functions and variables, and add function definitions; 
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

#include "local_map.h"

namespace cvo_slam
{

namespace internal
{

struct LocalMapImpl
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef g2o::BlockSolver_6_3 BlockSolver;
  typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;
  cvo_slam::KeyframePtr keyframe_;
  cvo_slam::Image_Ptr current_;
  g2o::VertexSE3ExpmapInv *keyframe_vertex_, *previous_vertex_, *current_vertex_;

  g2o::SparseOptimizer graph_;
  int id_interval_, vertex_id_, edge_id_;

  bool last_map;

  cvo_slam::cfg cfg_;

  // Only used in the last local map
  cvo_slam::KeyframePtr last_keyframe_;

  // store keyframe pose and current frame pose that comes from KF-CVO (output to Tracking trajectory), which does not include graph optimization
  Transform keyframe_pose, current_pose;

  LocalMapImpl(cvo_slam::KeyframePtr& keyframe, const Transform& cvo_keyframe_pose, const cvo_slam::cfg& cfg) :
    keyframe_ (keyframe),
    cfg_ (cfg),
    id_interval_(1),
    keyframe_vertex_(0),
    previous_vertex_(0),
    current_vertex_(0),
    vertex_id_(0),
    edge_id_(0),
    last_map(false)
    // keyframe_pose(cvo_keyframe_pose)
  {
    keyframe_pose = cvo_keyframe_pose;

    // g2o version used here is 20170730 (https://github.com/RainerKuemmerle/g2o/releases), can not use a newer version 
    // graph_.setAlgorithm(
    //     new g2o::OptimizationAlgorithmLevenberg(
    //         new BlockSolver(
    //             new LinearSolver()
    //         )
    //     )
    // );

    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph_.setAlgorithm(solver);
            
    graph_.setVerbose(false);

    keyframe_vertex_ = addFrameVertex(keyframe->timestamp);
    keyframe_vertex_->setFixed(true);
    keyframe_vertex_->setEstimateInv(internal::toSE3Quat(cvo_keyframe_pose));

  }

  g2o::VertexSE3ExpmapInv* addFrameVertex(const std::string& timestamp)
  {
    g2o::VertexSE3ExpmapInv* frame_vertex = new g2o::VertexSE3ExpmapInv();
    frame_vertex->setId(vertex_id_);
    frame_vertex->setUserData(new cvo_slam::Timestamped(timestamp));

    vertex_id_ += id_interval_;

    if(!graph_.addVertex(frame_vertex))
    {
      throw std::runtime_error("failed to add vertex to g2o graph!");
    }

    return frame_vertex;
  }

  g2o::EdgeSE3ExpmapInv* addTransformationEdge(g2o::VertexSE3ExpmapInv *from, g2o::VertexSE3ExpmapInv *to, const cvo_slam::tracking_result &result)
  {
    assert(from != 0 && to != 0);

    g2o::EdgeSE3ExpmapInv* edge = new g2o::EdgeSE3ExpmapInv();
    edge->setId(edge_id_);
    edge->resize(2);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    edge->setMeasurement(internal::toSE3Quat(result.transform));
    edge->setInformation(result.information);
    edge->setUserData(new cvo_slam::tracking_result(result));
    edge->setRobustKernel(createRobustKernel());

    graph_.addEdge(edge);

    edge_id_ += id_interval_;

    return edge;
  }

  g2o::RobustKernel* createRobustKernel()
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
};

} // namespace internal

LocalMap::Ptr LocalMap::create(KeyframePtr& keyframe, const Transform& cvo_keyframe_pose, const cvo_slam::cfg& cfg)
{
  LocalMap::Ptr result(new LocalMap(keyframe,cvo_keyframe_pose,cfg));
  return result;
}

LocalMap::LocalMap(KeyframePtr& keyframe, const Transform& cvo_keyframe_pose, const cvo_slam::cfg& cfg) :
    impl_(new internal::LocalMapImpl(keyframe, cvo_keyframe_pose, cfg))
{
}

LocalMap::~LocalMap()
{
}

KeyframePtr LocalMap::getKeyframe()
{
  return impl_->keyframe_;
}

Image_Ptr LocalMap::getCurrentFrame()
{
  return impl_->current_;
}

void LocalMap::getCurrentFramePose(Transform& current_pose)
{
  current_pose = getCurrentFramePose();
}

void LocalMap::setKeyframePose(const Transform& keyframe_pose)
{
  impl_->keyframe_vertex_->setEstimateInv(internal::toSE3Quat(keyframe_pose));

  g2o::OptimizableGraph::EdgeSet& edges = impl_->keyframe_vertex_->edges();

  for(g2o::OptimizableGraph::EdgeSet::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    g2o::EdgeSE3ExpmapInv *e = (g2o::EdgeSE3ExpmapInv*)(*it);

    assert(e->vertex(0) == impl_->keyframe_vertex_);

    g2o::VertexSE3ExpmapInv *v = (g2o::VertexSE3ExpmapInv*)e->vertex(1);
    v->setEstimateInv(impl_->keyframe_vertex_->estimateInv() * e->measurement());
  }
}

Transform LocalMap::getCurrentFramePose()
{
  // return internal::toAffine(impl_->current_vertex_->estimateInv());
  return impl_->current_pose;
}

g2o::SparseOptimizer& LocalMap::getGraph()
{
  return impl_->graph_;
}

void LocalMap::addFrame(const Image_Ptr& frame)
{
  impl_->current_ = frame;
  impl_->previous_vertex_ = impl_->current_vertex_;
  impl_->current_vertex_ = impl_->addFrameVertex(frame->timestamp);
}

void LocalMap::addOdometryMeasurement(const cvo_slam::tracking_result &result)
{
  impl_->addTransformationEdge(impl_->previous_vertex_, impl_->current_vertex_, result);
}

void LocalMap::addKeyframeMeasurement(const cvo_slam::tracking_result &result)
{
  impl_->addTransformationEdge(impl_->keyframe_vertex_, impl_->current_vertex_, result);
  impl_->current_vertex_->setEstimateInv(impl_->keyframe_vertex_->estimateInv() * internal::toSE3Quat(result.transform));
  impl_->current_pose = impl_->keyframe_pose * result.transform;
}

void LocalMap::optimize()
{
  impl_->graph_.initializeOptimization();
  impl_->graph_.computeInitialGuess();
  impl_->graph_.optimize(50);
}

void LocalMap::setLastMap()
{
  impl_->last_map = true;
}

bool LocalMap::lastMap()
{
  return impl_->last_map;
}

void LocalMap::setLastKeyframe(cvo_slam::KeyframePtr& keyframe)
{
  impl_->last_keyframe_ = keyframe;
}

KeyframePtr LocalMap::getLastKeyframe()
{
  return impl_->last_keyframe_;
}

int LocalMap::getFrameNumber()
{
  return impl_->graph_.vertices().size();
}

} //namespace cvo_slam
