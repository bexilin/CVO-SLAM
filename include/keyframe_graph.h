/**
 * This is a modified version of keyframe_graph.h from dvo (see below).
 * Changes: 1) remove some original member functions and variables in class KeyframeGraph; 
 *          2) claim new member functions and variables in KeyframeGraph; 
 *          3) remove some original hearder files and add new hearder files; 
 *          4) change the namespace from dvo_slam to cvo_slam.
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

#ifndef KEYFRAME_GRAPH_H_
#define KEYFRAME_GRAPH_H_

#include "local_map.h"
#include "keyframe.h"

#include <boost/function.hpp>

#include "local_tracker.h"
#include "timestamped.h"
#include "cfg.h"

#include <tbb/concurrent_queue.h>
#include <tbb/tbb_thread.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/mutex.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/core/estimate_propagator.h>

// #include <g2o/types/slam3d/vertex_se3.h>
// #include <g2o/types/slam3d/edge_se3.h>
// #include <g2o/types/slam3d/edge_se3_offset.h>

#include "ORBmatcher.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/transformation_estimation_svd.h>

#include <utility>
#include <queue>
#include <set>

// #include <boost/filesystem.hpp>

namespace cvo_slam
{

typedef boost::shared_ptr<ORB_SLAM2::ORBmatcher> OrbMatcherPtr;
namespace internal
{

class KeyframeGraphImpl;

typedef boost::scoped_ptr<KeyframeGraphImpl> KeyframeGraphImplPtr;

} // namespace internal

class KeyframeGraph
{
public:
  typedef void MapChangedCallbackSignature(KeyframeGraph&);
  typedef boost::function<MapChangedCallbackSignature> MapChangedCallback;
  typedef boost::shared_ptr<KeyframeGraph> Ptr;

  KeyframeGraph(OrbVocabularyPtr& OrbVoc_, const std::string& folder, const std::string& calib_file);
  virtual ~KeyframeGraph();

  void configure(const cvo_slam::cfg& config);

  void add(const LocalMap::Ptr& keyframe);

  // Perform global pose graph optimization when SLAM process ends
  // void finalOptimization();

  void addMapChangedCallback(const KeyframeGraph::MapChangedCallback& callback);

  const KeyframeVector& keyframes() const;

  const g2o::SparseOptimizer& graph() const;

private:
  internal::KeyframeGraphImplPtr impl_;
};

} // namespace cvo_slam

#endif // KEYFRAME_GRAPH_H_
