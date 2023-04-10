/**
 * This is a modified version of local_map.h from dvo (see below).
 * Changes: 1) remove some original member functions and variables in class LocalMap; 
 *          2) claim new member functions and variables in LocalMap; 
 *          3) remove some original hearder files and add new hearder files; 
 *          4) change the namespace from dvo_slam to cvo_slam.
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

#ifndef LOCAL_MAP_H_
#define LOCAL_MAP_H_

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>
#include <g2o/core/robust_kernel_impl.h>

#include <boost/scoped_ptr.hpp>
#include "timestamped.h"
#include "tracking_result.h"

#include "cvo_image.h"

#include "keyframe.h"

#include "ORBVocabulary.h"

#include "vertex_and_edge.h"

namespace cvo_slam
{

typedef boost::shared_ptr<ORB_SLAM2::ORBVocabulary> OrbVocabularyPtr;

namespace internal
{
  struct LocalMapImpl;
} // namespace internal

class LocalMap
{
public:
  typedef boost::shared_ptr<LocalMap> Ptr;
  
  virtual ~LocalMap();

  // Creates a new local map with the given keyframe.
  static LocalMap::Ptr create(KeyframePtr& keyframe, const Transform& cvo_keyframe_pose, const cfg& cfg);

  KeyframePtr getKeyframe();

  void setKeyframePose(const Transform& keyframe_pose);

  Image_Ptr getCurrentFrame();

  void getCurrentFramePose(Transform& current_pose);
  Transform getCurrentFramePose();

  g2o::SparseOptimizer& getGraph();

  void addFrame(const Image_Ptr& frame);

  // Adds the transformation estimation between the last frame and the current frame to local map.
  void addOdometryMeasurement(const cvo_slam::tracking_result &result);

  // Adds the transformation estimation between the current keyframe and the current frame to local map.
  void addKeyframeMeasurement(const cvo_slam::tracking_result &result);

  void optimize();

  void setLastMap();

  bool lastMap();

  void setLastKeyframe(cvo_slam::KeyframePtr& keyframe);

  KeyframePtr getLastKeyframe();

  int getFrameNumber();
private:
  boost::scoped_ptr<internal::LocalMapImpl> impl_;
  LocalMap(KeyframePtr& keyframe, const Transform& cvo_keyframe_pose, const cfg& cfg);
};

} // namespace cvo_slam

#endif // LOCAL_MAP_H_
