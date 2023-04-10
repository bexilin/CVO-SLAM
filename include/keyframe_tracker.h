/**
 * This is a modified version of keyframe_tracker.h from dvo (see below).
 * Changes: 1) remove some original member functions and variables in class KeyframeTracker; 
 *          2) claim new member functions and variables in KeyframeTracker; 
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

#ifndef KEYFRAME_TRACKER_H_
#define KEYFRAME_TRACKER_H_

#include "local_tracker.h"
#include "keyframe_graph.h"
#include "cfg.h"
// #include "map_serializer.h"
#include <math.h>

namespace cvo_slam
{

class KeyframeTracker
{
public:

  KeyframeTracker(const string& strVocFile, const string& strSettingsFile, const string& folder);
  ~KeyframeTracker();

  void configure(const cfg& cfg);
  
  void configureGraph(const cfg& cfg);

  void configureTracker(const cfg& cfg);

  void init();
  void init(const Transform& initial_transformation);

  void update(const Image_Ptr& current, Transform& absolute_transformation);

  bool checkNewMap();

  void forceKeyframe();

  // void finish();

  // void serializeMap(MapSerializerInterface& serializer);

  void writeSLAMTrajectoryAndLoopClosureFile(string& SLAM_file_, string& lc_file_);

private:
  class Impl;
  boost::shared_ptr<Impl> impl_;
};

} // namespace cvo_slam

#endif // KEYFRAME_TRACKER_H_
