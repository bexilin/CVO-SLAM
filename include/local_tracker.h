/**
 * This is a modified version of local_tracker.h from dvo (see below).
 * Changes: 1) remove some original member functions and variables in class LocalTracker; 
 *          2) claim new member functions and variables in LocalTracker; 
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

#ifndef LOCAL_TRACKER_H_
#define LOCAL_TRACKER_H_

#include "local_map.h"

#include <boost/signals2.hpp>

#include "cvo.hpp"
#include "tracking_result.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBextractor.h"

#include "Converter.h"

// #include <Python.h>

namespace cvo_slam
{

typedef unique_ptr<cvo::cvo> CvoPtr;

namespace internal
{
  struct LocalTrackerImpl;
} // namespace internal

class LocalTracker
{
public:
  typedef boost::shared_ptr<LocalTracker> Ptr;

  struct All
  {
    typedef bool result_type;

    template<typename InputIterator>
    bool operator()(InputIterator first, InputIterator last) const
    {
      int i = 0;
      bool result = true;
      for(;first != last; ++first)
      {
        bool tmp = *first;
        result = result && tmp;
        i++;
      }

      return result;
    }
  };

  typedef boost::signals2::signal<bool (const LocalTracker&, const tracking_result&, const tracking_result&), LocalTracker::All> AcceptSignal;
  typedef AcceptSignal::slot_type AcceptCallback;
  typedef boost::signals2::signal<void (const LocalTracker&, const LocalMap::Ptr&, const tracking_result&)> MapInitializedSignal;
  typedef MapInitializedSignal::slot_type MapInitializedCallback;
  typedef boost::signals2::signal<void (const LocalTracker&, const LocalMap::Ptr&)> MapCompleteSignal;
  typedef MapCompleteSignal::slot_type MapCompleteCallback;

  LocalTracker(OrbVocabularyPtr& OrbVoc, const string& strSettingsFile);
  virtual ~LocalTracker();

  cvo_slam::LocalMap::Ptr getLocalMap() const;

  void getCurrentPose(Transform& pose);

  void initNewLocalMap(const Image_Ptr& keyframe, const Image_Ptr& frame, const Transform& keyframe_pose = Transform::Identity());

  void update(const Image_Ptr& image, Transform& pose);

  bool checkNewMap();

  void forceCompleteCurrentLocalMap();

  void configure(const cfg& config);

  boost::signals2::connection addAcceptCallback(const AcceptCallback& callback);
  boost::signals2::connection addMapInitializedCallback(const MapInitializedCallback& callback);
  boost::signals2::connection addMapCompleteCallback(const MapCompleteCallback& callback);

  // std::ofstream pcd_num_file;
  // std::ofstream pcd_num_file_kf;
  std::ofstream cos_angle_file;
  std::ofstream cos_angle_file_kf;

private:
  boost::scoped_ptr<internal::LocalTrackerImpl> impl_;
  LocalMap::Ptr local_map_;
  void initNewLocalMap(const Image_Ptr& keyframe, const Image_Ptr& frame, const tracking_result& r_odometry, const Transform& keyframe_pose, std::vector<cv::Point2f>& selected_points);
};

} // namespace cvo_slam

#endif // LOCAL_TRACKER_H_
