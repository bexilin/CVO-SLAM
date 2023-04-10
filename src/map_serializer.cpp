/**
 * This is a modified version of map_serializer.h from dvo (see below).
 * Changes: remove definition of some subclasses of MapSerializerInterface.
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

#include "map_serializer.h"


namespace cvo_slam
{

MapSerializerInterface::MapSerializerInterface()
{
}

MapSerializerInterface::~MapSerializerInterface()
{
}

TrajectorySerializer::TrajectorySerializer(std::ostream& stream, std::ostream& stream_2) :
    stream_(stream), stream_2_(stream_2_)
{
}

TrajectorySerializer::~TrajectorySerializer()
{
}

void TrajectorySerializer::serialize(const KeyframeGraph::Ptr& map)
{
  KeyframeVector keyframes = map->keyframes();
  
  for (size_t i = 0; i < keyframes.size(); i++){
    KeyframePtr keyframe = keyframes[i];
    Eigen::Quaterniond q(keyframe->pose.rotation());
    stream_ << keyframe->timestamp << " " << keyframe->pose.translation()(0) << " " << keyframe->pose.translation()(1) << " " << keyframe->pose.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
    if(keyframe->frameLists.size()>0){
      FrameVector frames = keyframe->frameLists;

      for (size_t j = 0; j < frames.size(); j++){
        FramePtr frame = frames[j];
        Transform global_pose = keyframe->pose * frame->relative_pose;
        Eigen::Quaterniond q2(global_pose.rotation());
        stream_ << frame->timestamp << " " << global_pose.translation()(0) << " " << global_pose.translation()(1) << " " << global_pose.translation()(2) << " " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << " " << std::endl;
      }
    }
  }

  g2o::EdgeSE3ExpmapInv::InformationType edge_information;
    
  for (g2o::OptimizableGraph::EdgeSet::iterator iter = map->graph().edges().begin(); iter != map->graph().edges().end(); ++iter)
  {
    std::cout << "checking" << std::endl;
    if ((*iter)->id() % 2 != 0) continue;
    g2o::EdgeSE3ExpmapInv *it = (g2o::EdgeSE3ExpmapInv *)(*iter);

    g2o::VertexSE3ExpmapInv *temp_v1 = (g2o::VertexSE3ExpmapInv *)it->vertex(0);
    g2o::VertexSE3ExpmapInv *temp_v2 = (g2o::VertexSE3ExpmapInv *)it->vertex(1);

    std::cout << "get vertices" << std::endl;
    if (std::abs(temp_v1->id() - temp_v2->id()) == 2) continue;

    std::cout << temp_v1->id() << " " << temp_v2->id() << std::endl;
    std::cout << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << std::endl;
    stream_2_ << temp_v1->id() << " " << temp_v2->id() << " ";
    std::cout << "complete id printing" << std::endl;
    stream_2_ << ((cvo_slam::Timestamped *)((temp_v1)->userData()))->timestamp << " " << ((cvo_slam::Timestamped *)((temp_v2)->userData()))->timestamp << " ";

    g2o::Vector7d measurement;
    it->getMeasurementData(measurement);
    for (size_t i = 0; i < 7; ++i)
    {
      stream_2_ << measurement(i) << " ";
    }
    
    Information post = ((cvo_slam::tracking_result *)(it->userData()))->post_hessian;
    for (size_t i = 0; i < 6; ++i)
      for (size_t j = 0; j < 6; ++j)
        stream_2_ << post(i,j) << " ";

    stream_2_ << ((cvo_slam::tracking_result *)(it->userData()))->score << " ";
    stream_2_ << ((cvo_slam::tracking_result *)(it->userData()))->matches << " ";
    stream_2_ << ((cvo_slam::tracking_result *)(it->userData()))->inn_prior.value << " ";
    stream_2_ << ((cvo_slam::tracking_result *)(it->userData()))->inn_lc_prior.value << " ";
    stream_2_ << ((cvo_slam::tracking_result *)(it->userData()))->inn_post.value << std::endl;
  }
}

// void TrajectorySerializer::serialize(const KeyframeGraph::Ptr& map)
// {
//   std::map<ros::Time, Transform_Vertex> poses;

//   for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = map->graph().vertices().begin(); it != map->graph().vertices().end(); ++it)
//   {
//     if(it->second->id() % 3 == 2) continue;
    
//     g2o::VertexSE3ExpmapInv *v = (g2o::VertexSE3ExpmapInv *) it->second;

//     Timestamped *t = dynamic_cast<Timestamped *>(v->userData());

//     assert(t != 0);

//     poses[t->timestamp] = v->estimateInv();
//   }

//   for (std::map<ros::Time, Transform_Vertex>::iterator it = poses.begin(); it != poses.end(); ++it)
//   {
//     Eigen::Quaterniond q(it->second.rotation());

//     stream_ << it->first << " " << it->second.translation()(0) << " " << it->second.translation()(1) << " " << it->second.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
//   }
// }

}
