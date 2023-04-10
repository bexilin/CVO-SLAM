/**
 * This is a modified version of map_serializer.h from dvo (see below).
 * Changes: 1) remove some subclasses of MapSerializerInterface; 
 *          2) remove some original hearder files and add new hearder files; 
 *          3) change the namespace from dvo_slam to cvo_slam.
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

#ifndef MAP_SERIALIZER_H_
#define MAP_SERIALIZER_H_

#include <iostream>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include "timestamped.h"
#include "cvo_image.h"
#include "keyframe_graph.h"

namespace cvo_slam
{

class MapSerializerInterface
{
public:
  MapSerializerInterface();
  virtual ~MapSerializerInterface();

  virtual void serialize(const KeyframeGraph::Ptr& map) = 0;
};

template<typename DelegateType>
class FileSerializer : public MapSerializerInterface
{
public:
  FileSerializer(const std::string& filename, const std::string& filename_2) :
    fstream_(filename.c_str()),
    fstream_2_(filename_2.c_str()),
    delegate_(fstream_,fstream_2_)
  {
    std::cerr << filename << std::endl;
    std::cerr << filename_2 << std::endl;
  }
  virtual ~FileSerializer()
  {
    fstream_.flush();
    fstream_.close();
    fstream_2_.flush();
    fstream_2_.close();
  }

  virtual void serialize(const KeyframeGraph::Ptr& map)
  {
    delegate_.serialize(map);
  }
private:
  std::ofstream fstream_;
  std::ofstream fstream_2_;
  DelegateType delegate_;
};

class TrajectorySerializer : public MapSerializerInterface
{
public:
  TrajectorySerializer(std::ostream& stream, std::ostream& stream_2);
  virtual ~TrajectorySerializer();

  virtual void serialize(const KeyframeGraph::Ptr& map);
private:
  std::ostream& stream_;
  std::ostream& stream_2_;
};

} // namespace cvo_slam

#endif // MAP_SERIALIZER_H_
