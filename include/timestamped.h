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

#ifndef TIMESTAMPED_H_
#define TIMESTAMPED_H_

// #include <ros/time.h>
#include <iostream>
#include <g2o/core/optimizable_graph.h>

namespace cvo_slam
{

class Timestamped : public g2o::OptimizableGraph::Data
{
public:
  Timestamped()
  {
  }

  Timestamped(const std::string& new_timestamp) :
    timestamp(new_timestamp)
  {
    // assert(!timestamp.isZero());
  }

  // Read the data from a stream
  virtual bool read(std::istream& is)
  {
    // double sec;
    // is >> sec;
    // timestamp.fromSec(sec);
    is >> timestamp;

    return true;
  }

  // Write the data to a stream
  virtual bool write(std::ostream& os) const
  {
    os << timestamp;
    return true;
  }

  std::string timestamp;
};

} // namespace cvo_slam

#endif // TIMESTAMPED_H_
