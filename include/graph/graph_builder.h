/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_GRAPH_GRAPH_BUILDER_H
#define PCL_GRAPH_GRAPH_BUILDER_H

#include <boost/mpl/contains.hpp>
#include <boost/ref.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "graph/pointcloud_adjacency_list.h"

namespace pcl
{

  namespace graph
  {

    /** \brief This is an abstract base class for building a BGL-compatible
      * graph from a point cloud.
      *
      * Building a graph involves creating vertices and establishing edges
      * between them. (A particular algorithm for doing these depends on the
      * extending class.)
      *
      * The two template parameters are the type of points in input cloud and
      * the output graph type. The graph type should be either
      * boost::pointcloud_adjacency_list or boost::subgraph wrapped around the
      * former. The data contained in the points of the input cloud will be
      * copied inside the vertices of the newly created graph. Note that the
      * points in the input cloud and the output graph may have different types,
      * in which case only the intersecting fields will be copied over.
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      */
    template <typename PointT, typename Graph>
    class PCL_EXPORTS GraphBuilder : public pcl::PCLBase<PointT>
    {

      public:

        typedef boost::shared_ptr<GraphBuilder<PointT, Graph> > Ptr;

        /// Type of points in the input cloud.
        typedef PointT PointInT;
        /// Type of points in the output graph.
        typedef typename boost::vertex_point_type<Graph>::type PointOutT;

        typedef typename Graph::vertex_descriptor VertexId;

        /** Build a graph based on the provided input data. */
        virtual void
        compute (Graph& graph) = 0;

        /** \brief Get a mapping between points in the input cloud and the
          * vertices in the output graph.
          *
          * \warning Some points may have no corresponding vertex. This happens
          * e.g. for NaN points, or if the user intentionally excluded some
          * points from graph building process by providing indices vector with
          * setIndices(). For such points the "nil" vertex id will be assigned,
          * which is equal to std::numeric_limits<VertexId>::max (). */
        virtual void
        getPointToVertexMap (std::vector<VertexId>& vertex_ids) = 0;

    };

  }

}

#endif /* PCL_GRAPH_GRAPH_BUILDER_H */

