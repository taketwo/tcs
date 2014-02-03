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

#ifndef PCL_GRAPH_IMPL_COMMON_HPP
#define PCL_GRAPH_IMPL_COMMON_HPP

#include <boost/graph/connected_components.hpp>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include "graph/common.h"
#include "graph/pointcloud_adjacency_list.h"

template <typename Graph> void
pcl::graph::computeNormalsAndCurvatures (Graph& graph)
{
  typedef typename boost::vertex_point_type<Graph>::type PointT;
  typedef typename Graph::adjacency_iterator AdjacencyIterator;
  typedef typename Graph::vertex_descriptor VertexId;

  typename pcl::PointCloud<PointT>::ConstPtr cloud = boost::get_pointcloud (graph);

  for (VertexId vertex = 0; vertex < boost::num_vertices (graph); ++vertex)
  {
    // Determine 2-ring neighborhood
    std::vector<int> neighbors (1, vertex);
    neighbors.reserve (256);
    AdjacencyIterator vi1, ve1, vi2, ve2;
    for (boost::tie (vi1, ve1) = boost::adjacent_vertices (vertex, graph); vi1 != ve1; ++vi1)
    {
      neighbors.push_back (*vi1);
      for (boost::tie (vi2, ve2) = boost::adjacent_vertices (*vi1, graph); vi2 != ve2; ++vi2)
        neighbors.push_back (*vi2);
    }

    Eigen::Vector4f normal;
    float curvature;
    pcl::computePointNormal (*cloud, neighbors, normal, curvature);

    pcl::flipNormalTowardsViewpoint (graph[vertex], 0.0f, 0.0f, 0.0f, normal);
    normal[3] = 0;
    normal.normalize ();
    graph[vertex].getNormalVector4fMap () = normal;
    graph[vertex].curvature = std::isnan (curvature) ? 0.0f : curvature;
  }
}

template <typename Graph> void
pcl::graph::computeSignedCurvatures (Graph& graph)
{
  typedef typename boost::vertex_point_type<Graph>::type PointT;
  typedef typename Graph::edge_iterator EdgeIterator;
  typedef typename Graph::vertex_descriptor VertexId;

  EdgeIterator ei, ee;
  std::vector<float> convexities (boost::num_vertices (graph), 0.0f);
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
  {
    VertexId v1 = boost::source (*ei, graph);
    VertexId v2 = boost::target (*ei, graph);
    const PointT& p1 = graph[v1];
    const PointT& p2 = graph[v2];
    const Eigen::Vector3f&  d = p2.getVector3fMap () - p1.getVector3fMap ();
    const Eigen::Vector3f& n1 = p1.getNormalVector3fMap ();
    const Eigen::Vector3f& n2 = p2.getNormalVector3fMap ();
    float c = (((d - d.dot (n1) * n1).dot (n2) > 0 ? 1.0 : -1.0) * (n1 - n2).squaredNorm ());
    convexities[v1] += c;
    convexities[v2] += c;
  }

  for (VertexId vertex = 0; vertex < boost::num_vertices (graph); ++vertex)
    graph[vertex].curvature = std::copysign (graph[vertex].curvature, convexities[vertex]);
}

template <typename Graph> size_t
pcl::graph::createSubgraphsFromConnectedComponents (Graph& graph, std::vector<boost::reference_wrapper<Graph> >& subgraphs)
{
  typedef typename Graph::vertex_descriptor VertexId;
  typedef typename boost::reference_wrapper<Graph> GraphRef;

  subgraphs.clear ();
  std::vector<int> component (boost::num_vertices (graph));
  size_t num_components = boost::connected_components (graph, &component[0]);
  for (size_t i = 0; i < num_components; ++i)
    subgraphs.push_back (GraphRef (graph.create_subgraph ()));
  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
    boost::add_vertex (v, subgraphs.at (component[v]).get ());
  return num_components;
}

#endif /* PCL_GRAPH_IMPL_COMMON_HPP */

