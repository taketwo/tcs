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

#ifndef PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP
#define PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP

#include <boost/fusion/include/make_vector.hpp>
#include <boost/fusion/include/fold.hpp>

// TODO: do need to require that graph is actually a pointcloud adjacency list?
#include "graph/pointcloud_adjacency_list.h"

#include "graph/edge_weight_computer.h"

namespace pcl
{

  namespace traits
  {

    template <typename T> struct has_convexity : boost::mpl::and_<has_curvature<T>, has_normal<T> > { };

  }

  namespace detail
  {

#define CREATE_TERM(name, block)                                                                                \
    template <typename T, typename Enable = void>                                                               \
    struct BOOST_JOIN(name, _term)                                                                              \
    { float operator () (const T& p1, const T& p2) const { return 0.0; } };                                     \
                                                                                                                \
    template <typename T>                                                                                       \
    struct BOOST_JOIN(name, _term)<T, typename boost::enable_if<pcl::traits::BOOST_JOIN(has_, name)<T> >::type> \
    { float operator () (const T& p1, const T& p2) const { block } };

    CREATE_TERM(convexity,
               {
                 const Eigen::Vector3f   d = p2.getVector3fMap () - p1.getVector3fMap ();
                 const Eigen::Vector3f& n1 = p1.getNormalVector3fMap ();
                 const Eigen::Vector3f& n2 = p2.getNormalVector3fMap ();
                 return (((d - d.dot (n1) * n1).dot (n2) > 0 ? 1.0 : -1.0) * (n1 - n2).squaredNorm ());
               })

    CREATE_TERM(xyz,
               {
                 return (p2.getVector3fMap () - p1.getVector3fMap ()).squaredNorm ();
               })

    CREATE_TERM(normal,
               {
                 return (0.5 * (p1.getNormalVector3fMap () - p2.getNormalVector3fMap ()).squaredNorm ());
               })

    CREATE_TERM(curvature,
               {
                 return (std::fabs (p1.curvature) * std::fabs (p2.curvature));
               })

    CREATE_TERM(color,
               {
                 return ((p1.getRGBVector3i ().template cast<float> () - p2.getRGBVector3i ().template cast<float> ()).norm () / 255.0f);
               })

  }

}

template <class Graph, class EdgeWeightMap> void
pcl::graph::EdgeWeightComputer::compute (Graph& graph, EdgeWeightMap weights)
{
  typedef typename boost::vertex_point_type<Graph>::type PointT;
  typename boost::graph_traits<Graph>::edge_iterator ei, ee;

  detail::convexity_term <PointT> convexity;
  detail::xyz_term       <PointT> xyz;
  detail::normal_term    <PointT> normal;
  detail::curvature_term <PointT> curvature;
  detail::color_term     <PointT> color;

  // Step 1: compute convexity for each vertex. This may be omitted if the user
  // set convex_discount_ to 1.0, i.e. it is not needed, or if the point type
  // does not have relevant fields.
  std::vector<float> convexities (boost::num_vertices (graph), 0.0f);
  if (convex_discount_ != 1.0f && pcl::traits::has_convexity<PointT>::value)
  {
    for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
    {
      typename boost::graph_traits<Graph>::vertex_descriptor v1, v2;
      v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
      float c = convexity (graph[v1], graph[v2]);
      convexities[v1] += c;
      convexities[v2] += c;
    }
  }

  // Step 2: compute weight for each edge.
  struct { float operator () (float v, float w) const { return (w > 0.0f ? std::exp (-v / w) : 1.0f); } } gaussian;
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
  {
    typename boost::graph_traits<Graph>::vertex_descriptor v1, v2;
    v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
    const PointT& p1 = graph[v1];
    const PointT& p2 = graph[v2];

    float convex_discount = (convexities[v1] > 0.0f || convexities[v2] > 0.0f) ? convex_discount_ : 1.0f;

    weights[*ei] = gaussian (xyz       (p1, p2)                  , distance_weight_ ) *
                   gaussian (normal    (p1, p2) * convex_discount, normal_weight_   ) *
                   gaussian (curvature (p1, p2) * convex_discount, curvature_weight_) *
                   gaussian (color     (p1, p2)                  , color_weight_    ) ;
  }

  // Step 3: find edges with very small weight and modify them according to the
  // policy set by the user.
  switch (policy_)
  {
    case SMALL_WEIGHT_DO_NOTHING:
      {
        break;
      }
    case SMALL_WEIGHT_COERCE_TO_THRESHOLD:
      {
        typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIterator;
        EdgeIterator ei, ee;
        for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
          if (weights[*ei] < threshold_)
            weights[*ei] = threshold_;
        break;
      }
    case SMALL_WEIGHT_REMOVE_EDGE:
      {
        typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeId;
        // It is only okay to remove edges this way before any subgraph was
        // created.
        // TODO: move this note in an appropriate place.
        // TODO: get rid of lambda
        // TODO: make work with non-subgraph
        boost::remove_edge_if ([&](EdgeId edge) { return weights[edge] < threshold_; }, graph.m_graph);
        break;
      }
  }
}

#endif /* PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP */

