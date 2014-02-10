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

#ifndef PCL_GRAPH_COMMON_H
#define PCL_GRAPH_COMMON_H

#include <boost/ref.hpp>

#include "graph/pointcloud_adjacency_list.h"

namespace pcl
{

  namespace graph
  {

    /** \brief Compute normals and curvatures for all vertices in a graph.
      *
      * For each vertex the function finds its 1- or 2-ring neighbors and uses
      * pcl::computePointNormal() to calculate normal and curvature. It also
      * flips the calculated normal towards 0,0,0 viewpoint.
      *
      * \param[in] neighborhood_1ring flag which controls whether 1- or 2- ring
      * neighborhood is used. Default is 2-ring, which is slower, but produces
      * smoother normals.
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      */
    template <typename Graph> void
    computeNormalsAndCurvatures (Graph& graph, bool neighborhood_1ring = false);


    /** \brief Compute the type of curvature (concave/convex) for each vertex.
      *
      * The type of curvature is expressed through the sign. Convex curvature
      * is positive and concave curvature is negative. The absolute values of
      * curvatures are not altered by this function.
      *
      * TODO: add the formula.
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      */
    template <typename Graph> void
    computeSignedCurvatures (Graph& graph);


    /** \brief Find connected components in a graph and create a subgraph for
      * each of them.
      *
      * Each created subgraph is filled with the vertices that belong to the
      * corresponding connected component.
      *
      * In order to allow creation of subgraphs, the graph type should be an
      * instantiation of boost::subgraph template. Note that the graph is
      * passed by non-const reference, because subgraph creation modifies the
      * parent graph. Also, note that the created subgraphs are output as
      * references wrapped with boost::reference_wrapper. The reason is that
      * the factory function for subgraph creation in BGL returns newly created
      * subgraphs by reference.
      *
      * \param[in]  graph an input graph
      * \param[out] subgraphs a vector of references to created subgraps
      *
      * \return the number of connected components
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      * */
    template <typename Graph> size_t
    createSubgraphsFromConnectedComponents (Graph& graph,
                                            std::vector<boost::reference_wrapper<Graph> >& subgraphs);


    /** \brief Create two subgraphs of a given graph, one containing the points
      * with the given indices, and the other containing the remaining points.
      *
      * In order to allow creation of subgraphs, the graph type should be an
      * instantiation of boost::subgraph template. Note that the graph is
      * passed by non-const reference, because subgraph creation modifies the
      * parent graph. Also, note that the created subgraphs are output as
      * references wrapped with boost::reference_wrapper. The reason is that
      * the factory function for subgraph creation in BGL returns newly created
      * subgraphs by reference.
      *
      * \param[in]  graph an input graph
      * \param[in]  indices indices of points to be inserted in the first
      *             subgraph
      * \param[out] subgraphs a vector of references to created subgraps
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      * */
    template <typename Graph> void
    createSubgraphsFromIndices (Graph& graph,
                                const pcl::PointIndices& indices,
                                std::vector<boost::reference_wrapper<Graph> >& subgraphs);


    /** \brief Create subgraphs of a given graph containing vertices from a
      * given indices vector.
      *
      * For each set of indices in the \p indices vector this function will
      * create a subgraph containing corresponding vertices of the parent
      * graph. An additional subgraph containing all the remaining vertices
      * (not included in any other subgraph) will be created as well.
      *
      * In order to allow creation of subgraphs, the graph type should be an
      * instantiation of boost::subgraph template. Note that the graph is
      * passed by non-const reference, because subgraph creation modifies the
      * parent graph. Also, note that the created subgraphs are output as
      * references wrapped with boost::reference_wrapper. The reason is that
      * the factory function for subgraph creation in BGL returns newly created
      * subgraphs by reference.
      *
      * \param[in]  graph an input graph
      * \param[in]  indices a vector of indices of points to be inserted in the
      *             subgraphs
      * \param[out] subgraphs a vector of references to created subgraps
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      * */
    template <typename Graph> void
    createSubgraphsFromIndices (Graph& graph,
                                const std::vector<pcl::PointIndices>& indices,
                                std::vector<boost::reference_wrapper<Graph> >& subgraphs);


    /** \brief Apply bilateral filtering to a given point cloud graph.
      *
      * TODO: add detailed documentation and tests.
      *
      * \author Sergey Alexandrov
      * \ingroup graph
      * */
    template <typename Graph> void
    smoothen (Graph& graph, float spatial_sigma, float influence_sigma);

  }

}

#include "graph/impl/common.hpp"

#endif /* PCL_GRAPH_COMMON_H */

