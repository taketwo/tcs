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

#ifndef PCL_GRAPH_IMPL_OCTREE_ADJACENCY_GRAPH_BUILDER_HPP
#define PCL_GRAPH_IMPL_OCTREE_ADJACENCY_GRAPH_BUILDER_HPP

#include <boost/bind.hpp>

#include "graph/octree_adjacency_graph_builder.h"

// TODO: replace with a library copyPoint() function as soon as it emerges
#include "copy_point.h"

template <typename PointT, typename Graph> void
pcl::graph::OctreeAdjacencyGraphBuilder<PointT, Graph>::compute (Graph& graph)
{
  if (!initCompute ())
  {
    graph = Graph ();
    deinitCompute ();
    return;
  }

  if (!octree_adjacency_)
  {
    octree_adjacency_.reset (new OctreeAdjacency (voxel_resolution_));
    if (with_transform_function_)
      octree_adjacency_->setTransformFunction (boost::bind (&OctreeAdjacencyGraphBuilder<PointT, Graph>::transformFunction, _1));
  }

  octree_adjacency_->setInputCloud (input_, indices_);
  octree_adjacency_->addPointsFromInputCloud ();

  graph = Graph (boost::make_shared<pcl::PointCloud<PointOutT> > (octree_adjacency_->getLeafCount (), 1));

  leaf_vertex_map_.clear ();
  typename OctreeAdjacency::iterator leaf_itr = octree_adjacency_->begin ();
  for (VertexId v = 0; leaf_itr != octree_adjacency_->end (); ++leaf_itr, ++v)
  {
    leaf_vertex_map_[*leaf_itr] = v;
    copyPoint<PointT, PointOutT> ((*leaf_itr)->getData (), graph[v]);
  }

  for (leaf_itr = octree_adjacency_->begin (); leaf_itr != octree_adjacency_->end (); ++leaf_itr)
  {
    const VertexId idx1 = leaf_vertex_map_[*leaf_itr];
    typename OctreeAdjacencyContainer::iterator neighb_itr;
    for (neighb_itr = (*leaf_itr)->begin (); neighb_itr != (*leaf_itr)->end (); ++neighb_itr)
    {
      const VertexId idx2 = leaf_vertex_map_[*neighb_itr];
      if (idx1 > idx2) // this prevents from adding self-edges and duplicates
        boost::add_edge (idx1, idx2, graph);
    }
  }
}

template <typename PointT, typename Graph> void
pcl::graph::OctreeAdjacencyGraphBuilder<PointT, Graph>::getPointToVertexMap (std::vector<VertexId>& indices)
{
  indices.clear ();
  indices.resize (input_->size (), std::numeric_limits<VertexId>::max ());
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    const PointT& pt = input_->operator[] (indices_->operator[] (i));
    if (pcl::isFinite (pt))
    {
      OctreeAdjacencyContainer* leaf = octree_adjacency_->getLeafContainerAtPoint (pt);
      indices[indices_->operator[] (i)] = leaf_vertex_map_[leaf];
    }
  }
}

template <typename PointT, typename Graph> void
pcl::graph::OctreeAdjacencyGraphBuilder<PointT, Graph>::transformFunction (PointT& p)
{
  p.x /= p.z;
  p.y /= p.z;
  p.z = std::log (p.z);
}

#endif /* PCL_GRAPH_IMPL_OCTREE_ADJACENCY_GRAPH_BUILDER_HPP */

