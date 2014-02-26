/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_GRAPH_OCTREE_ADJACENCY_GRAPH_BUILDER_H
#define PCL_GRAPH_OCTREE_ADJACENCY_GRAPH_BUILDER_H

//#include <pcl/octree/octree_pointcloud_adjacency.h>
#include "octree_pointcloud_adjacency3.h"

#include "graph/graph_builder.h"

namespace pcl
{

  namespace graph
  {

    /** This class builds a BGL graph representing an input dataset by using
      * octree::OctreePointCloudAdjacency.
      *
      * For additional information see documentation for \ref GraphBuilder.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <typename PointT, typename Graph>
    class PCL_EXPORTS OctreeAdjacencyGraphBuilder : public GraphBuilder<PointT, Graph>
    {

        using PCLBase<PointT>::initCompute;
        using PCLBase<PointT>::deinitCompute;
        using PCLBase<PointT>::indices_;
        using PCLBase<PointT>::input_;

      public:

        using typename GraphBuilder<PointT, Graph>::PointInT;
        using typename GraphBuilder<PointT, Graph>::PointOutT;
        using typename GraphBuilder<PointT, Graph>::VertexId;

        typedef pcl::octree::OctreePointCloudAdjacencyContainer<PointT> OctreeAdjacencyContainer;
        typedef pcl::octree::OctreePointCloudAdjacency<PointT, OctreeAdjacencyContainer> OctreeAdjacency;
        typedef typename OctreeAdjacency::Ptr OctreeAdjacencyPtr;

        /** Constructor.
          *
          * \param[in] voxel_resolution resolution of the adjacency octree
          * \param[in] with_transform_function controls whether the octree
          * should use a point transform function (see \ref
          * octree::OctreePointCloudAdjacency::setTransformFunction) */
        OctreeAdjacencyGraphBuilder (float voxel_resolution, bool with_transform_function = false)
        : voxel_resolution_ (voxel_resolution)
        , with_transform_function_ (with_transform_function)
        {
        }

        virtual void
        compute (Graph& graph);

        virtual void
        getPointToVertexMap (std::vector<VertexId>& indices);

        inline void
        setOctreeAdjacency (const OctreeAdjacencyPtr& octree)
        {
          octree_adjacency_ = octree;
        }

        inline void
        setVoxelResolution (float resolution)
        {
          voxel_resolution_ = resolution;
        }

        inline float
        getVoxelResolution () const
        {
          return voxel_resolution_;
        }

      private:

        static void
        transformFunction (PointT& p);

        /// Adjacency octree that will be used to voxelize the input dataset.
        OctreeAdjacencyPtr octree_adjacency_;

        /// Voxel resolution for adjacency octree
        float voxel_resolution_;

        bool with_transform_function_;

        std::map<typename OctreeAdjacency::LeafVectorT::value_type, VertexId> leaf_vertex_map_;

    };

  }

}

#include "graph/impl/octree_adjacency_graph_builder.hpp"

#endif /* PCL_GRAPH_OCTREE_ADJACENCY_GRAPH_BUILDER_H */

