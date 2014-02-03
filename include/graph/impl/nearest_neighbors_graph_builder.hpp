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

#ifndef PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP
#define PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include "graph/nearest_neighbors_graph_builder.h"

namespace pcl
{

// TODO: remove as soon as my PR gets merged
template <typename PointInT, typename PointOutT> void
copyPointCloud2 (const pcl::PointCloud<PointInT> &cloud_in, 
                 const std::vector<int> &indices,
                 pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = uint32_t (indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Copy all the data fields from the input cloud to the output one
  typedef typename pcl::traits::fieldList<PointInT>::type FieldListInT;
  typedef typename pcl::traits::fieldList<PointOutT>::type FieldListOutT;
  typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList; 

  // If the point types are the same, don't copy one by one
  if (isSamePointType<PointInT, PointOutT> ())
  {
    // Iterate over each point
    for (size_t i = 0; i < indices.size (); ++i)
      memcpy (&cloud_out.points[i], &cloud_in.points[indices[i]], sizeof (PointInT));
    return;
  }

  std::vector<pcl::PCLPointField> fields_in, fields_out;
  pcl::for_each_type<FieldListInT> (pcl::detail::FieldAdder<PointInT> (fields_in));
  pcl::for_each_type<FieldListOutT> (pcl::detail::FieldAdder<PointOutT> (fields_out));

  // RGB vs RGBA is an official missmatch until PCL 2.0, so we need to search for it and 
  // fix it manually
  int rgb_idx_in = -1, rgb_idx_out = -1;
  for (size_t i = 0; i < fields_in.size (); ++i)
    if (fields_in[i].name == "rgb" || fields_in[i].name == "rgba")
    {
      rgb_idx_in = int (i);
      break;
    }
  for (size_t i = 0; i < fields_out.size (); ++i)
    if (fields_out[i].name == "rgb" || fields_out[i].name == "rgba")
    {
      rgb_idx_out = int (i);
      break;
    }

  // We have one of the two cases: RGB vs RGBA or RGBA vs RGB
  if (rgb_idx_in != -1 && rgb_idx_out != -1 && 
      fields_in[rgb_idx_in].name != fields_out[rgb_idx_out].name)
  {
    size_t field_size_in  = getFieldSize (fields_in[rgb_idx_in].datatype),
           field_size_out = getFieldSize (fields_out[rgb_idx_out].datatype);

    if (field_size_in == field_size_out)
    {
      for (size_t i = 0; i < indices.size (); ++i)
      {
        // Copy the rest
        pcl::for_each_type<FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (cloud_in.points[indices[i]], cloud_out.points[i]));
        // Copy RGB<->RGBA
        memcpy (reinterpret_cast<char*> (&cloud_out.points[i]) + fields_out[rgb_idx_out].offset, reinterpret_cast<const char*> (&cloud_in.points[indices[i]]) + fields_in[rgb_idx_in].offset, field_size_in);
      }
      return;
    }
  }

  // Iterate over each point if no RGB/RGBA or if their size is different
  for (size_t i = 0; i < indices.size (); ++i)
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (cloud_in.points[indices[i]], cloud_out.points[i]));
}

}

template <typename PointT, typename Graph> void
pcl::graph::NearestNeighborsGraphBuilder<PointT, Graph>::compute (Graph& graph)
{
  if (!initCompute ())
  {
    graph = Graph ();
    deinitCompute ();
    return;
  }

  size_t k = 0;
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    if (!pcl::isFinite (input_->operator[] (indices_->operator[] (i))))
      fake_indices_ = false;
    else
      indices_->operator[] (k++) = indices_->operator[] (i);
  }
  indices_->resize (k);

  // Create a new point cloud which will be the basis for the constructed graph.
  // All the fields that are also present in the output point type will be
  // copied over from the original point cloud.
  typename pcl::PointCloud<PointOutT>::Ptr cloud (new pcl::PointCloud<PointOutT>);
  pcl::copyPointCloud2 (*input_, *indices_, *cloud);
  graph = Graph (cloud);

  // In case a search method has not been given, initialize it using defaults
  if (!search_)
  {
    // For organized datasets, use an OrganizedDataIndex
    if (cloud->isOrganized ())
      search_.reset (new pcl::search::OrganizedNeighbor<PointOutT>);
    // For unorganized data, use a KdTree
    else
      search_.reset (new pcl::search::KdTree<PointOutT>);
  }

  // Establish edges with nearest neighbors.
  std::vector<int> neighbors (num_neighbors_);
  std::vector<float> distances (num_neighbors_);
  search_->setInputCloud (cloud);
  for (size_t i = 0; i < cloud->size (); ++i)
  {
    // Search for num_neighbors_ + 1 because the first neighbor output by KdTree
    // is always the query point itself.
    search_->nearestKSearch (i, num_neighbors_ + 1, neighbors, distances);
    for (size_t j = 1; j < neighbors.size (); ++j)
      if (!boost::edge (i, neighbors[j], graph).second)
        boost::add_edge (i, neighbors[j], graph);
  }
}

template <typename PointT, typename Graph> void
pcl::graph::NearestNeighborsGraphBuilder<PointT, Graph>::getPointToVertexMap (std::vector<VertexId>& indices)
{
  indices.clear ();
  indices.resize (input_->size (), std::numeric_limits<VertexId>::max ());
  VertexId v = 0;
  for (size_t i = 0; i < indices_->size (); ++i)
    indices[indices_->operator[] (i)] = v++;
}

#endif /* PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP */

