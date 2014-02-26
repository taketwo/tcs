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

#include <gtest/gtest.h>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_tests.h>

#include "graph/point_cloud_graph.h"

#include "graph/graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"
#include "graph/octree_adjacency_graph_builder.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;
typedef typename PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;

using namespace pcl::graph;

class GraphBuilderTest : public ::testing::Test
{

  public:

    template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
    createGrid (size_t m, size_t n, float step, size_t points_per_cell = 1, float random_z = 0.0f)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> (m * n * points_per_cell, 1));
      typename pcl::PointCloud<PointT>::iterator iter = cloud->points.begin ();
      for (size_t i = 0; i < m; ++i)
      {
        float x = i * step;
        for (size_t j = 0; j < n; ++j)
        {
          float y = j * step;
          for (size_t k = 0; k < points_per_cell; ++k, ++iter)
          {
            iter->x = x;
            iter->y = y;
            iter->z = random_z * step * rand () / RAND_MAX;
          }
        }
      }
      return cloud;
    }

    template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
    copyAndAddRandomPoints (const pcl::PointCloud<PointT>& cloud_in,
                            size_t num_of_points_to_add,
                            std::vector<int>& indices_copied)
    {
      int target_size = cloud_in.size () + num_of_points_to_add;
      typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> (target_size, 1));
      std::set<int> positions;
      while (positions.size () < num_of_points_to_add)
        positions.insert ((rand () % target_size));
      for (int i = 0; i < target_size; ++i)
      {
        if (positions.count (i))
        {
          cloud->at (i).getVector3fMap () << rand (), rand (), rand ();
        }
        else
        {
          cloud->at (i) = cloud_in.at (indices_copied.size ());
          indices_copied.push_back (i);
        }
      }
      return cloud;
    }

};

class NearestNeighborsGraphBuilderTest : public GraphBuilderTest
{

  public:

    NearestNeighborsGraphBuilderTest ()
    : grid2x3 (createGrid<pcl::PointXYZ> (2, 3, 1.0, 1))
    , grid2x3_indices (new std::vector<int>)
    , grid2x3_with_extra_points (copyAndAddRandomPoints<pcl::PointXYZ> (*grid2x3, 6, *grid2x3_indices))
    , grid2x3_with_color (createGrid<pcl::PointXYZRGBA> (2, 3, 1.0, 1))
    , grid3x3 (createGrid<pcl::PointXYZ> (3, 3, 0.1, 1, 0.1))
    {
      for (size_t i = 0; i < grid2x3_with_color->size (); ++i)
        grid2x3_with_color->at (i).rgba = rand ();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr grid2x3;
    pcl::IndicesPtr grid2x3_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid2x3_with_extra_points;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr grid2x3_with_color;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid3x3;

};

TEST_F (NearestNeighborsGraphBuilderTest, Constructor)
{
  const size_t K = 10;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
  ASSERT_EQ (K, gb.getNumberOfNeighbors ());
}

TEST_F (NearestNeighborsGraphBuilderTest, NumberOfNeighbors)
{
  const size_t K = 10;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb;
  gb.setNumberOfNeighbors (K);
  ASSERT_EQ (K, gb.getNumberOfNeighbors ());
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeWithoutCloud)
{
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb;
  Graph graph (10);
  gb.compute (graph);
  EXPECT_EQ (0, boost::num_vertices (graph));
  EXPECT_EQ (0, boost::num_edges (graph));
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeSamePointTypeWithoutIndices)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
  gb.setInputCloud (grid2x3);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (11, boost::num_edges (graph));
  ASSERT_EQ (6, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
    EXPECT_XYZ_EQ (grid2x3->at (i), graph[i]);
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeSamePointTypeWithIndices)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
  gb.setInputCloud (grid2x3_with_extra_points);
  gb.setIndices (grid2x3_indices);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (11, boost::num_edges (graph));
  ASSERT_EQ (6, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < grid2x3_indices->size (); ++i)
    EXPECT_XYZ_EQ (grid2x3_with_extra_points->at (grid2x3_indices->at (i)), graph[i]);
}

TEST_F (NearestNeighborsGraphBuilderTest, GetPointToVertexMap)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  Graph graph;
  std::vector<Graph::vertex_descriptor> indices;
  // Without indices
  {
    pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
    gb.setInputCloud (grid2x3_with_extra_points);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    EXPECT_EQ (grid2x3_with_extra_points->size (), boost::num_vertices (graph));
    EXPECT_EQ (grid2x3_with_extra_points->size (), indices.size ());
    for (size_t i = 0; i < indices.size (); ++i)
      EXPECT_EQ (i, indices[i]);
  }
  // With indices
  {
    pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
    gb.setInputCloud (grid2x3_with_extra_points);
    gb.setIndices (grid2x3_indices);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    EXPECT_EQ (grid2x3_indices->size (), boost::num_vertices (graph));
    EXPECT_EQ (grid2x3_with_extra_points->size (), indices.size ());
    for (size_t i = 0; i < grid2x3_indices->size (); ++i)
      EXPECT_NE (std::numeric_limits<Graph::vertex_descriptor>::max (), indices[grid2x3_indices->at (i)]);
  }
  // With NaN
  {
    pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
    grid2x3->push_back (pcl::PointXYZ (0, NAN, 0));
    gb.setInputCloud (grid2x3);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    EXPECT_EQ (grid2x3->size () - 1, boost::num_vertices (graph));
    EXPECT_EQ (grid2x3->size (), indices.size ());
    for (size_t i = 0; i < grid2x3->size () - 1; ++i)
      EXPECT_EQ (i, indices[i]);
    EXPECT_EQ (std::numeric_limits<Graph::vertex_descriptor>::max (), indices.back ());
  }
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeDifferentPointTypes)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZRGBNormal> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZRGBA, Graph> gb (K);
  gb.setInputCloud (grid2x3_with_color);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (11, boost::num_edges (graph));
  ASSERT_EQ (6, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
  {
    const pcl::Normal ZERO_NORMAL;
    EXPECT_XYZ_EQ (grid2x3->at (i), graph[i]);
    EXPECT_NORMAL_EQ (ZERO_NORMAL, graph[i]);
    EXPECT_EQ (grid2x3_with_color->at (i).rgba, graph[i].rgba);
  }
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeWithNaNs)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> gb (K);
  // Without indices
  {
    grid2x3->push_back (pcl::PointXYZ (0, NAN, 0));
    gb.setInputCloud (grid2x3);
    Graph graph;
    gb.compute (graph);
    // We expect that the last (NaN) point will not make it to the output graph
    EXPECT_EQ (11, boost::num_edges (graph));
    ASSERT_EQ (6, boost::num_vertices (graph));
    for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
      EXPECT_XYZ_EQ (grid2x3->at (i), graph[i]);
  }
  // With indices
  {
    grid2x3_with_extra_points->push_back (pcl::PointXYZ (0, NAN, 0));
    grid2x3_indices->push_back (grid2x3_with_extra_points->size () - 1);
    gb.setInputCloud (grid2x3_with_extra_points);
    gb.setIndices (grid2x3_indices);
    Graph graph;
    gb.compute (graph);
    EXPECT_EQ (11, boost::num_edges (graph));
    ASSERT_EQ (6, boost::num_vertices (graph));
    for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
      EXPECT_XYZ_EQ (grid2x3_with_extra_points->at (grid2x3_indices->at (i)), graph[i]);
  }
}

TEST_F (NearestNeighborsGraphBuilderTest, ComputeWithSubgraph)
{
  const size_t K = 3;
  typedef point_cloud_graph<pcl::PointXYZ,
                            boost::vecS,
                            boost::undirectedS,
                            boost::no_property,
                            boost::property<boost::edge_index_t, float> > Graph;
  typedef boost::subgraph<Graph> Subgraph;
  pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Subgraph> gb (K);
  gb.setInputCloud (grid2x3);
  Subgraph graph;
  gb.compute (graph);
  EXPECT_EQ (11, boost::num_edges (graph));
  ASSERT_EQ (6, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
    EXPECT_XYZ_EQ (grid2x3->at (i), graph[i]);
}

class OctreeAdjacencyGraphBuilderTest : public GraphBuilderTest
{

  public:

    OctreeAdjacencyGraphBuilderTest ()
    : grid4x4 (createGrid<pcl::PointXYZ> (4, 4, 1.0, 4))
    , grid4x4_indices (new std::vector<int>)
    , grid4x4_with_extra_points (copyAndAddRandomPoints<pcl::PointXYZ> (*grid4x4, 10, *grid4x4_indices))
    , grid4x4_with_colors (createGrid<pcl::PointXYZRGBA> (4, 4, 1.0, 3))
    , grid4x4_with_normals (createGrid<pcl::PointNormal> (4, 4, 1.0, 2))
    , grid4x4_distorted_with_normals (createGrid<pcl::PointNormal> (4, 4, 1.0, 1, 0.01))
    {
      for (size_t i = 0; i < grid4x4_with_colors->size ();)
      {
        grid4x4_with_colors->at (i++).r = 3;
        grid4x4_with_colors->at (i++).g = 3;
        grid4x4_with_colors->at (i++).b = 3;
      }
      for (size_t i = 0; i < grid4x4_with_normals->size ();)
      {
        grid4x4_with_normals->at (i).normal_z = 2.2;
        grid4x4_with_normals->at (i).curvature = 0.2;
        ++i;
        grid4x4_with_normals->at (i).normal_z = 1.8;
        grid4x4_with_normals->at (i).curvature = 0.4;
        ++i;
      }
      for (size_t i = 0; i < grid4x4_distorted_with_normals->size (); ++i)
      {
        grid4x4_distorted_with_normals->at (i).normal_z = 0.5;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr grid4x4;
    pcl::IndicesPtr grid4x4_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid4x4_with_extra_points;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr grid4x4_with_colors;
    pcl::PointCloud<pcl::PointNormal>::Ptr grid4x4_with_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr grid4x4_distorted_with_normals;

};

TEST_F (OctreeAdjacencyGraphBuilderTest, Constructor)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZRGB> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZRGBA, Graph> gb (RESOLUTION);
  ASSERT_FLOAT_EQ (RESOLUTION, gb.getVoxelResolution ());
}

TEST_F (OctreeAdjacencyGraphBuilderTest, VoxelResolution)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZRGB> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZRGBA, Graph> gb (RESOLUTION);
  gb.setVoxelResolution (RESOLUTION);
  ASSERT_FLOAT_EQ (RESOLUTION, gb.getVoxelResolution ());
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeWithoutCloud)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZRGB> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZRGBA, Graph> gb (RESOLUTION);
  Graph graph (10);
  gb.compute (graph);
  EXPECT_EQ (0, boost::num_vertices (graph));
  EXPECT_EQ (0, boost::num_edges (graph));
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeSamePointTypeWithoutIndices)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeSamePointTypeWithIndices)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4_with_extra_points);
  gb.setIndices (grid4x4_indices);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
}

TEST_F (OctreeAdjacencyGraphBuilderTest, GetPointToVertexMap)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZ> Graph;
  Graph graph;
  std::vector<Graph::vertex_descriptor> indices;
  // Without indices
  {
    pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
    gb.setInputCloud (grid4x4);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    ASSERT_EQ (16, boost::num_vertices (graph));
    ASSERT_EQ (grid4x4->size (), indices.size ());
    for (size_t i = 0; i < indices.size (); ++i)
      EXPECT_XYZ_EQ (grid4x4->at (i), graph[indices[i]]);
  }
  // With indices
  {
    pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
    gb.setInputCloud (grid4x4_with_extra_points);
    gb.setIndices (grid4x4_indices);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    ASSERT_EQ (16, boost::num_vertices (graph));
    ASSERT_EQ (grid4x4_with_extra_points->size (), indices.size ());
    for (size_t i = 0; i < grid4x4_indices->size (); ++i)
      EXPECT_NE (std::numeric_limits<Graph::vertex_descriptor>::max (), indices[grid4x4_indices->at (i)]);
  }
  // With NaN
  {
    pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
    grid4x4->push_back (pcl::PointXYZ (0, NAN, 0));
    gb.setInputCloud (grid4x4);
    gb.compute (graph);
    gb.getPointToVertexMap (indices);
    ASSERT_EQ (16, boost::num_vertices (graph));
    ASSERT_EQ (grid4x4->size (), indices.size ());
    for (size_t i = 0; i < grid4x4->size () - 1; ++i)
      EXPECT_XYZ_EQ (grid4x4->at (i), graph[indices[i]]);
    EXPECT_EQ (std::numeric_limits<Graph::vertex_descriptor>::max (), indices.back ());
  }
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeDifferentPointTypes)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZRGBNormal> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Graph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
  {
    const pcl::Normal ZERO_NORMAL;
    const uint32_t ZERO_RGBA = 0x000000;
    EXPECT_NORMAL_EQ (ZERO_NORMAL, graph[i]);
    EXPECT_FLOAT_EQ (0.0f, graph[i].curvature);
    EXPECT_EQ (ZERO_RGBA, graph[i].rgba);
  }
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeRGBAveraging)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointXYZRGBA> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZRGBA, Graph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4_with_colors);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
    EXPECT_EQ (0x010101, graph[i].rgba);
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeNormalAveraging)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointNormal> Graph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointNormal, Graph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4_with_normals);
  Graph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
  {
    const pcl::Normal EXPECTED_NORMAL (0.0f, 0.0f, 1.0f);
    EXPECT_NORMAL_EQ (EXPECTED_NORMAL, graph[i]);
    EXPECT_FLOAT_EQ (0.3f, graph[i].curvature);
  }
}

TEST_F (OctreeAdjacencyGraphBuilderTest, ComputeWithSubgraph)
{
  const float RESOLUTION = 1.0;
  typedef point_cloud_graph<pcl::PointNormal,
                            boost::vecS,
                            boost::undirectedS,
                            boost::no_property,
                            boost::property<boost::edge_index_t, float> > Graph;
  typedef boost::subgraph<Graph> Subgraph;
  pcl::graph::OctreeAdjacencyGraphBuilder<pcl::PointXYZ, Subgraph> gb (RESOLUTION);
  gb.setInputCloud (grid4x4);
  Subgraph graph;
  gb.compute (graph);
  EXPECT_EQ (42, boost::num_edges (graph));
  ASSERT_EQ (16, boost::num_vertices (graph));
}

int main (int argc, char **argv)
{
  try
  {
    ::testing::InitGoogleTest (&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS ();
  }
  catch (std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what () << "\n";
  }
  return 1;
}

