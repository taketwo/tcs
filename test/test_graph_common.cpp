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

#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>

#include "graph/common.h"
#include "graph/point_cloud_graph.h"
#include "graph/nearest_neighbors_graph_builder.h"

typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;
typedef typename PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::graph::point_cloud_graph<PointT,
                                      boost::vecS,
                                      boost::undirectedS,
                                      boost::property<boost::vertex_color_t, uint32_t>,
                                      boost::property<boost::edge_index_t, float> > Graph;
typedef typename Graph::vertex_descriptor VertexId;
typedef boost::subgraph<Graph> Subgraph;
typedef boost::reference_wrapper<Subgraph> SubgraphRef;

PointCloudPtr
generateRandomPlanarCloud (size_t num_points = 1000)
{
  using namespace pcl::common;
  CloudGenerator<PointT, UniformGenerator<float> > generator;
  UniformGenerator<float>::Parameters x_params (-1.0f, 1.0f, 1);
  generator.setParametersForX (x_params);
  UniformGenerator<float>::Parameters y_params (-1.0f, 1.0f, 2);
  generator.setParametersForY (y_params);
  UniformGenerator<float>::Parameters z_params ( 1.0f, 1.0001f);
  generator.setParametersForZ (z_params);
  PointCloudPtr output (new PointCloud);
  generator.fill (num_points, 1, *output);
  return output;
}

TEST (GraphCommon, ComputeNormalsAndCurvature)
{
  pcl::graph::NearestNeighborsGraphBuilder<PointT, Graph> gb;
  gb.setInputCloud (generateRandomPlanarCloud ());
  Graph graph;
  gb.compute (graph);
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
  {
    // Fill in some non-sense
    graph[i].getNormalVector4fMap () << rand (), rand (), rand (), rand ();
    graph[i].curvature = rand ();
  }
  pcl::graph::computeNormalsAndCurvatures (graph);
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
  {
    const pcl::Normal EXPECTED_NORMAL (0.0f, 0.0f, -1.0f);
    EXPECT_NORMAL_NEAR (EXPECTED_NORMAL, graph[i], 0.001);
    EXPECT_NEAR (0.0f, graph[i].curvature, 0.001);
  }
}

TEST (GraphCommon, ComputeSignedCurvature)
{
  const size_t ROWS = 5;
  const size_t COLS = 12;
  const float STEP = 1.0;
  // Create a "staircase" cloud
  //
  //     top view            side view
  //
  //   * * * * * * * *           * * * * *
  //   * * * * * * * *           *
  //   * * * * * * * *           *
  //   * * * * * * * *           *
  //   * * * * * * * *     * * * *
  //
  PointCloudPtr cloud (new PointCloud (ROWS * COLS, 1));
  size_t idx = 0;
  for (size_t y = 0; y < ROWS; ++y)
  {
    for (size_t x = 0; x < COLS; ++x, ++idx)
    {
      cloud->at (idx).y = y * STEP;
      if (x < COLS / 3)
      {
        cloud->at (idx).x = x * STEP;
        cloud->at (idx).z = -5.0f;
      }
      else if (x < 2 * COLS / 3)
      {
        cloud->at (idx).x = (COLS / 3 - 1) * STEP;
        cloud->at (idx).z = (x - COLS / 3 + 1) * STEP - 5.0f;
      }
      else
      {
        cloud->at (idx).x = (x - COLS / 3) * STEP;
        cloud->at (idx).z = (COLS / 3) * STEP - 5.0f;
      }
    }
  }

  pcl::graph::NearestNeighborsGraphBuilder<PointT, Graph> gb;
  gb.setNumberOfNeighbors (6);
  gb.setInputCloud (cloud);
  Graph graph;
  gb.compute (graph);
  pcl::graph::computeNormalsAndCurvatures (graph);

  // Expect that curvatures are all non-negative
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
    EXPECT_LE (0.0f, graph[i].curvature);

  pcl::graph::computeSignedCurvatures (graph);

  // Expect curvatures of the points on the concave corner to be less than zero,
  // and on the convex corner the other way round.
  for (Graph::vertex_descriptor i = 0; i < boost::num_vertices (graph); ++i)
    if (i % COLS == 3)
      EXPECT_GT (0.0f, graph[i].curvature);
    else if (i % COLS == 7)
      EXPECT_LT (0.0f, graph[i].curvature);
}

TEST (GraphCommon, CreateSubgraphsFromConnectedComponents)
{
  const size_t N = 100;
  pcl::graph::NearestNeighborsGraphBuilder<PointT, Subgraph> gb;
  gb.setNumberOfNeighbors (0);
  gb.setInputCloud (generateRandomPlanarCloud (N));
  // No edges, expect as many components as there are vertices
  {
    Subgraph graph;
    gb.compute (graph);
    std::vector<SubgraphRef> subgraphs;
    size_t n = pcl::graph::createSubgraphsFromConnectedComponents (graph, subgraphs);
    ASSERT_EQ (boost::num_vertices (graph), n);
    ASSERT_EQ (boost::num_vertices (graph), subgraphs.size ());
    for (size_t i = 0; i < n; ++i)
      EXPECT_EQ (1, boost::num_vertices (subgraphs[i].get ()));
  }
  // Several connected components
  {
    Subgraph graph;
    gb.compute (graph);
    for (size_t i = 0; i < N; ++i)
      if (i % 10)
        boost::add_edge ((i / 10) * 10, i, graph);
    std::vector<SubgraphRef> subgraphs;
    size_t n = pcl::graph::createSubgraphsFromConnectedComponents (graph, subgraphs);
    ASSERT_EQ (N / 10, n);
    ASSERT_EQ (N / 10, subgraphs.size ());
    for (size_t i = 0; i < n; ++i)
      EXPECT_EQ (10, boost::num_vertices (subgraphs[i].get ()));
  }
}

TEST (GraphCommon, CreateSubgraphsFromConnectedComponentsInNonRootSubgraph)
{
  Subgraph graph (5);
  boost::add_edge (1, 2, graph);
  boost::add_edge (3, 4, graph);
  Subgraph s1 = graph.create_subgraph ();
  boost::add_vertex (1, s1);
  boost::add_vertex (2, s1);
  boost::add_vertex (3, s1);
  boost::add_vertex (4, s1);
  std::vector<SubgraphRef> subgraphs;
  pcl::graph::createSubgraphsFromConnectedComponents (s1, subgraphs);
  ASSERT_EQ (2, subgraphs.size ());
  EXPECT_EQ (1, subgraphs[0].get ().local_to_global (0));
  EXPECT_EQ (3, subgraphs[1].get ().local_to_global (0));
}

TEST (GraphCommon, CreateSubgraphsFromColorMap)
{
  const size_t N = 100;
  PointCloudPtr cloud = generateRandomPlanarCloud (N);
  Subgraph graph (cloud);

  std::vector<pcl::PointIndices> indices (3);
  for (size_t i = 0; i < N; ++i)
    indices[i % 3].indices.push_back (i);

  std::vector<SubgraphRef> subgraphs;
  pcl::graph::createSubgraphsFromIndices (graph, indices, subgraphs);
  ASSERT_EQ (indices.size () + 1, subgraphs.size ());
  ASSERT_EQ (0, boost::num_vertices (subgraphs[3].get ()));
  for (size_t i = 0; i < indices.size (); ++i)
  {
    ASSERT_EQ (indices[i].indices.size (), boost::num_vertices (subgraphs[i].get ()));
    for (size_t j = 0; j < boost::num_vertices (subgraphs[i].get ()); ++j)
      EXPECT_XYZ_EQ (cloud->at (indices[i].indices[j]), subgraphs[i].get ()[j]);
  }
}

TEST (GraphCommon, CreateSubgraphsFromIndicesSingle)
{
  const size_t N = 100;
  PointCloudPtr cloud = generateRandomPlanarCloud (N);
  Subgraph graph (cloud);
  std::vector<SubgraphRef> subgraphs;

  boost::property_map<Graph, boost::vertex_color_t>::type colors;

  // All colors are unique
  {
    for (VertexId i = 0; i < N; ++i)
      boost::put (boost::vertex_color, graph, i, i);
    size_t n = pcl::graph::createSubgraphsFromColorMap (graph,
                                                        boost::get (boost::vertex_color, graph),
                                                        subgraphs);
    ASSERT_EQ (N, n);
    ASSERT_EQ (N, subgraphs.size ());
    for (size_t i = 0; i < boost::num_vertices (subgraphs[0].get ()); ++i)
    {
      ASSERT_EQ (1, boost::num_vertices (subgraphs[i].get ()));
      EXPECT_XYZ_EQ (cloud->at (i), subgraphs[i].get ()[static_cast<VertexId> (0)]);
    }
  }

  // All colors are the same
  {
    for (VertexId i = 0; i < N; ++i)
      boost::put (boost::vertex_color, graph, i, 1);
    size_t n = pcl::graph::createSubgraphsFromColorMap (graph,
                                                        boost::get (boost::vertex_color, graph),
                                                        subgraphs);
    ASSERT_EQ (1, n);
    ASSERT_EQ (1, subgraphs.size ());
    ASSERT_EQ (N, boost::num_vertices (subgraphs[0].get ()));
  }

  // Reordering in the order of increasing color
  {
    boost::put (boost::vertex_color, graph, 0, 2);
    boost::put (boost::vertex_color, graph, N - 1, 0);
    // ... and the rest remains ones from the previous test
    size_t n = pcl::graph::createSubgraphsFromColorMap (graph,
                                                        boost::get (boost::vertex_color, graph),
                                                        subgraphs);
    ASSERT_EQ (3, n);
    ASSERT_EQ (3, subgraphs.size ());
    ASSERT_EQ (N - 2, boost::num_vertices (subgraphs[1].get ()));
    EXPECT_XYZ_EQ (cloud->at (0), subgraphs[2].get ()[static_cast<VertexId> (0)]);
    EXPECT_XYZ_EQ (cloud->at (N - 1), subgraphs[0].get ()[static_cast<VertexId> (0)]);
  }
}

TEST (GraphCommon, CreateSubgraphsFromIndicesVector)
{
  const size_t N = 100;
  PointCloudPtr cloud = generateRandomPlanarCloud (N);
  Subgraph graph (cloud);

  std::vector<pcl::PointIndices> indices (3);
  for (size_t i = 0; i < N; ++i)
    indices[i % 3].indices.push_back (i);

  std::vector<SubgraphRef> subgraphs;
  pcl::graph::createSubgraphsFromIndices (graph, indices, subgraphs);
  ASSERT_EQ (indices.size () + 1, subgraphs.size ());
  ASSERT_EQ (0, boost::num_vertices (subgraphs[3].get ()));
  for (size_t i = 0; i < indices.size (); ++i)
  {
    ASSERT_EQ (indices[i].indices.size (), boost::num_vertices (subgraphs[i].get ()));
    for (size_t j = 0; j < boost::num_vertices (subgraphs[i].get ()); ++j)
      EXPECT_XYZ_EQ (cloud->at (indices[i].indices[j]), subgraphs[i].get ()[j]);
  }
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

