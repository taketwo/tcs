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

#include <pcl/pcl_tests.h>

#include "graph/point_cloud_graph.h"
#include "graph/weight.h"

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;
typedef typename PointCloud::ConstPtr PointCloudConstPtr;

using namespace pcl::graph;

typedef point_cloud_graph<PointT,
                          boost::vecS,
                          boost::undirectedS,
                          boost::property<boost::vertex_color_t, int>,
                          boost::property<boost::edge_weight_t, float,
                          boost::property<boost::edge_index_t, int>>,
                          boost::listS>       Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor        VertexId;
typedef boost::graph_traits<Graph>::edge_descriptor          EdgeId;
typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef typename boost::graph_traits<Graph>::edge_iterator   EdgeIterator;
typedef boost::subgraph<Graph>                               Subgraph;
typedef boost::graph_traits<Subgraph>::vertex_descriptor     SubgraphVertexId;

class WeightComputerTest : public ::testing::Test
{

  public:

    typedef point_cloud_graph<pcl::Label,
                              boost::vecS,
                              boost::undirectedS,
                              boost::property<boost::vertex_color_t, int>,
                              boost::property<boost::edge_weight_t, float,
                              boost::property<boost::edge_index_t, int>>,
                              boost::listS> GraphLabel;

    typedef point_cloud_graph<pcl::PointXYZ,
                              boost::vecS,
                              boost::undirectedS,
                              boost::property<boost::vertex_color_t, int>,
                              boost::property<boost::edge_weight_t, float,
                              boost::property<boost::edge_index_t, int>>,
                              boost::listS> GraphXYZ;

    typedef point_cloud_graph<pcl::PointNormal,
                              boost::vecS,
                              boost::undirectedS,
                              boost::property<boost::vertex_color_t, int>,
                              boost::property<boost::edge_weight_t, float,
                              boost::property<boost::edge_index_t, int>>,
                              boost::listS> GraphXYZNormal;

    WeightComputerTest ()
    : square_graph (4)
    , triangle_graph (3)
    , xyz_graph (2)
    , label_graph (2)
    {
      square_graph[0].getVector3fMap () << 0, 0, 0;
      square_graph[1].getVector3fMap () << 2, 0, 0;
      square_graph[2].getVector3fMap () << 2, 2, 0;
      square_graph[3].getVector3fMap () << 0, 2, 0;
      square_graph[0].getNormalVector3fMap () << 1, 0, 0;
      square_graph[1].getNormalVector3fMap () << 1, 0, 0;
      square_graph[2].getNormalVector3fMap () << 0.707107, 0.707107, 0;
      square_graph[3].getNormalVector3fMap () << 0.707107, 0.707107, 0;
      square_graph[0].curvature =  0.1;
      square_graph[1].curvature = -0.2;
      square_graph[2].curvature =  0.3;
      square_graph[3].curvature =  0.4;
      square_graph[0].r = 255;
      square_graph[1].g = 255;
      square_graph[2].b = 255;
      boost::add_edge (0, 1, square_graph);
      boost::add_edge (1, 2, square_graph);
      boost::add_edge (2, 3, square_graph);
      boost::add_edge (3, 0, square_graph);

      triangle_graph[0].getVector3fMap () << 4, 2, 1;
      triangle_graph[1].getVector3fMap () << 1, 2, 2;
      triangle_graph[2].getVector3fMap () << 8, 8, 7;
      triangle_graph[0].getNormalVector3fMap () << 1, 0, 0;
      triangle_graph[1].getNormalVector3fMap () << 0, 1, 0;
      triangle_graph[2].getNormalVector3fMap () << 0, 0.707107, 0.707107;
      triangle_graph[0].curvature = 0.5;
      triangle_graph[1].curvature = 0.25;
      triangle_graph[2].curvature = 0.1;
      boost::add_edge (0, 1, triangle_graph);
      boost::add_edge (1, 2, triangle_graph);
      boost::add_edge (2, 0, triangle_graph);

      xyz_graph[0].getVector3fMap () << 0, 0, 0;
      xyz_graph[0].getVector3fMap () << 3, 0, 0;
      boost::add_edge (0, 1, xyz_graph);

      label_graph[0].label = 1;
      label_graph[0].label = 2;
      boost::add_edge (0, 1, label_graph);
    }

    template <typename Graph>
    void checkEdgeWeights (Graph& graph, const Eigen::VectorXf& expected, float deviation = 0.0002) const
    {
      ASSERT_EQ (expected.size (), boost::num_edges (graph));
      size_t i = 0;
      typename boost::graph_traits<Graph>::edge_iterator ei, ee;
      for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
        EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, graph, *ei), deviation);
    }

    Graph square_graph;
    Graph triangle_graph;
    GraphXYZ xyz_graph;
    GraphLabel label_graph;

};

TEST_F (WeightComputerTest, TermUnfeasible)
{
  using namespace pcl::graph::weight;
  weight_computer<pcl::Label,
                  terms<
                    tag::xyz
                  , tag::normal
                  , tag::curvature
                  , tag::color
                  >,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (label_graph);
  Eigen::VectorXf expected (1); expected << 1;
  checkEdgeWeights (label_graph, expected);
}

TEST_F (WeightComputerTest, TermXYZ)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::xyz>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 4, 4, 4, 4;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermXYZWithScale)
{
  using namespace pcl::graph::weight;
  Eigen::VectorXf expected0 (4); expected0 << 0, 0, 0, 0;
  Eigen::VectorXf expected10 (4); expected10 << 0.4, 0.4, 0.4, 0.4;
  // Scale = 10
  {
    weight_computer<PointT,
                    terms<tag::xyz>,
                    function::identity,
                    policy::ignore> computeWeights (tag::xyz::scale = 10);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 10, plus scales for other terms
  {
    weight_computer<PointT,
                    terms<tag::xyz>,
                    function::identity,
                    policy::ignore> computeWeights (tag::curvature::scale = 5,
                                                    tag::xyz::scale = 10,
                                                    tag::normal::scale = 0);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 0
  {
    weight_computer<PointT,
                    terms<tag::xyz>,
                    function::identity,
                    policy::ignore> computeWeights (tag::xyz::scale = 0);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected0);
  }
}

TEST_F (WeightComputerTest, TermNormal)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::normal>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0, 0.29289, 0, 0.29289;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermNormalWithScale)
{
  using namespace pcl::graph::weight;
  Eigen::VectorXf expected0 (4); expected0 << 0, 0, 0, 0;
  Eigen::VectorXf expected10 (4); expected10 << 0, 0.029289, 0, 0.029289;
  // Scale = 10
  {
    weight_computer<PointT,
                    terms<tag::normal>,
                    function::identity,
                    policy::ignore> computeWeights (tag::normal::scale = 10);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 10, plus scales for other terms
  {
    weight_computer<PointT,
                    terms<tag::normal>,
                    function::identity,
                    policy::ignore> computeWeights (tag::curvature::scale = 5,
                                                    tag::xyz::scale = 0,
                                                    tag::normal::scale = 10);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 0
  {
    weight_computer<PointT,
                    terms<tag::normal>,
                    function::identity,
                    policy::ignore> computeWeights (tag::normal::scale = 0);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected0);
  }
}

TEST_F (WeightComputerTest, TermCurvature)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::curvature>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermCurvatureWithScale)
{
  using namespace pcl::graph::weight;
  Eigen::VectorXf expected0 (4); expected0 << 0, 0, 0, 0;
  Eigen::VectorXf expected10 (4); expected10 << 0.002, 0.006, 0.012, 0.004;
  // Scale = 10
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::ignore> computeWeights (tag::curvature::scale = 10);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 10, plus scales for other terms
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::ignore> computeWeights (tag::curvature::scale = 10,
                                                    tag::xyz::scale = 0,
                                                    tag::normal::scale = 5);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 0
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::ignore> computeWeights (tag::curvature::scale = 0);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected0);
  }
}

TEST_F (WeightComputerTest, TermColor)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::color>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 1.414214, 1.414214, 1, 1;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermColorWithScale)
{
  using namespace pcl::graph::weight;
  Eigen::VectorXf expected0 (4); expected0 << 0, 0, 0, 0;
  Eigen::VectorXf expected10 (4); expected10 << 0.1414214, 0.1414214, 0.1, 0.1;
  // Scale = 10
  {
    weight_computer<PointT,
                    terms<tag::color>,
                    function::identity,
                    policy::ignore> computeWeights (tag::color::scale = 10);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 10, plus scales for other terms
  {
    weight_computer<PointT,
                    terms<tag::color>,
                    function::identity,
                    policy::ignore> computeWeights (tag::color::scale = 10,
                                                    tag::xyz::scale = 0,
                                                    tag::normal::scale = 5);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected10);
  }
  // Scale = 0
  {
    weight_computer<PointT,
                    terms<tag::color>,
                    function::identity,
                    policy::ignore> computeWeights (tag::color::scale = 0);
    computeWeights (square_graph);
    checkEdgeWeights (square_graph, expected0);
  }
}

TEST_F (WeightComputerTest, TermConvexity)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::convexity>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << -1, -1, 1, 1;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermCurvatureVertexNormalized)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<
                    tag::normalized<tag::curvature, tag::vertex>
                  >,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.571429, 0.923077, 1.411765, 0.727273;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermCurvatureVertexNormalizedWithScale)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<
                    tag::normalized<tag::curvature, tag::vertex>
                  >,
                  function::identity,
                  policy::ignore> computeWeights (tag::curvature::scale = 2.0f);
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.285714, 0.461539, 0.705882, 0.363636;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermCurvatureGraphNormalized)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<
                    tag::normalized<tag::curvature, tag::graph>
                  >,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.333333, 1.0, 2.0, 0.666667;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, TermCurvatureGraphNormalizedWithScale)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<
                    tag::normalized<tag::curvature, tag::graph>
                  >,
                  function::identity,
                  policy::ignore> computeWeights (tag::curvature::scale = 0.5f);
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.666667, 2.0, 4.0, 1.333334;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, WithNormalized)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<
                    tag::normalized<tag::curvature, tag::vertex>
                  , tag::normal
                  , tag::color
                  >,
                  function::gaussian,
                  policy::ignore> computeWeights;
  PointCloudPtr cloud (new PointCloud (3, 1));
  cloud->at (0).getNormalVector3fMap () << 1, 0, 0;
  cloud->at (1).getNormalVector3fMap () << 0, 1, 0;
  cloud->at (2).getNormalVector3fMap () << 0, 1, 0;
  cloud->at (0).curvature = 0.5;
  cloud->at (1).curvature = 0.5;
  cloud->at (2).curvature = 0.1;
  Graph g (cloud);
  boost::add_edge (0, 1, g);
  boost::add_edge (1, 2, g);
  boost::add_edge (2, 0, g);
  computeWeights (g);
  // Color has no effect
  // Expected weights due to normals: 1, 0, 1 -> 0.3678, 1, 0.3678
  // Expected weights due to curvature: 0.25, 0.05, 0.05 -> 1.6667, 0.5, 0.5 -> 0.1888, 0.6065, 0.6065
  // Expectde weights: 0.0694, 0.6065, 0.2230
  std::vector<float> expected = {0.0694, 0.6065, 0.2230};
  size_t i = 0;
  typename boost::graph_traits<Graph>::edge_iterator ei, ee;
  for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei)
    EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, g, *ei), 0.0002);
}

TEST_F (WeightComputerTest, TriangleWithoutNormalized)
{
  using namespace pcl::graph::weight;
  typename boost::graph_traits<Graph>::edge_iterator ei, ee;
  // Scaled xyz, curvature, color (no effect)
  {
    weight_computer<PointT,
                    terms<
                      tag::curvature
                    , tag::xyz
                    , tag::color
                    >,
                    function::gaussian,
                    policy::ignore> computeWeights (tag::xyz::scale=50.0);
    Eigen::VectorXf expected (3); expected << 0.722527, 0.108067, 0.163654;
    computeWeights (triangle_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (triangle_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, triangle_graph, *ei), 0.00001);
  }
  // Scaled xyz, scaled to zero curvature, normal, color (no effect)
  {
    weight_computer<PointT,
                    terms<
                      tag::curvature
                    , tag::xyz
                    , tag::normal
                    , tag::color
                    >,
                    function::gaussian,
                    policy::ignore> computeWeights (tag::xyz::scale = 40.0,
                                                    tag::curvature::scale = 0.0);
    Eigen::VectorXf expected (3); expected << 0.286505, 0.0476967, 0.0407622;
    computeWeights (triangle_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (triangle_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, triangle_graph, *ei), 0.00001);
  }
  // Scaled xyz, normal, curvature
  {
    weight_computer<PointT,
                    terms<
                      tag::xyz
                    , tag::normal
                    , tag::curvature
                    >,
                    function::gaussian,
                    policy::ignore> computeWeights (tag::xyz::scale=40.0);
    Eigen::VectorXf expected (3); expected << 0.25284, 0.0465191, 0.0387742;
    computeWeights (triangle_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (triangle_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, triangle_graph, *ei), 0.00001);
  }
}

TEST_F (WeightComputerTest, DropIfConvex)
{
  using namespace pcl::graph::weight;
  typename boost::graph_traits<Graph>::edge_iterator ei, ee;
  // All vertices are convex, so all terms should be dropped
  {
    weight_computer<PointT,
                    terms<
                      tag::drop_if_convex<tag::xyz>
                    , tag::drop_if_convex<tag::normal>
                    , tag::drop_if_convex<tag::curvature>
                    >,
                    function::identity,
                    policy::ignore> computeWeights;
    Eigen::VectorXf expected (3); expected << 1, 1, 1;
    computeWeights (triangle_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (triangle_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, triangle_graph, *ei), 0.00001);
  }
  // Some vertices are convex, some are not; only droppable term
  {
    weight_computer<PointT,
                    terms<
                      tag::drop_if_convex<tag::curvature>
                    >,
                    function::identity,
                    policy::ignore> computeWeights;
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 1, 1;
    computeWeights (square_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (square_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, square_graph, *ei), 0.00001);
  }
  // Some vertices are convex, some are not; several terms
  {
    weight_computer<PointT,
                    terms<
                      tag::drop_if_convex<tag::curvature>
                    , tag::xyz
                    >,
                    function::identity,
                    policy::ignore> computeWeights;
    Eigen::VectorXf expected (4); expected << 0.08, 0.24, 4, 4;
    computeWeights (square_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (square_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, square_graph, *ei), 0.00001);
  }
  // Droppable terms, but convexity is not supported
  {
    weight_computer<pcl::PointXYZ,
                    terms<
                      tag::drop_if_convex<tag::xyz>
                    >,
                    function::identity,
                    policy::ignore> computeWeights;
    Eigen::VectorXf expected (2); expected << 9, 9;
    computeWeights (xyz_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (xyz_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, xyz_graph, *ei), 0.00001);
  }
}

TEST_F (WeightComputerTest, SquareWithoutNormalized)
{
  using namespace pcl::graph::weight;
  // add one diagonal edge
  boost::add_edge (0, 2, square_graph);
  typename boost::graph_traits<Graph>::edge_iterator ei, ee;
  // Normal, scaled curvature, color
  {
    weight_computer<PointT,
                    terms<
                      tag::curvature
                    , tag::normal
                    , tag::color
                    >,
                    function::gaussian,
                    policy::ignore> computeWeights (tag::curvature::scale=2.0);
    Eigen::VectorXf expected (5); expected << 0.240698, 0.176029, 0.346456, 0.269041, 0.178689;
    computeWeights (square_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (square_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, square_graph, *ei), 0.00001);
  }
  // Scaled xyz, scaled normal, curvature, color
  {
    weight_computer<PointT,
                    terms<
                      tag::curvature
                    , tag::xyz
                    , tag::normal
                    , tag::color
                    >,
                    function::gaussian,
                    policy::ignore> computeWeights (tag::xyz::scale = 2.0,
                                                    tag::normal::scale = 2.0);
    Eigen::VectorXf expected (5); expected << 0.0322508, 0.026765, 0.0441572, 0.0413184, 0.00373256;
    computeWeights (square_graph);
    size_t i = 0;
    for (boost::tie (ei, ee) = boost::edges (square_graph); ei != ee; ++ei)
      EXPECT_NEAR (expected[i++], boost::get (boost::edge_weight, square_graph, *ei), 0.0002);
  }
}

TEST_F (WeightComputerTest, SmallWeightPolicyIgnore)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::curvature>,
                  function::identity,
                  policy::ignore> computeWeights;
  computeWeights (square_graph);
  Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
  checkEdgeWeights (square_graph, expected);
}

TEST_F (WeightComputerTest, SmallWeightPolicyCoerce)
{
  using namespace pcl::graph::weight;
  // Default threshold is much smaller than the weights, so no effect
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::coerce> computeWeights;
    computeWeights (square_graph);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
    checkEdgeWeights (square_graph, expected);
  }
  // With threshold=0.1 we expect to coerce several edges
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::coerce> computeWeights (tag::weight::threshold = 0.1);
    computeWeights (square_graph);
    Eigen::VectorXf expected (4); expected << 0.1, 0.1, 0.12, 0.1;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (WeightComputerTest, SmallWeightPolicyRemove)
{
  using namespace pcl::graph::weight;
  // Default threshold is much smaller than the weights, so no effect
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::remove> computeWeights;
    computeWeights (square_graph);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
    checkEdgeWeights (square_graph, expected);
  }
  // With threshold=0.1 we expect to remove several edges
  {
    weight_computer<PointT,
                    terms<tag::curvature>,
                    function::identity,
                    policy::remove> computeWeights (tag::weight::threshold = 0.1);
    computeWeights (square_graph);
    Eigen::VectorXf expected (1); expected << 0.12;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (WeightComputerTest, SmallWeightPolicyRemoveWithSubgraph)
{
  using namespace pcl::graph::weight;
  weight_computer<PointT,
                  terms<tag::curvature>,
                  function::identity,
                  policy::remove> computeWeights (tag::weight::threshold = 0.1);
  PointCloudPtr cloud (new PointCloud (3, 1));
  cloud->at (0).curvature = 5.0;
  cloud->at (1).curvature = 0.1;
  cloud->at (2).curvature = 0.1;
  // Remove on root graph, no subgraphs
  {
    Subgraph graph (cloud);
    boost::add_edge (0, 1, graph);
    boost::add_edge (1, 2, graph);
    boost::add_edge (2, 0, graph);
    computeWeights (graph);
    EXPECT_EQ (2, boost::num_edges (graph));
  }
  // Remove on root graph, with subraph (no effect on it)
  {
    Subgraph graph (cloud);
    boost::add_edge (0, 1, graph);
    boost::add_edge (1, 2, graph);
    boost::add_edge (2, 0, graph);
    Subgraph sub (graph.create_subgraph ());
    boost::add_vertex (0, sub);
    boost::add_vertex (1, sub);
    boost::add_vertex (2, sub);
    computeWeights (graph);
    EXPECT_EQ (2, boost::num_edges (graph));
    EXPECT_EQ (3, boost::num_edges (sub));
  }
  // Remove on subgraph graph, no effect on root graph
  {
    Subgraph graph (cloud);
    boost::add_edge (0, 1, graph);
    boost::add_edge (1, 2, graph);
    boost::add_edge (2, 0, graph);
    Subgraph sub (graph.create_subgraph ());
    boost::add_vertex (0, sub);
    boost::add_vertex (1, sub);
    boost::add_vertex (2, sub);
    computeWeights (sub);
    EXPECT_EQ (3, boost::num_edges (graph));
    EXPECT_EQ (2, boost::num_edges (sub));
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

