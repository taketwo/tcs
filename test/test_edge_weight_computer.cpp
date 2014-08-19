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
#include "graph/edge_weight_computer.h"

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

using namespace pcl::graph;
using namespace pcl::graph::terms;

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

class EdgeWeightComputerTest : public ::testing::Test
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

    EdgeWeightComputerTest ()
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

float
identity (float val, float influence)
{
  return (influence > 0.0f ? val * influence : 1.0);
};

TEST_F (EdgeWeightComputerTest, TermUnfeasible)
{
  EdgeWeightComputer<GraphLabel> ewc;
  ewc.addTerm<XYZ> (1.0);
  ewc.addTerm<Normal> (1.0);
  //ewc.addTerm<terms::Curvature > (1.0);
  //ewc.addTerm<terms::RGB > (1.0);
  ewc.compute (label_graph);
  Eigen::VectorXf expected (1); expected << 1;
  checkEdgeWeights (label_graph, expected);
}

TEST_F (EdgeWeightComputerTest, TermXYZ)
{
  // Influence = 1 (default)
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (1.0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 4, 4, 4, 4;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0.1
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (0.1);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.4, 0.4, 0.4, 0.4;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 1, 1, 1, 1;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, TermNormal)
{
  // Influence = 1 (default)
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Normal> (1.0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0, 0.29289, 0, 0.29289;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0.1
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Normal> (0.1);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0, 0.029289, 0, 0.029289;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Normal> (0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 1, 1, 1, 1;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, TermCurvature)
{
  // Influence = 1 (default)
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (1.0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0.1
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (0.1);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.002, 0.006, 0.012, 0.004;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 1, 1, 1, 1;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, TermRGB)
{
  // Influence = 1 (default)
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<RGB> (1.0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 1.414214, 1.414214, 1, 1;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0.1
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<RGB> (0.1);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.1414214, 0.1414214, 0.1, 0.1;
    checkEdgeWeights (square_graph, expected);
  }
  // Influence = 0
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<RGB> (0);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 1, 1, 1, 1;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, NormalizationLocalOnSquareGraph)
{
  // Curvature term
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_LOCAL);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.571429, 0.923077, 1.411765, 0.727273;
    checkEdgeWeights (square_graph, expected);
  }
  // Curvature term, non-identity influence
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (0.5, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_LOCAL);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.285714, 0.461539, 0.705882, 0.363636;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, NormalizationLocalOnTriangleGraph)
{
  // XYZ term, gaussian combining function
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<XYZ> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_LOCAL);
    ewc.compute (triangle_graph);
    Eigen::VectorXf expected (3); expected << 0.832363, 0.250663, 0.304468;
    checkEdgeWeights (triangle_graph, expected);
  }
  // Normal, RGB, and curvature terms, gaussian combining function
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<Normal> (2.0);
    ewc.addTerm<RGB> (1.0);
    ewc.addTerm<Curvature> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_LOCAL);
    ewc.compute (triangle_graph);
    Eigen::VectorXf expected (3); expected << 0.130229, 0.553834, 0.272532;
    checkEdgeWeights (triangle_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, NormalizationGlobalOnSquareGraph)
{
  // Curvature term
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_GLOBAL);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.333333, 1.0, 2.0, 0.666667;
    checkEdgeWeights (square_graph, expected);
  }
  // Curvature term, non-identity influence
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (2.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_GLOBAL);
    ewc.compute (square_graph);
    Eigen::VectorXf expected (4); expected << 0.666667, 2.0, 4.0, 1.333334;
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, NormalizationGlobalOnTriangleGraph)
{
  // XYZ term, gaussian combining function
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<XYZ> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_GLOBAL);
    ewc.compute (triangle_graph);
    Eigen::VectorXf expected (3); expected << 0.865688, 0.204633, 0.281048;
    checkEdgeWeights (triangle_graph, expected);
  }
  // XYZ (with non-identity influence), curvature, and RGB terms
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<XYZ> (50.0);
    ewc.addTerm<Curvature> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_GLOBAL);
    ewc.addTerm<RGB> (1.0);
    Eigen::VectorXf expected (3); expected << 0.125556, 0.076153, 0.081268;
    ewc.compute (triangle_graph);
    checkEdgeWeights (triangle_graph, expected);
  }
  // XYZ (with non-identity influence), curvature (with no influence), normal, and RGB terms
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<XYZ> (40.0);
    ewc.addTerm<Curvature> (0.0);
    ewc.addTerm<Normal> (1.0, 1.0, EdgeWeightComputer<Graph>::NORMALIZATION_GLOBAL);
    ewc.addTerm<RGB> (1.0);
    Eigen::VectorXf expected (3); expected << 0.210475, 0.043577, 0.029945;
    ewc.compute (triangle_graph);
    checkEdgeWeights (triangle_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, ConvexInfluenceMultiplier)
{
  // All edges are convex, so all influences will be zeroed
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (1.0, 0.0);
    ewc.addTerm<Normal> (1.0, 0.0);
    ewc.addTerm<Curvature> (1.0, 0.0);
    Eigen::VectorXf expected (3); expected << 1, 1, 1;
    ewc.compute (triangle_graph);
    checkEdgeWeights (triangle_graph, expected);
  }
  // Some vertices are convex, some are not; single term
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<Curvature> (1.0, 0.0);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 1, 1;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
  // Some vertices are convex, some are not;
  // XYZ will have twice as large influence, curvature will be zeroed completely
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (1.0, 2.0);
    ewc.addTerm<Curvature> (1.0, 0.0);
    Eigen::VectorXf expected (4); expected << 0.08, 0.24, 8, 8;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
  // Convexity is not supported
  {
    EdgeWeightComputer<GraphXYZ> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.addTerm<XYZ> (1.0, 0.0);
    Eigen::VectorXf expected (1); expected << 9;
    ewc.compute (xyz_graph);
    checkEdgeWeights (xyz_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, SquareGraph)
{
  // add one diagonal edge
  boost::add_edge (0, 2, square_graph);

  // Normal, curvature with non-identity influence, and RGB terms
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<Normal> (1.0);
    ewc.addTerm<Curvature> (2.0);
    ewc.addTerm<RGB> (1.0);
    Eigen::VectorXf expected (5); expected << 0.240698, 0.176029, 0.346456, 0.269041, 0.178689;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
  // XYZ and normal with non-identity influnce, curvature, and RGB term
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.addTerm<XYZ> (2.0);
    ewc.addTerm<Normal> (2.0);
    ewc.addTerm<Curvature> (1.0);
    ewc.addTerm<RGB> (1.0);
    Eigen::VectorXf expected (5); expected << 0.0322508, 0.026765, 0.0441572, 0.0413184, 0.00373256;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, SmallWeightPolicyIgnore)
{
  EdgeWeightComputer<Graph> ewc;
  ewc.setTermBalancingFunction (identity);
  ewc.setSmallWeightPolicy (EdgeWeightComputer<Graph>::SMALL_WEIGHT_IGNORE);
  ewc.addTerm<Curvature> (1.0);
  Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
  ewc.compute (square_graph);
  checkEdgeWeights (square_graph, expected);
}

TEST_F (EdgeWeightComputerTest, SmallWeightPolicyCoerceToThreshold)
{
  // Default threshold is zero, so no effect
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.setSmallWeightPolicy (EdgeWeightComputer<Graph>::SMALL_WEIGHT_COERCE_TO_THRESHOLD);
    ewc.addTerm<Curvature> (1.0);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
  // With threshold=0.1 we expect to coerce several edges
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.setSmallWeightPolicy (EdgeWeightComputer<Graph>::SMALL_WEIGHT_COERCE_TO_THRESHOLD);
    ewc.setSmallWeightThreshold (0.1);
    ewc.addTerm<Curvature> (1.0);
    Eigen::VectorXf expected (4); expected << 0.1, 0.1, 0.12, 0.1;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, SmallWeightPolicyRemoveEdge)
{
  // Default threshold is zero, so no effect
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.setSmallWeightPolicy (EdgeWeightComputer<Graph>::SMALL_WEIGHT_REMOVE_EDGE);
    ewc.addTerm<Curvature> (1.0);
    Eigen::VectorXf expected (4); expected << 0.02, 0.06, 0.12, 0.04;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
  // With threshold=0.1 we expect to remove several edges
  {
    EdgeWeightComputer<Graph> ewc;
    ewc.setTermBalancingFunction (identity);
    ewc.setSmallWeightPolicy (EdgeWeightComputer<Graph>::SMALL_WEIGHT_REMOVE_EDGE);
    ewc.setSmallWeightThreshold (0.1);
    ewc.addTerm<Curvature> (1.0);
    Eigen::VectorXf expected (1); expected << 0.12;
    ewc.compute (square_graph);
    checkEdgeWeights (square_graph, expected);
  }
}

TEST_F (EdgeWeightComputerTest, SmallWeightPolicyRemoveEdgeWithSubgraph)
{
  EdgeWeightComputer<Subgraph> ewc;
  ewc.setTermBalancingFunction (identity);
  ewc.setSmallWeightPolicy (EdgeWeightComputer<Subgraph>::SMALL_WEIGHT_REMOVE_EDGE);
  ewc.setSmallWeightThreshold (0.1);
  ewc.addTerm<Curvature> (1.0);

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
    ewc.compute (graph);
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
    ewc.compute (graph);
    EXPECT_EQ (2, boost::num_edges (graph));
    EXPECT_EQ (3, boost::num_edges (sub));
  }
  // Remove on subgraph, no effect on root graph
  {
    Subgraph graph (cloud);
    boost::add_edge (0, 1, graph);
    boost::add_edge (1, 2, graph);
    boost::add_edge (2, 0, graph);
    Subgraph sub (graph.create_subgraph ());
    boost::add_vertex (0, sub);
    boost::add_vertex (1, sub);
    boost::add_vertex (2, sub);
    ewc.compute (sub);
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

