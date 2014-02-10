#include <boost/make_shared.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "graph/pointcloud_adjacency_list.h"
#include "graph/common.h"

#include "tviewer/tviewer.h"
#include "io.h"
#include "graph_visualizer.h"
#include "measure_runtime.h"

#include "factory/weight_computer_factory.h"
#include "factory/graph_builder_factory.h"

#include "graph/weight.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBNormal PointWithNormalT;
typedef pcl::Normal NormalT;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

typedef boost::subgraph
        <boost::pointcloud_adjacency_list
        <PointWithNormalT,
         boost::vecS,
         boost::undirectedS,
         boost::property<boost::vertex_color_t, uint32_t>,
         boost::property<boost::edge_weight_t, float,
         boost::property<boost::edge_index_t, int>>>> Graph;

using namespace pcl::graph::weight;
typedef weight_computer<PointWithNormalT,
                        terms<
                          tag::normalized<tag::xyz, tag::graph>
                        , tag::drop_if_convex<tag::normal>
                        , tag::drop_if_convex<tag::curvature>
                        , tag::normalized<tag::color, tag::graph>
                        >,
                        pcl::graph::weight::function::gaussian,
                        policy::coerce
                        > WeightComputer;

int main (int argc, char ** argv)
{
  factory::WeightComputerFactory<WeightComputer> wc_factory;
  factory::GraphBuilderFactory<PointT, Graph> gb_factory;

  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pcd-file>\n"
                               "--1-ring (use 1-ring neighborhood for normal computation)\n"
                               "--smoothing-spatial <float>\n"
                               "--smoothing-influence <float>\n"
                               "%s\n"
                               "%s\n"
                               , argv[0]
                               , wc_factory.getUsage ().c_str ()
                               , gb_factory.getUsage ().c_str ());
    return (1);
  }

  typename PointCloudT::Ptr cloud (new PointCloudT);
  typename NormalCloudT::Ptr normals (new NormalCloudT);

  if (!load<PointT> (argv[1], cloud, normals))
    return (1);

  bool neighborhood_1ring = pcl::console::find_switch (argc, argv, "--1-ring");

  float spatial_sigma = 0.012f;
  pcl::console::parse (argc, argv, "--smoothing-spatial", spatial_sigma);

  float influence_sigma = 0.0012f;
  pcl::console::parse (argc, argv, "--smoothing-influence", influence_sigma);

  auto wc = wc_factory.instantiate (argc, argv);
  auto gb = gb_factory.instantiate (argc, argv);

  wc_factory.printValues ();
  gb_factory.printValues ();

  Graph graph;
  gb->setInputCloud (cloud);

  MEASURE_RUNTIME ("Building graph... ", gb->compute (graph));
  MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (graph, neighborhood_1ring));
  MEASURE_RUNTIME ("Smoothening graph... ", pcl::graph::smoothen (graph, spatial_sigma, influence_sigma));
  MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (graph, neighborhood_1ring));
  MEASURE_RUNTIME ("Computing curvature signs... ", pcl::graph::computeSignedCurvatures (graph));
  MEASURE_RUNTIME ("Computing edge weights... ", wc (graph));

  pcl::console::print_info ("Built a graph with %zu vertices and %zu edges\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));

  using namespace tviewer;
  auto viewer = create ();
  GraphVisualizer<Graph> gv (graph);

  viewer->add<PointCloudObject<PointT>> (
      "voxels",
      "voxels centroids",
      "v",
      gv.getVerticesCloudColorsNatural (),
      4,
      0.95
  );

  viewer->add<PointCloudObject<PointT>> (
      "curvature",
      "voxel curvatures",
      "c",
      gv.getVerticesCloudColorsCurvature (),
      4,
      0.95
  );

  viewer->add<NormalCloudObject> (
      "normals",
      "voxel normals",
      "n",
      gv.getVerticesNormalsCloud (),
      1,
      0.01
  );

  viewer->add<PolyDataObject> (
      "edges",
      "graph adjacency edges",
      "a",
      gv.getEdgesPolyData ()
  );

  viewer->show ("edges");
  viewer->run ();

  return (0);
}

