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


int main (int argc, char ** argv)
{
  factory::WeightComputerFactory<PointWithNormalT, Graph> wc_factory;
  factory::GraphBuilderFactory<PointT, Graph> gb_factory;

  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pcd-file>\n"
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

  auto wc = wc_factory.instantiate (argc, argv);
  pcl::graph::GraphBuilder<PointT, Graph>::Ptr gb = gb_factory.instantiate (argc, argv);

  wc_factory.printValues ();
  gb_factory.printValues ();

  Graph graph;
  gb->setInputCloud (cloud);

  MEASURE_RUNTIME ("Building graph... ", gb->compute (graph));
  MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (graph));
  MEASURE_RUNTIME ("Computing curvature signs... ", pcl::graph::computeSignedCurvatures (graph));
  MEASURE_RUNTIME ("Computing edge weights... ", wc (graph));

  pcl::console::print_info ("Built a graph with %zu vertices and %zu edges\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));

  using namespace tviewer;
  auto viewer = create ();
  GraphVisualizer<Graph> gv (graph);

  viewer->registerVisualizationObject<PointCloudObject<PointT>> (
      "voxels",
      "voxels centroids",
      "v",
      gv.getVerticesCloudColorsNatural (),
      4,
      0.95
  );

  viewer->registerVisualizationObject<PointCloudObject<PointT>> (
      "curvature",
      "voxel curvatures",
      "c",
      gv.getVerticesCloudColorsCurvature (),
      4,
      0.95
  );

  //viewer->registerVisualizationObject<PointCloudObject<PointT>> (
      //"degree",
      //"voxel degrees",
      //"d",
      //gv.getVerticesCloudColorsDegree (),
      //4,
      //0.95
  //);

  viewer->registerVisualizationObject<NormalCloudObject> (
      "normals",
      "voxel normals",
      "n",
      gv.getVerticesNormalsCloud (),
      1,
      0.01
  );

  viewer->registerVisualizationObject<PolyDataObject> (
      "edges",
      "graph adjacency edges",
      "a",
      gv.getEdgesPolyData ()
  );

  viewer->showVisualizationObject ("edges");
  viewer->run ();

  return (0);
}

