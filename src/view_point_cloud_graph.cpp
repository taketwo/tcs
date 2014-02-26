#include <boost/make_shared.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define WC_WITHOUT_CONVEX_DROP
#include "typedefs.h"

#include "io.h"
#include "tviewer/tviewer.h"
#include "measure_runtime.h"
#include "graph_visualizer.h"
#include "factory/graph_factory.h"


int main (int argc, char ** argv)
{
  factory::GraphFactory<PointT, Graph, WeightComputer> g_factory;

  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pcd-file>\n"
                               "--1-ring (use 1-ring neighborhood for normal computation)\n"
                               "--smoothing-spatial <float>\n"
                               "--smoothing-influence <float>\n"
                               "%s\n"
                               "%s\n"
                               , argv[0]
                               , g_factory.getUsage ().c_str ());
    return (1);
  }

  typename PointCloudT::Ptr cloud (new PointCloudT);
  typename NormalCloudT::Ptr normals (new NormalCloudT);

  if (!load<PointT> (argv[1], cloud, normals))
    return (1);


  /*********************************************************************
   *                         Pre-compute graph                         *
   *********************************************************************/


  auto g = g_factory.instantiate (cloud, argc, argv);
  auto& graph = g.get ();

  g_factory.printValues ();

  pcl::console::print_info ("Working with component of size: %zu / %zu\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));


  /*********************************************************************
   *                          Visualize graph                          *
   *********************************************************************/


  using namespace tviewer;
  auto viewer = create (argc, argv);

  typedef GraphVisualizer<Graph> GraphVisualizer;
  GraphVisualizer gv (graph);

  viewer->add<PointCloudObject<pcl::PointXYZRGBA>> (
      "vertices",
      "graph vertices",
      "v",
      gv.getVerticesCloudColorsNatural (),
      6,
      0.95
  );

  viewer->add<PointCloudObject<pcl::PointXYZRGBA>> (
      "curvature",
      "vertex curvature",
      "C",
      gv.getVerticesCloudColorsCurvature (),
      6,
      0.95
  );

  viewer->add<NormalCloudObject> (
      "normals",
      "vertex normals",
      "n",
      gv.getVerticesNormalsCloud (),
      1,
      0.01
  );

  viewer->add<PolyDataObject> (
      "edges",
      "adjacency edges",
      "a",
      gv.getEdgesPolyData ()
  );

  viewer->show ("edges");
  viewer->run ();

  return (0);
}

