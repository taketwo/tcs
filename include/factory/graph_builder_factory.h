#ifndef FACTORY_GRAPH_BUILDER_FACTORY_H
#define FACTORY_GRAPH_BUILDER_FACTORY_H

#include "factory.h"
#include "graph/graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"
#include "graph/octree_adjacency_graph_builder.h"

namespace factory
{

template <typename PointT, class GraphT>
class GraphBuilderFactory : public Factory
{

public:

  typedef pcl::graph::GraphBuilder<PointT, GraphT> GraphBuilderT;

  GraphBuilderFactory ()
  : Factory ("Graph Builder")
  , builder_ ("builder type", "--builder", { { "octree", "OCTREE ADJACENCY" },
                                             { "nn", "NEAREST NEIGHBORS"} })
  , voxel_resolution_ ("voxel resolution", "-v", 0.006f)
  , number_of_neighbors_ ("number of neighbors", "--nn", 14)
  , no_transform_ ("no transform", "-nt")
  {
    add (&builder_);
    add (&voxel_resolution_);
    add (&number_of_neighbors_);
    add (&no_transform_);
  }

  typename GraphBuilderT::Ptr
  instantiate (int argc, char** argv)
  {
    parse (argc, argv);
    typename GraphBuilderT::Ptr gb;
    if (builder_.value == "octree")
      gb.reset (new pcl::graph::OctreeAdjacencyGraphBuilder<PointT, GraphT> (voxel_resolution_, no_transform_));
    else
      gb.reset (new pcl::graph::NearestNeighborsGraphBuilder<PointT, GraphT> (number_of_neighbors_));
    return gb;
  }

private:

  EnumOption builder_;
  NumericOption<float> voxel_resolution_;
  NumericOption<int> number_of_neighbors_;
  BoolOption no_transform_;

};

}

#endif /* FACTORY_GRAPH_BUILDER_FACTORY_H */

