#ifndef GRAPH_VISUALIZER_H
#define GRAPH_VISUALIZER_H

#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>

#include "as_range.h"
#include "color.h"

#include "graph/pointcloud_adjacency_list.h"

template <typename GraphT>
class GraphVisualizer
{

  public:

    typedef typename pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;
    typedef typename pcl::PointCloud<pcl::PointNormal> NormalCloudT;

    typedef typename boost::graph_traits<GraphT>::vertex_iterator VertexIterator;
    typedef typename boost::graph_traits<GraphT>::edge_iterator   EdgeIterator;

    GraphVisualizer (const GraphT& g)
    : graph_ (g)
    {
    }

    vtkSmartPointer<vtkPolyData>
    getEdgesPolyData ()
    {
      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
      vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
      vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
      colors->SetNumberOfComponents (3);
      unsigned char c[3];
      EdgeIterator s, e;
      auto scale = getRangeScalingForEdges ();
      int id = 0;
      for (boost::tie (s, e) = boost::edges (graph_); s != e; ++s)
      {
        auto u = boost::source (*s, graph_);
        auto v = boost::target (*s, graph_);
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New ();
        points->InsertNextPoint (graph_[u].getVector3fMap ().data ());
        points->InsertNextPoint (graph_[v].getVector3fMap ().data ());
        line->GetPointIds ()->SetId (0, id++);
        line->GetPointIds ()->SetId (1, id++);
        cells->InsertNextCell (line);
        getRGBFromColor (getJetColor (scale (boost::get (boost::edge_weight_t (), graph_, *s))), c);
        colors->InsertNextTupleValue (c);
      }
      polydata->SetPoints (points);
      polydata->SetLines (cells);
      polydata->GetCellData ()->SetScalars (colors);
      return polydata;
    }

    typename NormalCloudT::Ptr
    getVerticesNormalsCloud ()
    {
      NormalCloudT::Ptr cloud (new NormalCloudT);
      copyPointCloud (*boost::get_pointcloud (graph_), *cloud);
      return cloud;
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsNatural ()
    {
      return constructVerticesCloud (MODE_NATURAL);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsCurvature ()
    {
      return constructVerticesCloud (MODE_CURVATURE);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsDegree ()
    {
      Eigen::VectorXf degrees = Eigen::VectorXf::Zero (boost::num_vertices (graph_));
      for (const auto& edge : as_range (boost::edges (graph_)))
      {
        auto src = boost::source (edge, graph_);
        auto tgt = boost::target (edge, graph_);
        auto degree = boost::get (boost::edge_weight_t (), graph_, edge);
        degrees (src) += degree;
        degrees (tgt) += degree;
      }
      return constructVerticesCloud (degrees);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromPropertyRandom ()
    {
      return constructVerticesCloud (MODE_FROM_PROPERTY_RANDOM);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromPropertyFixed ()
    {
      return constructVerticesCloud (MODE_FROM_PROPERTY_FIXED);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromPropertyPersistent ()
    {
      return constructVerticesCloud (MODE_FROM_PROPERTY_PERSISTENT);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromVector (const Eigen::VectorXf& colors)
    {
      return constructVerticesCloud (colors);
    }

  private:

    enum ColorMode

    {
      MODE_NATURAL,
      MODE_CURVATURE,
      MODE_FROM_PROPERTY_RANDOM,
      MODE_FROM_PROPERTY_FIXED,
      MODE_FROM_PROPERTY_PERSISTENT,
    };

    typename PointCloudT::Ptr
    constructVerticesCloud (ColorMode mode)
    {
      PointCloudT::Ptr cloud (new PointCloudT);
      copyPointCloud (*boost::get_pointcloud (graph_), *cloud);
      std::map<uint32_t, Color> colormap;
      const std::vector<Color> COLORS = { 0x9E9E9E, 0x29CC00, 0x008FCC, 0xA300CC, 0xCC3D00, 0xFFDD00, 0x63E6E6, 0xA5E663, 0x9E2B2B };
      size_t c = 0;
      for (const auto& s : as_range (boost::vertices (graph_)))
      {
        switch (mode)
        {
          case MODE_NATURAL:
            {
              break;
            }
          case MODE_CURVATURE:
            {
              // The curvature produced by PCAG is signed.
              // Empirically the absolute value of the curvature is below 0.25,
              // so it is safe (i.e. we end up in 0...1 region) to add 0.5.
              cloud->at (s).rgba = getJetColor (graph_[s].curvature + 0.5);
              break;
            }
          case MODE_FROM_PROPERTY_RANDOM:
            {
              auto color_id = boost::get (boost::vertex_color, graph_, s);
              if (!colormap.count (color_id))
                colormap[color_id] = generateRandomColor ();
              cloud->at (s).rgba = colormap[color_id];
              break;
            }
          case MODE_FROM_PROPERTY_FIXED:
            {
              auto color_id = boost::get (boost::vertex_color, graph_, s);
              if (!colormap.count (color_id))
                colormap[color_id] = COLORS[c++];
              cloud->at (s).rgba = colormap[color_id];
              break;
            }
          case MODE_FROM_PROPERTY_PERSISTENT:
            {
              auto color_id = boost::get (boost::vertex_color, graph_, s);
              if (!colormap_.count (color_id))
                colormap_[color_id] = generateRandomColor ();
              cloud->at (s).rgba = colormap_[color_id];
              break;
            }
        }
      }
      return cloud;
    }

    typename PointCloudT::Ptr
    constructVerticesCloud (const Eigen::VectorXf& colors)
    {
      assert (static_cast<int> (boost::num_vertices (graph_)) == colors.size ());
      PointCloudT::Ptr cloud (new PointCloudT);
      copyPointCloud (*boost::get_pointcloud (graph_), *cloud);
      auto scale = getRangeScalingForVector (colors);
      for (const auto& s : as_range (boost::vertices (graph_)))
      {
        cloud->at (s).rgba = getJetColor (scale (colors[s]));
      }
      return cloud;
    }

    std::function<float (float)> getRangeScalingForEdges ()
    {
      float max = -std::numeric_limits<float>::infinity();
      float min = std::numeric_limits<float>::infinity();
      for (const auto& s : as_range (boost::edges (graph_)))
      {
        auto v = boost::get (boost::edge_weight_t (), graph_, s);
        if (v > max) max = v;
        if (v < min) min = v;
      }
      if (max != min)
        return [min, max] (float v) { return (v - min) / (max - min); };
      else
        return [] (float v) { return 1.0; };
    }

    std::function<float (float)> getRangeScalingForVector (const Eigen::VectorXf& values)
    {
      float max = values.maxCoeff ();
      float min = values.minCoeff ();
      float range = max - min;
      return [min, range] (float v) { return (v - min) / range; };
    }

    const GraphT& graph_;
    std::map<uint32_t, Color> colormap_;

};

#endif /* GRAPH_VISUALIZER_H */

