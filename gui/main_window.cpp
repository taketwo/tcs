#include <ctime>

#include <QModelIndex>

#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

#include "tviewer/color.h"

#include "main_window.h"
#include "ui_main_window.h"

#include "graph/common.h"
#include "graph/edge_weight_computer.h"
#include "graph/voxel_grid_graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"

using namespace tviewer;

MainWindow::MainWindow (const std::string& filename, QWidget* parent)
: QMainWindow (parent)
, ui_ (new Ui::MainWindow)
, graph_ (new Graph)
, seed_selection_ (new SeedSelection)
{
  srand (time (0));

  ui_->setupUi (this);
  viewer_.reset (new pcl::visualization::PCLVisualizer ("PCL Visualizer", false));
  ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (),
                            ui_->qvtkWidget->GetRenderWindow ());

  viewer_->registerPointPickingCallback (&MainWindow::pointPickingCallback, *this);

  cloud_.reset (new PointCloudT);
  if (pcl::io::loadPCDFile (filename, *cloud_))
    throw std::runtime_error ("failed to load input point cloud");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
  viewer_->addPointCloud (tmp, "vertices");

  viewer_->addPointCloud (tmp, "seeds");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             5, "seeds");

  ui_->qvtkWidget->update ();

  ui_->list_labels->setModel (seed_selection_.get ());

  connect (ui_->list_labels->selectionModel (),
           SIGNAL (selectionChanged (QItemSelection, QItemSelection)),
           seed_selection_.get (),
           SLOT (currentChanged (QItemSelection, QItemSelection)));

  connect (seed_selection_.get (),
           SIGNAL (seedsChanged ()),
           this,
           SLOT (seedsChanged ()));

  QModelIndex index = seed_selection_->addNewLabel ();
  ui_->list_labels->selectionModel ()->select (index, QItemSelectionModel::ClearAndSelect);

  onButtonUpdateVoxelsClicked ();
}

MainWindow::~MainWindow ()
{
  delete ui_;
}

void
MainWindow::seedsChanged ()
{
  displaySeeds ();
}

void
MainWindow::onButtonUpdateVoxelsClicked ()
{
  double r = ui_->spinbox_voxel_resolution->value ();
  pcl::graph::VoxelGridGraphBuilder<PointT, Graph> graph_builder (r);
  graph_builder.setInputCloud (cloud_);
  graph_builder.compute (*graph_);

  pcl::graph::computeNormalsAndCurvatures (*graph_);
  pcl::graph::computeSignedCurvatures (*graph_);
  computeEdgeWeights ();

  displayGraphVertices ();
  displayGraphEdges ();
}

void
MainWindow::onButtonUpdateNeighborsClicked ()
{
  int n = ui_->spinbox_nearest_neighbors->value ();
  pcl::graph::NearestNeighborsGraphBuilder<PointT, Graph> graph_builder;
  graph_builder.setNumberOfNeighbors (n);
  graph_builder.setInputCloud (cloud_);
  graph_builder.compute (*graph_);

  pcl::graph::computeNormalsAndCurvatures (*graph_);
  pcl::graph::computeSignedCurvatures (*graph_);
  computeEdgeWeights ();

  displayGraphVertices ();
  displayGraphEdges ();
}

void
MainWindow::buttonNewLabelClicked ()
{
  QModelIndex index = seed_selection_->addNewLabel ();
  ui_->list_labels->selectionModel ()->select (index, QItemSelectionModel::ClearAndSelect);
}

void
MainWindow::buttonDeleteLabelClicked ()
{
  seed_selection_->deleteLabel ();
}

void
MainWindow::buttonSegmentClicked ()
{
  pcl::segmentation::RandomWalkerSegmentation<pcl::PointXYZRGB> rws;
  rws.setInputGraph (graph_);
  rws.setSeeds (seed_selection_->getSelectedSeeds ());
  std::vector<pcl::PointIndices> clusters;
  rws.segment (clusters);
  displayGraphVertices (false);
}

void
MainWindow::pointPickingCallback (const pcl::visualization::PointPickingEvent& event, void*)
{
  int idx = event.getPointIndex ();
  if (idx == -1)
    return;

  pcl::PointXYZ p;
  event.getPoint (p.x, p.y, p.z);

  seed_selection_->pickPoint (p);
}

void
MainWindow::displayGraphVertices (bool how)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (ui_->checkbox_graph_vertices->checkState ())
  {
    pcl::copyPointCloud (*pcl::graph::point_cloud (*graph_), *vertices);
    boost::get (boost::vertex_color, *graph_);
    if (how == false)
      for (size_t i = 0; i < vertices->size (); ++i)
      {
        uint32_t label = boost::get (boost::vertex_color, *graph_, i);
        if (!colormap_.count (label))
          colormap_[label] = generateRandomColor ();
        vertices->at (i).rgba = colormap_[label];
      }
  }
  viewer_->updatePointCloud (vertices, "vertices");
  ui_->qvtkWidget->update ();
}

void
MainWindow::displayGraphEdges (uint32_t color)
{
  viewer_->removeShape ("edges");
  if (ui_->checkbox_graph_edges->checkState ())
  {
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    unsigned char c[3];
    boost::graph_traits<Graph>::edge_iterator s, e;
    int id = 0;
    for (boost::tie (s, e) = boost::edges (*graph_); s != e; ++s)
    {
      vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New ();
      points->InsertNextPoint ((*graph_)[boost::source (*s, *graph_)].getVector3fMap ().data ());
      points->InsertNextPoint ((*graph_)[boost::target (*s, *graph_)].getVector3fMap ().data ());
      line->GetPointIds ()->SetId (0, id++);
      line->GetPointIds ()->SetId (1, id++);
      cells->InsertNextCell (line);
      getRGBFromColor (getColor (boost::get (boost::edge_weight_t (), *graph_, *s)), c);
      colors->InsertNextTupleValue (c);
    }
    polydata->SetPoints (points);
    polydata->SetLines (cells);
    polydata->GetCellData ()->SetScalars (colors);
    viewer_->addModelFromPolyData (polydata, "edges");
  }
}

void
MainWindow::displaySeeds ()
{
  viewer_->updatePointCloud (seed_selection_->getPointCloudForVisualization (), "seeds");
  ui_->qvtkWidget->update ();
}

void
MainWindow::computeEdgeWeights ()
{
  using namespace pcl::graph;
  typedef EdgeWeightComputer<Graph> EWC;
  EWC computer;
  if (ui_->checkbox_xyz->checkState ())
  {
    float influence = ui_->spinbox_xyz_influence->value ();
    float multiplier = ui_->checkbox_xyz_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::XYZ> (influence, multiplier, EWC::NORMALIZATION_LOCAL);
  }
  if (ui_->checkbox_normal->checkState ())
  {
    float influence = ui_->spinbox_normal_influence->value ();
    float multiplier = ui_->checkbox_normal_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::Normal> (influence, multiplier);
  }
  if (ui_->checkbox_curvature->checkState ())
  {
    float influence = ui_->spinbox_curvature_influence->value ();
    float multiplier = ui_->checkbox_curvature_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::Curvature> (influence, multiplier);
  }
  if (ui_->checkbox_rgb->checkState ())
  {
    float influence = ui_->spinbox_rgb_influence->value ();
    float multiplier = ui_->checkbox_rgb_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::RGB> (influence, multiplier, EWC::NORMALIZATION_GLOBAL);
  }
  computer.setSmallWeightThreshold (1e-5);
  computer.setSmallWeightPolicy (EWC::SMALL_WEIGHT_COERCE_TO_THRESHOLD);
  computer.compute (*graph_);
}

