#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <map>

#include <QMainWindow>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "seed_selection.h"
#include "random_walker_segmentation.h"

namespace Ui
{
  class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointXYZRGBNormal PointWithNormalT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointWithNormalT> PointCloudWithNormalT;

    typedef pcl::segmentation::RandomWalkerSegmentation<PointWithNormalT> RandomWalkerSegmentation;
    typedef RandomWalkerSegmentation::Graph Graph;
    typedef RandomWalkerSegmentation::GraphPtr GraphPtr;
    typedef RandomWalkerSegmentation::GraphConstPtr GraphConstPtr;

    MainWindow (const std::string& filename, QWidget* parent = 0);

    ~MainWindow ();

  public Q_SLOTS:

    void
    onButtonUpdateGraphClicked ();

    void
    buttonNewLabelClicked ();

    void
    buttonDeleteLabelClicked ();

    void
    buttonSegmentClicked ();

    void
    checkboxDisplayStateChanged (int state)
    {
      displayGraphVertices ();
      displayGraphEdges ();
    }

    void
    onActionExitTriggered ()
    {
      this->close ();
    }

    void
    onActionLoadViewpointTriggered ()
    {
      if (boost::filesystem::exists ("viewpoint.cam"))
        viewer_->loadCameraParameters ("viewpoint.cam");
    }

    void
    onActionSaveViewpointTriggered ()
    {
      viewer_->saveCameraParameters ("viewpoint.cam");
    }

    void
    onActionLoadSeedsTriggered ()
    {
      if (boost::filesystem::exists ("seeds.pcd"))
      {
        pcl::PointCloud<pcl::PointXYZL> seeds;
        pcl::io::loadPCDFile<pcl::PointXYZL> ("seeds.pcd", seeds);
        seed_selection_->setSeeds (seeds);
      }
    }

    void
    onActionSaveSeedsTriggered ()
    {
      pcl::io::savePCDFile ("seeds.pcd", *seed_selection_->getSelectedSeeds ());
    }

    void
    onActionViewToggled (bool checked)
    {
      displayGraphVertices ();
      displayGraphEdges ();
    }

    void
    onActionSaveSegmentationTriggered ();

    void
    seedsChanged ();

    void
    onKeyUp ();

    void
    onKeyDown ();

  private:

    void
    pointPickingCallback (const pcl::visualization::PointPickingEvent& event, void*);

    void
    displayGraphVertices ();

    void
    displayGraphEdges (uint32_t color = 0);

    void
    displaySeeds ();

    void
    buildGraph (pcl::graph::GraphBuilder<PointT, Graph>::Ptr graph_builder);

    void
    computeEdgeWeights ();

    void
    saveConfig ();

    void
    loadConfig ();

    enum GlobalState
    {
      GS_NOT_SEGMENTED,
      GS_SEGMENTED,
    };

    void
    setGlobalState (GlobalState state);

    Ui::MainWindow* ui_;

    pcl::visualization::PCLVisualizer::Ptr viewer_;

    PointCloudT::Ptr cloud_;
    GraphPtr graph_;
    SeedSelection::Ptr seed_selection_;
    std::map<uint32_t, uint32_t> colormap_;

    GlobalState state_;

};

#endif // MAIN_WINDOW_H
