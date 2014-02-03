#ifndef TVIEWER_TVIEWER_H
#define TVIEWER_TVIEWER_H

#include <memory>

#include <boost/optional.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "visualization_object.h"
#include "point_cloud_object.h"
#include "point_cloud_with_color_shuffling_object.h"
#include "normal_cloud_object.h"
#include "poly_data_object.h"

#include "tviewer_interface.h"
#include "utils.h"

namespace tviewer
{

class TViewer : public TViewerInterface
{

public:

  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef pcl::PointNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointNCloud;
  typedef typename pcl::PointCloud<PointNT>::Ptr PointNCloudPtr;

  TViewer ();

  ~TViewer () { };

  virtual void run ();

  virtual void sleep (size_t milliseconds);

  /// Print question and wait until the user presses 'y' or 'n'.
  virtual bool askYesNo (const std::string& question, bool no_with_any_key = true);

  /// Wait for any key.
  virtual bool waitKeyPressed ();

  /// Wait for any key and return it.
  virtual bool waitKeyPressed (std::string& key);

  /// Wait for certain keys.
  virtual bool waitKeyPressed (const std::vector<std::string>& keys);

  /// Wait for certain keys and return it.
  virtual bool waitKeyPressed (std::string& key, const std::vector<std::string>& keys);

  /// Wait for a point to be selected and return its index and coordinates.
  virtual bool waitPointSelected (size_t& point_index, float& x, float& y, float& z);

  /// Wait for a point to be selected and return only its index.
  virtual bool waitPointSelected (size_t& point_index);

  /// Wait for a point to be selected and return its color.
  /// Note: if the point belongs to an object with color shuffling capability,
  /// then the original color (prior to user shuffling, if any) will be returned.
  virtual bool waitPointSelected (Color& point_color);

  /// Wait for set of points to be selected and return them as both:
  /// - a point cloud with labels
  /// - a vector of vectors of point indices (in the original cloud)
  /// When only one type of output is desired, use corresponding specialized version.
  virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                           std::vector<pcl::PointIndices>& indices,
                           bool skip_duplicates = true);

  /// Wait for set of points to be selected and return them as a point cloud.
  virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud, bool skip_duplicates = true);

  /// Wait for set of points to be selected and return them as a vector of vectors of indices.
  virtual bool waitPointsSelected (std::vector<pcl::PointIndices>& indices, bool skip_duplicates = true);

  virtual void saveCameraParameters (const std::string& filename);

  virtual void loadCameraParameters (const std::string& filename);

  template <typename VisualizationObjectT, typename... _Args> inline void
  registerVisualizationObject (_Args&&... __args)
  {
    registerVisualizationObject (std::make_shared<VisualizationObjectT> (std::forward<_Args> (__args)...));
  }

  virtual void
  registerVisualizationObject (VisualizationObject::Ptr object);

  virtual void
  deregisterVisualizationObject (const std::string& object_name);

  virtual void
  showVisualizationObject (const std::string& object_name);

  virtual void
  showVisualizationObjects ();

  virtual void
  hideVisualizationObject (const std::string& object_name);

  virtual void
  hideVisualizationObjects ();

  virtual void
  updateVisualizationObject (const std::string& object_name);

  virtual void
  updateVisualizationObjects ();

private:

  void keyboardEventCallback (const pcl::visualization::KeyboardEvent& event);

  void pickPointEventCallback (const pcl::visualization::PointPickingEvent& event);

  void dispatch (const pcl::visualization::KeyboardEvent& key_event);

  std::string getHelp ();

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

  typedef boost::optional<pcl::visualization::KeyboardEvent> OptionalKeyboardEvent;
  typedef boost::optional<pcl::visualization::PointPickingEvent> OptionalPointPickingEvent;

  OptionalKeyboardEvent last_keyboard_event_;
  OptionalPointPickingEvent last_point_picking_event_;

  std::vector<VisualizationObject::Ptr> objects_;

  bool mode_waiting_user_input_;

};

}

#endif /* TVIEWER_TVIEWER_H */

