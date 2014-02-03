#ifndef TVIEWER_TVIEWER_INTERFACE_H
#define TVIEWER_TVIEWER_INTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "color.h"
#include "visualization_object.h"

namespace tviewer
{

class TViewerInterface
{

public:

  TViewerInterface () { }

  virtual ~TViewerInterface () { }

  virtual void run () { }

  virtual void sleep (size_t milliseconds) { }

  virtual bool askYesNo (const std::string& question, bool no_with_any_key = true) { return false; }

  virtual bool waitKeyPressed () { return true; }

  virtual bool waitKeyPressed (std::string& key) { return true; }

  virtual bool waitKeyPressed (const std::vector<std::string>& keys) { return true; }

  virtual bool waitKeyPressed (std::string& key, const std::vector<std::string>& keys) { return true; }

  virtual bool waitPointSelected (size_t& point_index, float& x, float& y, float& z) { return true; }

  virtual bool waitPointSelected (size_t& point_index) { return true; }

  virtual bool waitPointSelected (Color& point_color) { return true; }

  virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                   std::vector<pcl::PointIndices>& indices,
                                   bool skip_duplicates = true) { return true; }

  virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud, bool skip_duplicates = true) { return true; }

  virtual bool waitPointsSelected (std::vector<pcl::PointIndices>& indices, bool skip_duplicates = true) { return true; }

  virtual void saveCameraParameters (const std::string& filename) { }

  virtual void loadCameraParameters (const std::string& filename) { };

  template <typename VisualizationObjectT, typename... _Args> inline void
  registerVisualizationObject (_Args&&... __args)
  {
    registerVisualizationObject (std::make_shared<VisualizationObjectT> (std::forward<_Args> (__args)...));
  }

  virtual void registerVisualizationObject (VisualizationObject::Ptr object) { }

  virtual void deregisterVisualizationObject (const std::string& object_name) { }

  virtual void showVisualizationObject (const std::string& object_name) { }

  virtual void showVisualizationObjects () { }

  virtual void hideVisualizationObject (const std::string& object_name) { }

  virtual void hideVisualizationObjects () { }

  virtual void updateVisualizationObject (const std::string& object_name) { }

  virtual void updateVisualizationObjects () { }

};

}

#endif /* TVIEWER_TVIEWER_H */

