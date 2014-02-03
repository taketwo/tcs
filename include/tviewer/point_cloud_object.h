#ifndef TVIEWER_POINT_CLOUD_OBJECT_H
#define TVIEWER_POINT_CLOUD_OBJECT_H

#include "color.h"
#include "visualization_object.h"

namespace tviewer
{

template <typename PointT>
class PointCloudObject : public VisualizationObject
{

  public:

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    typedef std::function<PointCloudPtr ()> RetrieveDataFunction;

    PointCloudObject (const std::string& name,
                      const std::string& description,
                      const std::string& key,
                      PointCloudPtr cloud,
                      int point_size = 1.0,
                      float visibility = 1.0,
                      Color color = 0)
    : VisualizationObject (name, description, key)
    , point_size_ (point_size)
    , visibility_ (visibility)
    , color_ (color)
    {
      setPointCloud (cloud);
    }

    PointCloudObject (const std::string& name,
                      const std::string& description,
                      const std::string& key,
                      const RetrieveDataFunction& retrieve_function,
                      int point_size = 1.0,
                      float visibility = 1.0,
                      Color color = 0)
    : VisualizationObject (name, description, key)
    , retrieve_function_ (retrieve_function)
    , point_size_ (point_size)
    , visibility_ (visibility)
    , color_ (color)
    {
      setPointCloud (PointCloudPtr (new PointCloud));
    }

    virtual void add ();

    virtual void remove ();

    virtual void update ();

    virtual bool at_ (size_t index, boost::any& item) const;

    virtual void setPointCloud (PointCloudPtr cloud);

    RetrieveDataFunction retrieve_function_;
    PointCloudPtr cloud_;

    int point_size_;
    float visibility_;
    Color color_;

};

}

#include "impl/point_cloud_object.hpp"

#endif /* TVIEWER_POINT_CLOUD_OBJECT_H */

