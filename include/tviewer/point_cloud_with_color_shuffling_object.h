#ifndef TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H
#define TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H

#include "point_cloud_object.h"

namespace tviewer
{

template <typename PointT>
class PointCloudWithColorShufflingObject : public PointCloudObject<PointT>
{

  public:

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    typedef std::function<PointCloudPtr ()> RetrieveDataFunction;

    PointCloudWithColorShufflingObject (const std::string& name,
                                        const std::string& description,
                                        const std::string& key,
                                        PointCloudPtr cloud,
                                        int point_size = 1.0,
                                        float visibility = 1.0,
                                        Color color = 0)
    : PointCloudObject<PointT> (name, description, key, cloud, point_size, visibility, color)
    {
      setPointCloud (cloud);
    }

    PointCloudWithColorShufflingObject (const std::string& name,
                                        const std::string& description,
                                        const std::string& key,
                                        const RetrieveDataFunction& retrieve_function,
                                        int point_size = 1.0,
                                        float visibility = 1.0,
                                        Color color = 0)
    : PointCloudObject<PointT> (name, description, key, retrieve_function, point_size, visibility, color)
    {
    }

    virtual bool execute (const pcl::visualization::KeyboardEvent& key_event);

    virtual bool at_ (size_t index, boost::any& item) const;

    virtual void setPointCloud (PointCloudPtr cloud);

    void shuffleColors ();

    using VisualizationObject::key_;
    using VisualizationObject::name_;
    using VisualizationObject::visible_;
    using PointCloudObject<PointT>::cloud_;

    // Maps current color to the original color.
    std::map<Color, Color> color_map_;

};

}

#include "impl/point_cloud_with_color_shuffling_object.hpp"

#endif /* TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H */

