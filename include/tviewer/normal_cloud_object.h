#ifndef TVIEWER_NORMAL_CLOUD_OBJECT_H
#define TVIEWER_NORMAL_CLOUD_OBJECT_H

#include "visualization_object.h"

namespace tviewer
{

class NormalCloudObject : public VisualizationObject
{

  public:

    typedef pcl::PointCloud<pcl::PointNormal> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;

    typedef std::function<NormalCloudPtr ()> RetrieveDataFunction;

    NormalCloudObject (const std::string& name,
                       const std::string& description,
                       const std::string& key,
                       NormalCloudPtr cloud,
                       int level = 100,
                       float scale = 0.02)
    : VisualizationObject (name, description, key)
    , cloud_ (cloud)
    , level_ (level)
    , scale_ (scale)
    {
    }

    NormalCloudObject (const std::string& name,
                       const std::string& description,
                       const std::string& key,
                       const RetrieveDataFunction& retrieve_function,
                       int level = 100,
                       float scale = 0.02)
    : VisualizationObject (name, description, key)
    , retrieve_function_ (retrieve_function)
    , cloud_ (new NormalCloud)
    , level_ (level)
    , scale_ (scale)
    {
    }

    virtual void add ();

    virtual void remove ();

    virtual void update ();

    RetrieveDataFunction retrieve_function_;
    NormalCloudPtr cloud_;

    int level_;
    float scale_;

};

}

#endif /* TVIEWER_NORMAL_CLOUD_OBJECT_H */

