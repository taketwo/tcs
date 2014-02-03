#include "normal_cloud_object.h"

namespace tviewer
{

void NormalCloudObject::add ()
{
  if (auto v = viewer_.lock())
  {
    v->addPointCloudNormals<pcl::PointNormal> (cloud_, level_, scale_, name_);
    visible_ = true;
  }
}

void NormalCloudObject::remove ()
{
  if (auto v = viewer_.lock())
  {
    v->removePointCloud (name_);
    visible_ = false;
  }
}

void NormalCloudObject::update ()
{
  if (retrieve_function_)
  {
    cloud_ = retrieve_function_();
  }

  if (visible_)
  {
    if (auto v = viewer_.lock())
    {
      v->removePointCloud (name_);
      v->addPointCloudNormals<pcl::PointNormal> (cloud_, level_, scale_, name_);
    }
  }
}

}

