#ifndef TVIEWER_POINT_CLOUD_OBJECT_HPP
#define TVIEWER_POINT_CLOUD_OBJECT_HPP

namespace tviewer
{

template <typename PointT>
void PointCloudObject<PointT>::add ()
{
  if (auto v = viewer_.lock())
  {
    v->addPointCloud (cloud_, name_);
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name_);
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, visibility_, name_);
    if (color_ != 0)
    {
      float r, g, b;
      std::tie (r, g, b) = getRGBFromColor (color_);
      v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name_);
    }
    visible_ = true;
  }
}

template <typename PointT>
void PointCloudObject<PointT>::remove ()
{
  if (auto v = viewer_.lock ())
  {
    v->removePointCloud (name_);
    visible_ = false;
  }
}

template <typename PointT>
void PointCloudObject<PointT>::update ()
{
  if (retrieve_function_)
  {
    setPointCloud (retrieve_function_ ());
  }

  if (visible_)
  {
    if (auto v = viewer_.lock ())
    {
      v->updatePointCloud (cloud_, name_);
    }
  }
}

template <typename PointT>
bool PointCloudObject<PointT>::at_ (size_t index, boost::any& item) const
{
  if (cloud_ && index < cloud_->size ())
  {
    item = cloud_->at (index);
    return true;
  }
  return false;
}

template <typename PointT>
void PointCloudObject<PointT>::setPointCloud (PointCloudPtr cloud)
{
  cloud_ = cloud;
}

}

#endif /* TVIEWER_POINT_CLOUD_OBJECT_HPP */

