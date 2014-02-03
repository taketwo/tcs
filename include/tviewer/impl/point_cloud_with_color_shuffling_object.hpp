#ifndef TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_HPP
#define TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_HPP

#include "tviewer/utils.h"

namespace tviewer
{

template <typename PointT>
bool PointCloudWithColorShufflingObject<PointT>::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, "C-" + key_))
  {
    this->shuffleColors ();
    return true;
  }
  if (matchKeys (key_event, key_))
  {
    this->toggle ();
    return true;
  }
  return false;
}

template <typename PointT>
bool PointCloudWithColorShufflingObject<PointT>::at_ (size_t index, boost::any& item) const
{
  if (cloud_ && index < cloud_->size ())
  {
    PointT pt = cloud_->at (index);
    pt.rgba = color_map_.at (pt.rgba);
    item = pt;
    return true;
  }
  return false;
}

template <typename PointT>
void PointCloudWithColorShufflingObject<PointT>::shuffleColors ()
{
  if (!cloud_ || !cloud_->size ())
    return;

  std::cout << name_ << ": shuffling colors\n";

  // Generate new random colors
  std::map<Color, Color> new_colors;
  for (const auto& current_original_pair : color_map_)
    new_colors[current_original_pair.first] = generateRandomColor ();

  // Apply new colors
  for (auto& point : cloud_->points)
    point.rgba = new_colors[point.rgba];

  // Update color map
  std::map<Color, Color> new_color_map;
  for (const auto& current_original_pair : color_map_)
    new_color_map.insert (std::make_pair (new_colors[current_original_pair.first], current_original_pair.second));
  color_map_.swap (new_color_map);

  if (visible_)
    if (auto v = this->viewer_.lock())
      v->updatePointCloud (cloud_, name_);
}

template <typename PointT>
void PointCloudWithColorShufflingObject<PointT>::setPointCloud (PointCloudPtr cloud)
{
  cloud_ = cloud;
  color_map_.clear ();

  if (!cloud_ || !cloud_->size ())
    return;

  for (const auto& point : cloud_->points)
    if (!color_map_.count (point.rgba))
      color_map_[point.rgba] = point.rgba;
}

}

#endif /* TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_HPP */

