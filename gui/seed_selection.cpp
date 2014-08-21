#include "seed_selection.h"

#include <pcl/common/utils.h>
#include <pcl/console/print.h>
#include <pcl/common/io.h>

SeedSelection::SeedSelection (QObject* parent)
: QAbstractListModel (parent)
, seeds_cloud_ (new LabeledPointCloud)
, num_labels_ (0)
, current_label_ (0)
{
}

SeedSelection::~SeedSelection ()
{
}

int
SeedSelection::rowCount (const QModelIndex&) const
{
  return num_labels_;
}

QVariant
SeedSelection::data (const QModelIndex& index, int role) const
{
  if (role == Qt::DisplayRole)
  {
    uint32_t label = index.row () + 1;
    return QString ("%1    %2 seeds").arg (label).arg (countSeedsWithLabel (label));
  }
  return QVariant ();
}

void
SeedSelection::pickPoint (const pcl::PointXYZ& p)
{
  using namespace pcl::utils;
  for (size_t i = 0; i < seeds_cloud_->size (); ++i)
  {
    pcl::PointXYZL& pt = seeds_cloud_->points[i];
    if (equal (p.x, pt.x) && equal (p.y, pt.y) && equal (p.z, pt.z))
    {
      if (pt.label == current_label_)
      {
        LabeledPointCloudPtr new_seeds (new LabeledPointCloud);
        for (size_t j = 0; j < seeds_cloud_->size (); ++j)
          if (j != i)
            new_seeds->push_back (seeds_cloud_->at (j));
        seeds_cloud_.swap (new_seeds);
        seedsChanged ();
        dataChanged (QModelIndex (), QModelIndex ());
        return;
      }
      else
      {
        pt.label = current_label_;
        seedsChanged ();
        dataChanged (QModelIndex (), QModelIndex ());
        return;
      }
    }
  }

  pcl::PointXYZL pt;
  pt.x = p.x, pt.y = p.y, pt.z = p.z, pt.label = current_label_;
  seeds_cloud_->push_back (pt);
  seedsChanged ();
  dataChanged (QModelIndex (), QModelIndex ());
}

QModelIndex
SeedSelection::addNewLabel ()
{
  ++num_labels_;
  current_label_ = num_labels_;
  dataChanged (QModelIndex (), QModelIndex ());
  return index (current_label_ - 1, 0);
}

void
SeedSelection::deleteLabel ()
{
  if (num_labels_ == 1)
    return;

  LabeledPointCloudPtr new_seeds (new LabeledPointCloud);
  for (size_t i = 0; i < seeds_cloud_->size (); ++i)
    if (seeds_cloud_->at (i).label != current_label_)
    {
      new_seeds->push_back (seeds_cloud_->at (i));
      if (seeds_cloud_->at (i).label > current_label_)
        --new_seeds->back ().label;
    }

  --num_labels_;
  seeds_cloud_.swap (new_seeds);
  dataChanged (QModelIndex (), QModelIndex ());
  seedsChanged ();
}

void
SeedSelection::setSeeds (const LabeledPointCloud& seeds)
{
  pcl::copyPointCloud (seeds, *seeds_cloud_);
  num_labels_ = 0;
  for (size_t i = 0; i < seeds.size (); ++i)
    if (seeds[i].label > num_labels_)
      num_labels_ = seeds[i].label;
  current_label_ = num_labels_;
  dataChanged (QModelIndex (), QModelIndex ());
  seedsChanged ();
}

SeedSelection::ColoredPointCloudPtr
SeedSelection::getPointCloudForVisualization ()
{
  ColoredPointCloudPtr cloud (new ColoredPointCloud);
  pcl::copyPointCloud (*seeds_cloud_, *cloud);
  for (size_t i = 0; i < cloud->size (); ++i)
    if (seeds_cloud_->at (i).label == current_label_)
      cloud->at (i).rgb = 0xFFFFFF;
    else
      cloud->at (i).rgb = 0xFF0000;
  return cloud;
}

void
SeedSelection::currentChanged (const QItemSelection& current, const QItemSelection& previous)
{
  current_label_ = current.indexes ().back ().row () + 1;
  seedsChanged ();
}

size_t
SeedSelection::countSeedsWithLabel (uint32_t label) const
{
  size_t count = 0;
  for (size_t i = 0; i < seeds_cloud_->size (); ++i)
    if (seeds_cloud_->points[i].label == label)
      ++count;
  return count;
}

