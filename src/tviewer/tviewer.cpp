#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <sys/stat.h>

#include <boost/none.hpp>
#include <boost/format.hpp>

#include <pcl/common/utils.h>
#include <pcl/console/print.h>

#include "tviewer.h"

namespace tviewer
{

TViewer::TViewer ()
: viewer_ (new pcl::visualization::PCLVisualizer ("T Viewer"))
, mode_waiting_user_input_ (false)
{
  viewer_->setBackgroundColor (0, 0, 0);
  viewer_->registerKeyboardCallback (boost::bind(&TViewer::keyboardEventCallback, this, _1));
  viewer_->registerPointPickingCallback (boost::bind(&TViewer::pickPointEventCallback, this, _1));
  loadCameraParameters ("viewpoint.cam");
}

void TViewer::run ()
{
  while (!viewer_->wasStopped ())
  {
    viewer_->spinOnce (100);
  }
}

void TViewer::sleep (size_t milliseconds)
{
  viewer_->spinOnce (100);
  std::this_thread::sleep_for (std::chrono::milliseconds (milliseconds));
}

bool TViewer::askYesNo (const std::string& question, bool no_with_any_key)
{
  mode_waiting_user_input_ = true;
  pcl::console::print_info (question.c_str ());
  pcl::console::print_value (" (y/n) ");
  std::cout << std::flush;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyUp ())
    {
      std::string key = last_keyboard_event_->getKeySym ();
      last_keyboard_event_ = boost::none;
      if (key == "y" || key == "Y")
      {
        pcl::console::print_error ("YES\n");
        return true;
      }
      if (no_with_any_key || key == "n" || key == "N")
      {
        pcl::console::print_error ("NO\n");
        return false;
      }
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitKeyPressed ()
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
    {
      last_keyboard_event_ = boost::none;
      return true;
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitKeyPressed (std::string& key)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
    {
      key = last_keyboard_event_->getKeySym ();
      last_keyboard_event_ = boost::none;
      return true;
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitKeyPressed (const std::vector<std::string>& keys)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
      for (const auto& k : keys)
        if (matchKeys (*last_keyboard_event_, k))
        {
          last_keyboard_event_ = boost::none;
          return true;
        }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitKeyPressed (std::string& key, const std::vector<std::string>& keys)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
      for (const auto& k : keys)
        if (matchKeys (*last_keyboard_event_, k))
        {
          key = k;
          last_keyboard_event_ = boost::none;
          return true;
        }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitPointSelected (size_t& point_index, float& x, float& y, float& z)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      point_index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_->getPoint (x, y, z);
      last_point_picking_event_ = boost::none;
      return true;
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitPointSelected (size_t& point_index)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      point_index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_ = boost::none;
      return true;
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitPointSelected (Color& point_color)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      size_t index;
      float x, y, z;
      index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_->getPoint (x, y, z);
      last_point_picking_event_ = boost::none;
      std::cout << "Point picked" << x << y<< z<< " " << index << "\n";
      // Iterate over visualization objects and find which one owns the point
      for (const auto& object : objects_)
      {
        if (!object->visible_)
          continue;
        std::cout << "visible object " << object->name_ << "\n";
        pcl::PointXYZRGBA pt;
        if (object->at (index, pt))  // will return false if object is not a point cloud
        {
          std::cout << "got something at this index\n";
          using namespace pcl::utils;
          if (equal (x, pt.x) && equal (y, pt.y) && equal (z, pt.z))
          {
            point_color = pt.rgba;
            return true;
          }
        }
      }
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool TViewer::waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  std::vector<pcl::PointIndices>& indices,
                                  bool skip_duplicates)
{
  cloud.clear ();
  indices.clear ();
  // Set up point cloud display
  PointCloudPtr all_points (new PointCloud);
  const std::string id = "point_picker_display";
  registerVisualizationObject<PointCloudObject<PointT>> (
        id,
        "User selected points",
        "u",
        all_points,
        8,
        0.9
  );
  showVisualizationObject (id);

  pcl::console::print_info ("Please select points:\n");

  // User input
  size_t label = 0;
  while (true)
  {
    size_t num_points = 0;
    pcl::PointIndices indexes;
    Color color = generateRandomColor ();
    mode_waiting_user_input_ = true;
    while (!viewer_->wasStopped () && mode_waiting_user_input_)
    {
      if (last_point_picking_event_)
      {
        pcl::PointXYZL pt;
        int index = last_point_picking_event_->getPointIndex ();
        last_point_picking_event_->getPoint (pt.x, pt.y, pt.z);
        last_point_picking_event_ = boost::none;
        if (skip_duplicates)
        {
          using namespace pcl::utils;
          bool duplicate = false;
          for (const auto& p : cloud)
          {
            if (equal (p.x, pt.x) && equal (p.y, pt.y) && equal (p.z, pt.z))
            {
              duplicate = true;
              break;
            }
          }
          if (duplicate)
            continue;
        }
        pt.label = label;
        cloud.push_back (pt);
        indexes.indices.push_back (index);
        ++num_points;
        // Visualize the selected point
        PointT vpoint;
        vpoint.x = pt.x;
        vpoint.y = pt.y;
        vpoint.z = pt.z;
        vpoint.rgba = color;
        all_points->push_back (vpoint);
        updateVisualizationObject (id);
      }
      viewer_->spinOnce (100);
    }
    // If the user did not input any points and pressed "Escape", which means
    // it is time to shut things down
    if (!num_points)
      break;
    pcl::console::print_info ("  [%zu] %zu points\n", label, num_points);
    // Now waiting for the next group of points
    ++label;
    indices.push_back (indexes);
  }
  deregisterVisualizationObject (id);
  return true;
}

bool TViewer::waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud, bool skip_duplicates)
{
  std::vector<pcl::PointIndices> indices;
  return waitPointsSelected (cloud, indices, skip_duplicates);
}

bool TViewer::waitPointsSelected (std::vector<pcl::PointIndices>& indices, bool skip_duplicates)
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  return waitPointsSelected (cloud, indices, skip_duplicates);
}

void TViewer::saveCameraParameters (const std::string& filename)
{
  std::vector<pcl::visualization::Camera> cameras;
  viewer_->getCameras (cameras);
  std::ofstream file (filename);
  if (file.is_open ())
  {
    const auto& clip = cameras[0].clip;
    const auto& focal = cameras[0].focal;
    const auto& pos = cameras[0].pos;
    const auto& view = cameras[0].view;
    const auto& fovy = cameras[0].fovy;
    const auto& win_size = cameras[0].window_size;
    const auto& win_pos = cameras[0].window_pos;
    file << clip[0]     << "," << clip[1]     << "/"
         << focal[0]    << "," << focal[1]    << "," << focal[2] << "/"
         << pos[0]      << "," << pos[1]      << "," << pos[2]   << "/"
         << view[0]     << "," << view[1]     << "," << view[2]  << "/"
         << fovy        << "/"
         << win_size[0] << "," << win_size[1] << "/"
         << win_pos[0]  << "," << win_pos[1]
         << endl;
    file.close ();
  }
  pcl::console::print_info ("Saved camera parameters to: %s\n", filename.c_str());
}

void TViewer::loadCameraParameters (const std::string& filename)
{
  int argc = 3;
  struct stat buffer;
  if (stat (filename.c_str(), &buffer) == 0)
  {
    const char* argv[] = {"dummy", "-cam", filename.c_str()};
    if (viewer_->getCameraParameters (argc, const_cast<char**> (argv)))
      pcl::console::print_info ("Loaded camera parameters from: %s\n", filename.c_str());
  }
  else
  {
    pcl::console::print_warn ("Failed to load camera parameters from: %s (file not accessible)\n", filename.c_str());
  }
}

void TViewer::keyboardEventCallback (const pcl::visualization::KeyboardEvent& event)
{
  last_keyboard_event_ = event;
  if (event.keyDown ())
  {
    // save camera parameters
    if (matchKeys (event, "z"))
      saveCameraParameters ("viewpoint.cam");
    // load camera parameters
    if (matchKeys (event, "Z"))
      loadCameraParameters ("viewpoint.cam");
    // print (H)elp
    if (matchKeys (event, "h"))
      pcl::console::print_info (getHelp ().c_str());
    dispatch (event);
    if (matchKeys (event, "Escape"))
      mode_waiting_user_input_ = false;
  }
}

void TViewer::pickPointEventCallback (const pcl::visualization::PointPickingEvent& event)
{
  last_point_picking_event_ = event;
}

void TViewer::registerVisualizationObject (VisualizationObject::Ptr object)
{
  for (const auto& obj : objects_)
    if (obj->name_ == object->name_)
      assert (false);
  objects_.push_back (object);
  object->injectViewer (viewer_);
}

void TViewer::deregisterVisualizationObject (const std::string& object_name)
{
  for (auto iter = objects_.begin (); iter != objects_.end (); ++iter)
    if ((*iter)->name_ == object_name)
    {
      (*iter)->remove ();
      objects_.erase (iter);
      break;
    }
}

void TViewer::showVisualizationObject (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->add ();
}

void TViewer::showVisualizationObjects ()
{
  for (const auto& object : objects_)
    object->add ();
}

void TViewer::hideVisualizationObject (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->remove ();
}

void TViewer::hideVisualizationObjects ()
{
  for (const auto& object : objects_)
    object->remove ();
}

void TViewer::updateVisualizationObject (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->update ();
}

void TViewer::updateVisualizationObjects ()
{
  for (const auto& object : objects_)
    object->update ();
}

void TViewer::dispatch (const pcl::visualization::KeyboardEvent& key_event)
{
  for (const auto& object : objects_)
    object->execute (key_event);
}

std::string TViewer::getHelp ()
{
  boost::format fmt (" %s  %s : %s\n");
  std::string out  = "\nVisualization objects\n---------------------\n";
  if (objects_.size())
    for (size_t i = 0; i < objects_.size (); ++i)
      out += boost::str (fmt % (objects_[i]->visible_ ? "◉" : "○") % objects_[i]->key_ % objects_[i]->description_);
  else
    out += " <none>\n";
  out += "\nPress Ctrl+<key> to shuffle colors";
  out += "\nPress z/Z to save/load camera viewpoint\n";
  return out;
}

}

