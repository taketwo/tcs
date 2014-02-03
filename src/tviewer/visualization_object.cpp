#include "visualization_object.h"
#include "utils.h"

namespace tviewer
{

VisualizationObject::VisualizationObject (const std::string& name, const std::string& description, const std::string& key)
: name_ (name)
, description_ (description)
, key_ (key)
, visible_ (false)
{
}

void VisualizationObject::injectViewer (std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
  viewer_ = viewer;
}

void VisualizationObject::toggle ()
{
  if (visible_)
    this->remove ();
  else
    this->add ();

  pcl::console::print_info (" %s  %s\n", (visible_ ? "◉" : "○"), name_.c_str ());
}

bool VisualizationObject::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, key_))
  {
    this->toggle ();
    return true;
  }
  return false;
}

}

