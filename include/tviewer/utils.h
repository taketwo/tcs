#ifndef TVIEWER_UTILS_H
#define TVIEWER_UTILS_H

#include <pcl/visualization/keyboard_event.h>

#include "tviewer_interface.h"

namespace tviewer
{

std::unique_ptr<TViewerInterface> create (bool with_gui = true);

bool matchKeys (const pcl::visualization::KeyboardEvent& key_event, const std::string& key);

}

#endif /* TVIEWER_UTILS_H */

