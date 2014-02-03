#include "utils.h"
#include "tviewer.h"

namespace tviewer
{

std::unique_ptr<TViewerInterface> create (bool with_gui)
{
  return std::unique_ptr<TViewerInterface> (with_gui ? (new TViewer) : (new TViewerInterface));
}

bool
matchKeys (const pcl::visualization::KeyboardEvent& key_event, const std::string& key)
{
  // Special case, keys with Control or Alt: "C-x" or "A-x"
  if (key.size () >= 3 && key[1] == '-')
  {
    const std::string k = key.substr (2);
    if ((key[0] == 'A' && key_event.isAltPressed ()) || (key[0] == 'C' && key_event.isCtrlPressed ()))
      return k == key_event.getKeySym ();
    else
      return false;
  }
  // Regular case
  if (!key_event.isAltPressed () && !key_event.isCtrlPressed ())
    return key_event.getKeySym () == key;
  else
    return false;
}

}

