#ifndef TVIEWER_VISUALIZATION_OBJECT_HPP
#define TVIEWER_VISUALIZATION_OBJECT_HPP

namespace tviewer
{

template <typename T> bool
VisualizationObject::at (size_t index, T& item) const
{
  boost::any any_item;
  if (at_ (index, any_item))
  {
    try
    {
      item = boost::any_cast<T> (any_item);
      return true;
    }
    catch (const boost::bad_any_cast&)
    {
      return false;
    }
  }
  return false;
}

}

#endif /* TVIEWER_VISUALIZATION_OBJECT_HPP */

