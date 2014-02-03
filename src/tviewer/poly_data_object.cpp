#include "poly_data_object.h"

namespace tviewer
{

void PolyDataObject::add ()
{
  if (auto v = viewer_.lock())
  {
    v->addModelFromPolyData (poly_data_, name_);
    visible_ = true;
  }
}

void PolyDataObject::remove ()
{
  if (auto v = viewer_.lock())
  {
    v->removeShape (name_);
    visible_ = false;
  }
}

void PolyDataObject::update ()
{
  if (retrieve_function_)
  {
    poly_data_ = retrieve_function_();
  }

  if (visible_)
  {
    if (auto v = viewer_.lock())
    {
      v->removeShape (name_);
      v->addModelFromPolyData (poly_data_, name_);
    }
  }
}

}

