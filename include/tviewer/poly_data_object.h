#ifndef TVIEWER_POLY_DATA_OBJECT_H
#define TVIEWER_POLY_DATA_OBJECT_H

#include "visualization_object.h"

namespace tviewer
{

class PolyDataObject : public VisualizationObject
{

  public:

    typedef vtkPolyData PolyData;
    typedef vtkSmartPointer<PolyData> PolyDataPtr;

    typedef std::function<PolyDataPtr ()> RetrieveDataFunction;

    PolyDataObject (const std::string& name,
                    const std::string& description,
                    const std::string& key,
                    PolyDataPtr poly_data)
    : VisualizationObject (name, description, key)
    , poly_data_ (poly_data)
    {
    }

    PolyDataObject (const std::string& name,
                    const std::string& description,
                    const std::string& key,
                    const RetrieveDataFunction& retrieve_function)
    : VisualizationObject (name, description, key)
    , retrieve_function_ (retrieve_function)
    , poly_data_ (PolyDataPtr::New ())
    {
    }

    virtual void add ();

    virtual void remove ();

    virtual void update ();

    RetrieveDataFunction retrieve_function_;
    PolyDataPtr poly_data_;

};

}

#endif /* TVIEWER_POLY_DATA_OBJECT_H */

