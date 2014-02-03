#ifndef TVIEWER_VISUALIZATION_OBJECT_H
#define TVIEWER_VISUALIZATION_OBJECT_H

#include <pcl/visualization/pcl_visualizer.h>

namespace tviewer
{

class VisualizationObject
{

  public:

    typedef std::shared_ptr<VisualizationObject> Ptr;

    VisualizationObject (const std::string& name, const std::string& description, const std::string& key);

    void injectViewer (std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

    /** \brief Show this object on the injected viewer. */
    virtual void add () = 0;

    /** \brief Hide this object from the injected viewer. */
    virtual void remove () = 0;

    /** \brief Update this object and refresh the display. */
    virtual void update () = 0;

    /** \brief Toggle the display of this object. */
    virtual void toggle ();

    /** \brief Execute a command associated with given keyboar event (if any). */
    virtual bool execute (const pcl::visualization::KeyboardEvent& key_event);

    /** \brief Get data element with given index. */
    template <typename T> bool at (size_t index, T& item) const;

    virtual bool
    at_ (size_t index, boost::any& item) const
    {
      return false;
    }

    std::string name_;
    std::string description_;
    std::string key_;
    bool visible_;

    std::weak_ptr<pcl::visualization::PCLVisualizer> viewer_;

};

}

#include "impl/visualization_object.hpp"

#endif /* TVIEWER_VISUALIZATION_OBJECT_H */

