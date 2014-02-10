/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_GRAPH_POINTCLOUD_ADJACENCY_LIST_H
#define PCL_GRAPH_POINTCLOUD_ADJACENCY_LIST_H

#include <boost/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace boost
{

    /* This is to make a property map on top of a point cloud. Code is based on
     * vector_property_map. */

    template <typename T>
    class pointcloud_property_map
      : public boost::put_get_helper<
          typename std::iterator_traits<
            typename std::vector<T>::iterator
          >::reference
        , pointcloud_property_map<T>
        >
    {

      public:

        typedef typename property_traits<identity_property_map>::key_type key_type;
        typedef T value_type;
        typedef typename std::iterator_traits<typename std::vector<T>::iterator>::reference reference;
        typedef boost::lvalue_property_map_tag category;

        pointcloud_property_map (const typename pcl::PointCloud<T>::Ptr& cloud,
                                 const identity_property_map& index = identity_property_map ())
        : data (cloud)
        , index (index)
        {
        }

        typename std::vector<T>::iterator
        storage_begin ()
        {
          return (data->begin ());
        }

        typename std::vector<T>::iterator
        storage_end ()
        {
          return (data->end ());
        }

        typename std::vector<T>::const_iterator
        storage_begin () const
        {
          return (data->begin ());
        }

        typename std::vector<T>::const_iterator
        storage_end () const
        {
          return (data->end ());
        }

        identity_property_map&
        get_index_map ()
        {
          return (index);
        }

        const identity_property_map&
        get_index_map () const
        {
          return (index);
        }

        reference
        operator[] (const key_type& v) const
        {
          return ((*data)[get (index, v)]);
        }

      private:

        typename pcl::PointCloud<T>::Ptr data;
        identity_property_map index;

  };

  /* This specialized class fixes some of the template parameters of
   * adjacency_list:
   *
   *   - VertexListS = vecS
   *   - GraphProperty = pcl::PointCloud<PointT>::Ptr */
  template <class PointT,
            class OutEdgeListS = vecS,
            class DirectedS = undirectedS,
            class VertexProperty = no_property,
            class EdgeProperty = no_property,
            class EdgeListS = listS>
  class pointcloud_adjacency_list : public adjacency_list<OutEdgeListS,
                                                          vecS,
                                                          DirectedS,
                                                          VertexProperty,
                                                          EdgeProperty,
                                                          typename pcl::PointCloud<PointT>::Ptr,
                                                          EdgeListS>
  {

    private:

      typedef adjacency_list<OutEdgeListS,
                             vecS,
                             DirectedS,
                             VertexProperty,
                             EdgeProperty,
                             typename pcl::PointCloud<PointT>::Ptr,
                             EdgeListS> Base;

    public:

      typedef typename pcl::PointCloud<PointT>::Ptr GraphProperty;

      typedef typename Base::graph_property_type graph_property_type;
      typedef typename Base::graph_bundled graph_bundled;

      // Ensure that the user did not provide his own Bundle for vertices
      //BOOST_STATIC_ASSERT ((is_same<typename Base::vertex_bundled, no_property>::value));

      typedef typename Base::vertex_property_type vertex_property_type;
      typedef PointT vertex_bundled;

      typedef typename Base::edge_property_type edge_property_type;
      typedef typename Base::edge_bundled edge_bundled;

      typedef typename Base::stored_vertex stored_vertex;
      typedef typename Base::vertices_size_type vertices_size_type;
      typedef typename Base::edges_size_type edges_size_type;
      typedef typename Base::degree_size_type degree_size_type;
      typedef typename Base::vertex_descriptor vertex_descriptor;
      typedef typename Base::edge_descriptor edge_descriptor;
      typedef typename Base::out_edge_list_selector out_edge_list_selector;
      typedef typename Base::vertex_list_selector vertex_list_selector;
      typedef typename Base::directed_selector directed_selector;
      typedef typename Base::edge_list_selector edge_list_selector;

      pointcloud_adjacency_list (const GraphProperty& p = GraphProperty (new pcl::PointCloud<PointT>))
      : Base (p->size (), p)
      {
      }

      pointcloud_adjacency_list (vertices_size_type num_vertices,
                                 const GraphProperty& p = GraphProperty (new pcl::PointCloud<PointT>))
      : Base (num_vertices, GraphProperty (new pcl::PointCloud<PointT> (num_vertices, 1)))
      {
      }

      /** Copy constructor.
        *
        * Acts just like the standard copy constructor of boost::adjacency_list,
        * i.e. copies vertex and edge set along with associated properties.
        * Note that a deep copy of the underlying point cloud is made. */
      pointcloud_adjacency_list (const pointcloud_adjacency_list& x)
      : Base (x)
      {
        typename pcl::PointCloud<PointT>::Ptr copy (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud (*get_property_value (*(x.m_property), graph_bundle), *copy);
        get_property_value (*m_property, graph_bundle) = copy;
      }

      /** Assignment operator.
        *
        * Acts just like the standard assignment operator of
        * boost::adjacency_list, i.e. copies vertex and edge set along with
        * associated properties. Note that a deep copy of the underlying point
        * cloud is made. */
      pointcloud_adjacency_list&
      operator= (const pointcloud_adjacency_list& x)
      {
        if (&x != this)
        {
          Base::operator= (x);
          typename pcl::PointCloud<PointT>::Ptr copy (new pcl::PointCloud<PointT>);
          pcl::copyPointCloud (*get_property_value (*(x.m_property), graph_bundle), *copy);
          get_property_value (*m_property, graph_bundle) = copy;
        }
        return (*this);
      }

      void
      clear ()
      {
        BOOST_ASSERT_MSG (false, "addition/removal of vertices is not permitted for pointcloud_adjacency_graph");
      }

      /* The following operator[]'s are exact copies of the ones defined for
       * adjacency_list. We need these because the return type of the original
       * operator[] is no_property (because underlying adjacency_list indeed
       * has no vertex bundle), whereas we need to return points. Since we
       * overrode one of the operators we need to overried the others also. */

      vertex_bundled&
      operator[] (vertex_descriptor e)
      {
        return (get (vertex_bundle, *this)[e]);
      }

      const vertex_bundled&
      operator[] (vertex_descriptor e) const
      {
        return (get (vertex_bundle, *this)[e]);
      }

      edge_bundled&
      operator[] (edge_descriptor e)
      {
        return (get (edge_bundle, *this)[e]);
      }

      const edge_bundled&
      operator[] (edge_descriptor e) const
      {
        return (get (edge_bundle, *this)[e]);
      }

      //This operator allows access to the underlying point cloud, however it
      //gets wraped in a constant pointer so that the user can not modify it.

      typename pcl::PointCloud<PointT>::ConstPtr
      operator[] (graph_bundle_t)
      {
        return (get_property_value (*m_property, graph_bundle_t ()));
      }

      using Base::m_property;

  };

#define PCADJLIST_PARAMS typename P, typename OEL, typename D, typename VP, typename EP, typename EL
#define PCADJLIST pointcloud_adjacency_list<P, OEL, D, VP, EP, EL>

  /* We specialize this traits struct for pointcloud_adjacency_list and further
   * for vertex bundle property in particular to make sure that property access
   * works for subgraphs based on pointcloud_adjacency_list. */

  template <PCADJLIST_PARAMS, typename Property>
  struct property_map<PCADJLIST, Property>
  {
    typedef typename property_map<adjacency_list<OEL, vecS, D, VP, EP, typename pcl::PointCloud<P>::Ptr, EL>, Property>::type type;
    typedef typename property_map<adjacency_list<OEL, vecS, D, VP, EP, typename pcl::PointCloud<P>::Ptr, EL>, Property>::const_type const_type;
  };

  template <PCADJLIST_PARAMS>
  struct property_map<PCADJLIST, vertex_bundle_t>
  {
    typedef pointcloud_property_map<P> type;
    typedef type const_type;
  };

  // TODO: Add documentation and tests

  template <PCADJLIST_PARAMS>
  inline typename pcl::PointCloud<P>::ConstPtr
  get_pointcloud (const PCADJLIST& g)
  {
    return (get_property_value (*g.m_property, graph_bundle_t ()));
  }

  template <PCADJLIST_PARAMS>
  inline pcl::PointIndices::ConstPtr
  get_indices (const PCADJLIST& g)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    indices->indices.resize (num_vertices (g));
    for (size_t i = 0; i < num_vertices (g); ++i)
      indices->indices[i] = i;
    return (indices);
  }

  // TODO: Should this create a new subgraph cloud?

  template <PCADJLIST_PARAMS>
  inline typename pcl::PointCloud<P>::ConstPtr
  get_pointcloud (const subgraph<PCADJLIST>& g)
  {
    return (get_property (g.root ().m_graph, graph_bundle_t ()));
  }

  template <PCADJLIST_PARAMS>
  inline pcl::PointIndices::ConstPtr
  get_indices (const subgraph<PCADJLIST>& g)
  {
    if (g.is_root ())
      return get_indices (g.m_graph);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    indices->indices.resize (num_vertices (g));
    for (size_t i = 0; i < num_vertices (g); ++i)
      indices->indices[i] = g.m_global_vertex[i];
    return (indices);
  }

  /* The purpose of these specializations of templated get/put functions is to
   * enable creation of "virtual" vertex property maps, which use the point
   * cloud stored in the graph bundle as the underlying storage. Without these,
   * more generic specializations that are not aware of the "virtual" vertex
   * bundle will be used.
   *
   * TODO: can we make this really const and provide a separate non-const
   * version? */

  template <PCADJLIST_PARAMS>
  inline pointcloud_property_map<P>
  get (vertex_bundle_t p, const PCADJLIST& g)
  {
    return (pointcloud_property_map<P> (get_property_value (*g.m_property, graph_bundle_t ())));
  }

  template <PCADJLIST_PARAMS, typename Key>
  inline P
  get (vertex_bundle_t p, const PCADJLIST& g, const Key& key)
  {
    return (get (get (p, g), key));
  }

  template <PCADJLIST_PARAMS, typename Key>
  inline void
  put (vertex_bundle_t p, PCADJLIST& g, const Key& key, const P& value)
  {
    put (get (p, g), key, value);
  }

  /* These are needed because pointcloud_adjacency_list is in fact a different
   * class from adjacency_list, and the linker does not allow the get_property
   * functions defined for adjacency_list to be used with
   * pointcloud_adjacency_list.
   *
   * Additionally, we have to disallow returning non-const reference to the
   * graph bundle (to prevent the user from changing it). Other bundles and
   * properties should not be affected though, hence a static assertion. */

  template <PCADJLIST_PARAMS, typename Tag>
  inline typename graph_property<PCADJLIST, Tag>::type&
  get_property (PCADJLIST& g, Tag tag)
  {
    BOOST_STATIC_ASSERT ((!is_same<Tag, graph_bundle_t>::value));
    return (get_property_value (*g.m_property, tag));
  }

  template <PCADJLIST_PARAMS, typename Tag>
  inline const typename graph_property<PCADJLIST, Tag>::type&
  get_property (const PCADJLIST& g, Tag tag)
  {
    return (get_property_value (*g.m_property, tag));
  }

  /* Still, there should be a way to access the graph bundle, so we create a
   * special override for get_property to return the graph bundle by value.
   * What's more, we actually return a const pointer to point cloud, so the
   * user will not be able to modify it.
   *
   * TODO: add an overload with const reference to graph? */

  template <PCADJLIST_PARAMS>
  inline typename pcl::PointCloud<P>::ConstPtr
  get_property (PCADJLIST& g, graph_bundle_t)
  {
    return (get_property_value (*g.m_property, graph_bundle_t ()));
  }

  /* Exactly the same set of get_property functions for subgraphs wrapping
   * pointcloud_adjacency_graph objects. */

  template <PCADJLIST_PARAMS, typename Tag>
  inline typename graph_property<PCADJLIST, Tag>::type&
  get_property (subgraph<PCADJLIST>& g, Tag tag)
  {
    BOOST_STATIC_ASSERT ((!is_same<Tag, graph_bundle_t>::value));
    return (get_property (g.m_graph, tag));
  }

  template <PCADJLIST_PARAMS, typename Tag>
  inline const typename graph_property<PCADJLIST, Tag>::type&
  get_property (const subgraph<PCADJLIST>& g, Tag tag)
  {
    BOOST_STATIC_ASSERT ((!is_same<Tag, graph_bundle_t>::value));
    return (get_property (g.m_graph, tag));
  }

  /* Again a special override for graph bundle, note that here we return the
   * graph bundle of the root graph.
   *
   * TODO: maybe return an on-the-fly constructed pointcloud which conatins
   * only the points of this subgraph?
   *
   * TODO: add an overload with const reference to graph? */

  template <PCADJLIST_PARAMS>
  inline typename pcl::PointCloud<P>::ConstPtr
  get_property (subgraph<PCADJLIST>& g, graph_bundle_t tag)
  {
    return get_property (g.root ().m_graph, tag);
  }

  // Special override for graph bundle, will fail to compile because
  // modification of graph bundle is not allowed
  template <PCADJLIST_PARAMS, typename Tag, typename Value>
  inline void set_property (PCADJLIST& g, Tag tag, const Value& value)
  {
    BOOST_STATIC_ASSERT_MSG ((!is_same<Tag, graph_bundle_t>::value), "modification of graph bundle is not allowed for pointcloud_adjacency_graph");
    get_property_value (*g.m_property, tag) = value;
  }

  /* Specialize some of the functions for manipulating graphs. In particular,
   * disable those that modify vertex set. */

  template <PCADJLIST_PARAMS>
  inline typename PCADJLIST::vertex_descriptor
  add_vertex (PCADJLIST& g)
  {
    BOOST_ASSERT_MSG (false, "addition/removal of vertices is not permitted for pointcloud_adjacency_graph");
    return (0);
  }

  template <PCADJLIST_PARAMS>
  inline typename PCADJLIST::vertex_descriptor
  add_vertex (const typename PCADJLIST::vertex_property_type& p, PCADJLIST& g)
  {
    BOOST_ASSERT_MSG (false, "addition/removal of vertices is not permitted for pointcloud_adjacency_graph");
    return (0);
  }

  template <PCADJLIST_PARAMS>
  inline void remove_vertex (typename PCADJLIST::vertex_descriptor v, PCADJLIST& g)
  {
    BOOST_ASSERT_MSG (false, "addition/removal of vertices is not permitted for pointcloud_adjacency_graph");
    return;
  }

  /* Unfortunately we need to copy over and modify the definition, because the
   * original one uses `add_vertex`, which is not allowed for
   * pointcloud_adjacency_graph. */

  template <PCADJLIST_PARAMS>
  typename subgraph<PCADJLIST>::vertex_descriptor
  add_vertex (typename subgraph<PCADJLIST>::vertex_descriptor u_global, subgraph<PCADJLIST>& g)
  {
    if (g.is_root ())
    {
      BOOST_ASSERT_MSG (false, "addition/removal of vertices is not permitted for root pointcloud_adjacency_graph");
      return (0);
    }

    typename subgraph<PCADJLIST>::vertex_descriptor u_local, v_global;
    typename subgraph<PCADJLIST>::edge_descriptor e_global;

    // Here the original implementation uses `add_vertex`, but we have to
    // replace it with the real code which does the job.
    {
      u_local = g.m_graph.m_vertices.size ();
      g.m_graph.m_vertices.resize (u_local + 1);
      g.m_graph.added_vertex (u_local);
    }

    g.m_global_vertex.push_back (u_global);
    g.m_local_vertex[u_global] = u_local;
    subgraph<PCADJLIST>& r = g.root ();

    typename subgraph<PCADJLIST>::vertex_iterator vi, ve;
    typename subgraph<PCADJLIST>::out_edge_iterator ei, ee;

    // Remember edge global and local maps
    for (boost::tie (ei, ee) = out_edges (u_global, r); ei != ee; ++ei)
    {
      e_global = *ei;
      v_global = target (e_global, r);
      if (g.find_vertex (v_global).second == true)
        g.local_add_edge (u_local, g.global_to_local (v_global), e_global);
    }

    // Not necessary for undirected graphs
    if (is_directed (g))
    {
      for (boost::tie (vi, ve) = vertices (r); vi != ve; ++vi)
      {
        v_global = *vi;
        if (v_global == u_global)
          continue; // don't insert self loops twice!
        if (!g.find_vertex (v_global).second)
          continue; // not a subgraph vertex => try next one
        for (boost::tie (ei, ee) = out_edges (*vi, r); ei != ee; ++ei)
        {
          e_global = *ei;
          if (target (e_global, r) == u_global)
            g.local_add_edge (g.global_to_local (v_global), u_local, e_global);
        }
      }
    }

    return (u_local);
  }

#undef PCADJLIST_PARAMS
#undef PCADJLIST

  /* In many cases it will be useful to be able to determine the bundled point
   * type based on a given Graph type. Unfortunately, it is not trivial to
   * access this type because Graph::vertex_bundled will not exist if Graph is
   * actually a boost::subgraph (which we would also like to support). In the
   * latter case Graph::graph_type::vertex_bundled has to be used. Hence this
   * tricky code to select the appropriate typename depending on the
   * availability. Example usage:
   *
   *   typedef typename boost::vertex_point_type<Graph>::type PointT;
   *
   */

  namespace detail { template <class T> struct void_ { typedef void type; }; }

  template <class T, class = void> struct vertex_point_type
  {
    typedef typename T::vertex_bundled type;
  };

  template <class T> struct vertex_point_type<T, typename detail::void_<typename T::graph_type>::type>
  {
    typedef typename T::graph_type::vertex_bundled type;
  };

}

#endif /* PCL_GRAPH_POINTCLOUD_ADJACENCY_LIST_H */

