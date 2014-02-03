#ifndef COPY_POINT_H
#define COPY_POINT_H

#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>

namespace pcl
{

  namespace detail
  {

    /* CopyPointHelper and its specializations copy the contents of a source
     * point to the target point. There are three cases:
     *
     *  - Points have the same type.
     *    In this case a single `memcpy` is used.
     *
     *  - Points have different types and these types either
     *      * have no RGB/RGBA fields;
     *      * both have RGB fields or both have RGBA fields.
     *    In this case we find the list of common fields and copy their
     *    contents one by one with `NdConcatenateFunctor`.
     *
     *  - Points have different types and one of these types has RGB field, and
     *    the other has RGBA field.
     *    In this case we also find the list of common fields and copy their
     *    contents. In order to account for the fact that RGB and RGBA do not
     *    match we have an additional `memcpy` to copy the contents of one into
     *    another.
     *
     * An appropriate version of CopyPointHelper is instantiated during
     * compilation time automatically, so there is absolutely no run-time
     * overhead.
     */

    template <typename PointInT, typename PointOutT, typename Enable = void>
    struct CopyPointHelper
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
      }
    };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT, typename boost::enable_if<boost::is_same<PointInT, PointOutT> >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        memcpy (&point_out, &point_in, sizeof (PointInT));
      }
    };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT,
                           typename boost::enable_if<boost::mpl::and_<boost::mpl::not_<boost::is_same<PointInT, PointOutT> >,
                                                                      boost::mpl::or_<boost::mpl::and_<boost::mpl::not_<pcl::traits::has_color<PointInT> >,
                                                                                                       boost::mpl::not_<pcl::traits::has_color<PointOutT> > >,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgb>>,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgba>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgba> > > > >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        typedef typename pcl::traits::fieldList<PointInT>::type FieldListInT;
        typedef typename pcl::traits::fieldList<PointOutT>::type FieldListOutT;
        typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (point_in, point_out));
      }
    };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT,
                           typename boost::enable_if<boost::mpl::and_<boost::mpl::not_<boost::is_same<PointInT, PointOutT> >,
                                                                      boost::mpl::or_<boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgba> >,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgba>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgb> > > > >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        typedef typename pcl::traits::fieldList<PointInT>::type FieldListInT;
        typedef typename pcl::traits::fieldList<PointOutT>::type FieldListOutT;
        typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
        const uint32_t offset_in  = boost::mpl::if_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointInT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointInT, pcl::fields::rgba> >::type::value;
        const uint32_t offset_out = boost::mpl::if_<pcl::traits::has_field<PointOutT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointOutT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointOutT, pcl::fields::rgba> >::type::value;
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (point_in, point_out));
        memcpy (reinterpret_cast<char*> (&point_out) + offset_out,
                reinterpret_cast<const char*> (&point_in) + offset_in,
                4);
      }
    };

  }

  template <typename PointInT, typename PointOutT>
  void copyPoint (const PointInT& point_in, PointOutT& point_out)
  {
    detail::CopyPointHelper<PointInT, PointOutT> copy;
    copy (point_in, point_out);
  }

  template <typename PointT>
  void copyPoint (const PointT& point_in, PointT& point_out)
  {
    detail::CopyPointHelper<PointT, PointT> copy;
    copy (point_in, point_out);
  }

}

#endif /* COPY_POINT_H */

