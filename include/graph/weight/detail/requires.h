/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_GRAPH_WEIGHT_DETAIL_REQUIRES_H
#define PCL_GRAPH_WEIGHT_DETAIL_REQUIRES_H

#include <boost/mpl/apply_fwd.hpp>
#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/limits/vector.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>

#include <pcl/point_types.h>

namespace pcl { namespace graph { namespace weight {

namespace detail
{

  template <BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Field)>
  struct requires_all
  {
    typedef
      typename boost::mpl::remove_if<
        boost::mpl::vector<BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, Field)>
      , boost::is_same<boost::mpl::na, boost::mpl::_1>
      >::type
    requirements;

    typedef
      typename pcl::traits::has_all_fields<
        boost::mpl::_1
      , requirements
      >
    is_satisfied;
  };

  template <BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Field)>
  struct requires_any
  {
    typedef
      typename boost::mpl::remove_if<
        boost::mpl::vector<BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, Field)>
      , boost::is_same<boost::mpl::na, boost::mpl::_1>
      >::type
    requirements;

    typedef
      typename pcl::traits::has_any_field<
        boost::mpl::_1
      , requirements
      >
    is_satisfied;
  };

  template <typename Term, typename Point>
  struct is_satisfied
  {
    typedef typename boost::mpl::apply<typename Term::is_satisfied, Point>::type type;
  };

} // namespace detail

} } } // namespace pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_DETAIL_REQUIRES_H */

