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

#ifndef PCL_GRAPH_WEIGHT_WEIGHT_FWD_H
#define PCL_GRAPH_WEIGHT_WEIGHT_FWD_H

#include <boost/mpl/apply_fwd.hpp>
#include <boost/mpl/limits/vector.hpp>
#include <boost/preprocessor/repetition/enum_params_with_a_default.hpp>

namespace pcl { namespace graph { namespace weight {

// TODO: is needed?
namespace tag
{
  struct computer;
}

namespace tag
{
  template <typename Term, typename NormalizationType>
  struct normalized;

  template <typename Term>
  struct drop_if_convex;
}

template <typename Point,
          typename Terms,
          typename WeightingFunction,
          typename SmallWeightPolicy>
struct weight_computer;

template <typename Term>
struct term_of;

template <typename Term>
struct as_term;

template <BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Term, boost::mpl::na)>
struct terms;

namespace detail
{

  template <BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Field, boost::mpl::na)>
  struct requires_all;

  template <BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Field, boost::mpl::na)>
  struct requires_any;

  template <typename Term, typename Point>
  struct to_computer;

  template <typename Computer>
  struct with_normalization;

}

} } } // pcl::graph::weight

// For defining boost::parameter keywords that can be inherited from to
// get a nested, class-scoped keyword with the requested alias
#define BOOST_PARAMETER_NESTED_KEYWORD(tag_namespace, name, alias)                                  \
    namespace tag_namespace                                                                         \
    {                                                                                               \
        template<int Dummy = 0>                                                                     \
        struct name ## _                                                                            \
        {                                                                                           \
            static char const* keyword_name()                                                       \
            {                                                                                       \
                return #name;                                                                       \
            }                                                                                       \
            static ::boost::parameter::keyword<name ## _<Dummy> > &alias;                           \
        };                                                                                          \
        template<int Dummy>                                                                         \
        ::boost::parameter::keyword<name ## _<Dummy> > &name ## _<Dummy>::alias =                   \
        ::boost::parameter::keyword<name ## _<Dummy> >::get();                                      \
        typedef name ## _ <> name;                                                                  \
    }                                                                                               \
    namespace                                                                                       \
    {                                                                                               \
        ::boost::parameter::keyword<tag_namespace::name> &name =                                    \
        ::boost::parameter::keyword<tag_namespace::name>::get();                                    \
    }

#endif /* PCL_GRAPH_WEIGHT_WEIGHT_FWD_H */

