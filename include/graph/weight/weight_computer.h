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

#ifndef PCL_GRAPH_WEIGHT_WEIGHT_COMPUTER_H
#define PCL_GRAPH_WEIGHT_WEIGHT_COMPUTER_H

#include <boost/mpl/limits/vector.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#include <boost/preprocessor/repetition/enum_binary_params.hpp>

#include "graph/weight/weight_fwd.h"
#include "graph/weight/detail/weight_computer.h"

namespace pcl { namespace graph { namespace weight {

BOOST_PARAMETER_KEYWORD(tag, computer)

template <BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, typename Term)>
struct terms
  : boost::mpl::vector<BOOST_PP_ENUM_PARAMS(BOOST_MPL_LIMIT_VECTOR_SIZE, Term)>
{
};

template <typename Term>
struct as_term
{
  typedef Term type;
};

template<typename Term>
struct term_of
{
  typedef Term type;
};

template <typename Point,
          typename Terms,
          typename WeightingFunction,
          typename SmallWeightPolicy>
struct weight_computer
{

  public:

    typedef
      detail::weight_computer_config<
        Point
      , Terms
      , WeightingFunction
      , SmallWeightPolicy
      >
    config;

    typedef Point point_type;
    typedef Terms terms_type;
    typedef WeightingFunction weighting_function_type;
    typedef SmallWeightPolicy small_weight_policy_type;

    weight_computer ()
    : computers_ (
        detail::make_computer_list (
          typename config::computers_mpl_vector ()
        , typename config::params () (*this)
        )
      )
    , small_weight_ (typename config::params () (*this))
    {
    }

    template<typename A1>
    explicit weight_computer (const A1& a1)
    : computers_ (
        detail::make_computer_list (
          typename config::computers_mpl_vector ()
        , typename config::params () (*this, a1)
        )
      )
    , small_weight_ (typename config::params () (*this, a1))
    {
    }

    // ... other overloads generated by Boost.Preprocessor:

#define PCL_GRAPH_WEIGHT_COMPUTER_CTOR(z, n, _)                                       \
    template<BOOST_PP_ENUM_PARAMS_Z(z, n, typename A)>                                \
    weight_computer (BOOST_PP_ENUM_BINARY_PARAMS_Z(z, n, A, const &a))                \
    : computers_ (                                                                    \
        detail::make_computer_list (                                                  \
          typename config::computers_mpl_vector ()                                    \
        , typename config::params () (                                                \
            *this BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, a)                            \
          )                                                                           \
        )                                                                             \
      )                                                                               \
    , small_weight_ (typename config::params () (                                     \
        *this BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, a)                                \
        )                                                                             \
      )                                                                               \
    {                                                                                 \
    }

    BOOST_PP_REPEAT_FROM_TO(
        2
      , BOOST_PP_INC(BOOST_MPL_LIMIT_VECTOR_SIZE)
      , PCL_GRAPH_WEIGHT_COMPUTER_CTOR
      , _
    )

    template <class Graph> void
    operator () (Graph& graph)
    {
      this->operator () (graph, boost::get (boost::edge_weight, graph));
    }

    template <class Graph, class EdgeWeightMap> void
    operator () (Graph& graph, EdgeWeightMap weights)
    {
      config::PreComputeNormalizedTerms::run (computers_, graph);
      config::ComputeTerms::run (computers_, graph, weights);
      small_weight_ (graph, weights);
    }

  private:

    typename config::computers_type computers_;
    small_weight_policy_type small_weight_;

};

} } } // pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_WEIGHT_COMPUTER_H */

