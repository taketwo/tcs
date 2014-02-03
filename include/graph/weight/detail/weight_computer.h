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

#ifndef PCL_GRAPH_WEIGHT_DETAIL_WEIGHT_COMPUTER_H
#define PCL_GRAPH_WEIGHT_DETAIL_WEIGHT_COMPUTER_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>

#include <boost/mpl/map.hpp>
#include <boost/mpl/apply.hpp>
#include <boost/mpl/insert.hpp>
#include <boost/mpl/has_xxx.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/mpl/count_if.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/insert_range.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/transform_view.hpp>
#include <boost/parameter/parameters.hpp>
#include <boost/fusion/include/mpl.hpp>
#include <boost/fusion/include/end.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/fusion/include/cons.hpp>
#include <boost/fusion/include/begin.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/value_of.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/filter_view.hpp>

#include "graph/weight/detail/requires.h"
#include "graph/weight/computers/computer_base.h"
#include "graph/weight/computers/normalized_computer.h"
#include "graph/weight/computers/droppable_computer.h"
// TODO: it should not be there
#include "graph/weight/terms/convexity.h"

namespace pcl { namespace graph { namespace weight {

namespace detail
{

  struct introspector
  {
    template <typename Item> void
    operator () (const Item& item) const
    {
      std::cout << " > " << item.to_str () << std::endl;
    }
  };

  template <typename Sequence>
  void introspect (const Sequence& sequence, const std::string& sequence_name = "")
  {
    std::cout << "## Introspecting [" << sequence_name << "]" << std::endl;
    boost::fusion::for_each (sequence, introspector ());
    std::cout << "## end of introspection" << std::endl;
  }

#define INTROSPECT(sequence) introspect(sequence, #sequence)

  template <typename Computer>
  struct with_normalization
    : Computer::is_normalized
  {
  };

  struct with_normalization_
  {
    template <typename Computer>
    struct apply
      : with_normalization<Computer>
    {
    };
  };

  struct without_normalization_
  {
    template <typename Computer>
    struct apply
      : boost::mpl::not_<with_normalization<Computer> >
    {
    };
  };

  template <typename Computer>
  struct with_droppable
    : Computer::is_droppable
  {
  };

  struct without_droppable_
  {
    template <typename Computer>
    struct apply
      : boost::mpl::not_<with_droppable<Computer> >
    {
    };
  };

  template <typename Terms>
  struct as_term_list
    : boost::mpl::transform_view<Terms, as_term<boost::mpl::_1> >
  {
  };

  template <typename TermMap, typename Term>
  struct insert_term
    : boost::mpl::eval_if<
        boost::mpl::has_key<TermMap, typename term_of<Term>::type>
      , boost::mpl::identity<TermMap>
      , boost::mpl::insert<TermMap, boost::mpl::pair<typename term_of<Term>::type, Term> >
      >
  {
  };

  template <typename Computer, typename Term>
  struct computer_wrapper
    : Computer
  {

    typedef Term term_tag;

    computer_wrapper (const computer_wrapper& that)
    : Computer (*static_cast<const Computer*> (&that))
    {
    }

    template <typename Args>
    computer_wrapper (const Args& args)
    : Computer (args)
    {
    }

  };

  template <typename Term, typename Point>
  struct to_computer
  {
    typedef
      computer_wrapper<
        typename boost::mpl::apply<typename Term::impl, Point>::type
      , Term
      >
    type;
  };

  template <typename Terms, typename Point>
  struct make_computer_vector
  {
    typedef
      typename boost::mpl::fold<
        as_term_list<Terms>
      , boost::mpl::map0<>
      , insert_term<boost::mpl::_1, boost::mpl::_2>
      >::type
    term_map;

    // Turn the map into a vector
    typedef
      typename boost::mpl::insert_range<
        boost::mpl::vector<>
      , boost::mpl::end<boost::mpl::vector<> >::type
      , boost::mpl::transform_view<term_map, boost::mpl::second<boost::mpl::_1> >
      >::type
    term_vector;

    // Keep only terms with satisfied requirements
    typedef
      typename boost::mpl::remove_if<
        term_vector
        , boost::mpl::not_<is_satisfied<boost::mpl::_1, Point> >
      >::type
    feasible_term_vector;

    // From the vector of terms construct a vector of computers
    typedef
      typename boost::mpl::transform<
        feasible_term_vector
      , to_computer<boost::mpl::_1, Point>
      >::type
    type;
  };

  template <
    typename First
  , typename Last
  , bool is_empty = boost::fusion::result_of::equal_to<First, Last>::value
  >
  struct build_computer_list;

  template<typename First, typename Last>
  struct build_computer_list<First, Last, true>
  {
    typedef boost::fusion::nil type;

    template<typename Args>
    static boost::fusion::nil
    call (const Args&, const First&, const Last&)
    {
      return boost::fusion::nil ();
    }
  };

  template<typename First, typename Last>
  struct build_computer_list<First, Last, false>
  {
    typedef build_computer_list<
              typename boost::fusion::result_of::next<First>::type
            , Last
            >
    next_build_computer_list;

    typedef boost::fusion::cons<
              typename boost::fusion::result_of::value_of<First>::type
            , typename next_build_computer_list::type>
    type;

    template<typename Args>
    static type
    call (const Args& args, const First& f, const Last& l)
    {
      return type (args, next_build_computer_list::call (args, boost::fusion::next (f), l));
    }
  };

  namespace meta
  {

    template <typename Sequence>
    struct make_computer_list
      : build_computer_list<
          typename boost::fusion::result_of::begin<Sequence>::type
        , typename boost::fusion::result_of::end<Sequence>::type
        >
    {
    };

  }

  template <typename Sequence, typename Args>
  typename meta::make_computer_list<Sequence>::type
  make_computer_list (const Sequence& seq, const Args& args)
  {
    return meta::make_computer_list<Sequence>::call (args,
                                                     boost::fusion::begin (seq),
                                                     boost::fusion::end (seq));
  }

  template <typename Config, typename Enable = void>
  struct pre_compute_normalized_terms
  {
    template <class Graph> static void
    run (typename Config::computers_type& computers, Graph& graph)
    {
    }
  };

  template <typename Config>
  struct pre_compute_normalized_terms<
    Config
  , typename boost::enable_if<
      typename Config::has_normalized_terms
    >::type
  >
  {
    template <class Graph> static void
    run (typename Config::computers_type& computers, Graph& graph)
    {
      boost::fusion::filter_view<
        typename Config::computers_type
      , with_normalization_
      > computers_with_normalization (computers);

      // Initialize computers with normalization
      boost::fusion::for_each (
        computers_with_normalization
      , normalized_computer_init_visitor (
          boost::num_edges (graph)
        , boost::num_vertices (graph)
        )
      );

      // For each edge pre-compute weights on computers with normalization
      size_t edge_index = 0;
      typename boost::graph_traits<Graph>::edge_iterator ei, ee;
      for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei, ++edge_index)
      {
        typename boost::graph_traits<Graph>::vertex_descriptor v1, v2;
        v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
        boost::fusion::for_each (
          computers_with_normalization
        , normalized_computer_round1_visitor<typename Config::point_type> (
            graph[v1]
          , graph[v2]
          , edge_index
          , v1
          , v2
          )
        );
      }

      // Extract (compute mean on edges and vertices)
      boost::fusion::for_each (
        computers_with_normalization
      , normalized_computer_extract_visitor ()
      );
    }
  };

  template <typename Config, typename Enable = void>
  struct compute_terms
  {
    template <class Graph, class EdgeWeightMap> static void
    run (typename Config::computers_type& computers, Graph& graph, EdgeWeightMap weights)
    {
      size_t edge_index = 0;
      typename boost::graph_traits<Graph>::edge_iterator ei, ee;
      for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei, ++edge_index)
      {
        typename boost::graph_traits<Graph>::vertex_descriptor v1, v2;
        v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
        weights[*ei] = boost::fusion::accumulate (
          computers
        , 1.0f
        , computer_visitor<typename Config::point_type, typename Config::weighting_function_type> (
            graph[v1]
          , graph[v2]
          , edge_index
          , v1
          , v2
          )
        );
      }
    }
  };

  template <typename Config>
  struct compute_terms<
    Config
  , typename boost::enable_if<
      typename Config::has_droppable_terms
    >::type
  >
  {
    template <class Graph, class EdgeWeightMap> static void
    run (typename Config::computers_type& computers, Graph& graph, EdgeWeightMap weights)
    {
      typename boost::mpl::apply<
        tag::convexity::impl
      , typename Config::point_type
      >::type convexity_computer (0);
      boost::fusion::filter_view<
        typename Config::computers_type
      , without_droppable_
      > computers_not_droppable (computers);
      size_t edge_index = 0;
      typename boost::graph_traits<Graph>::edge_iterator ei, ee;
      for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei, ++edge_index)
      {
        typename boost::graph_traits<Graph>::vertex_descriptor v1, v2;
        v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
        computer_visitor<typename Config::point_type, typename Config::weighting_function_type> visitor (
          graph[v1]
        , graph[v2]
        , edge_index
        , v1
        , v2
        );
        if (convexity_computer (graph[v1], graph[v2]) > 0.0f)
          weights[*ei] = boost::fusion::accumulate (
            computers_not_droppable
          , 1.0f
          , visitor
          );
        else
          weights[*ei] = boost::fusion::accumulate (
            computers
          , 1.0f
          , visitor
          );
      }
    }
  };

  template <typename Point,
            typename Terms,
            typename WeightingFunction,
            typename SmallWeightPolicy>
  struct weight_computer_config
  {
    struct config
    {
      typedef Point point_type;
      typedef Terms terms_type;
      typedef WeightingFunction weighting_function_type;
      typedef SmallWeightPolicy small_weight_policy_type;

      typedef
        typename detail::make_computer_vector<
          Terms
        , Point
        >::type
      computers_mpl_vector;

      typedef
        typename detail::meta::make_computer_list<
          computers_mpl_vector
        >::type
      computers_type;

      typedef
        boost::mpl::count_if<
          config::computers_mpl_vector
        , detail::with_normalization<boost::mpl::_1>
        >
      normalized_terms_count;

      typedef
        boost::mpl::count_if<
          config::computers_mpl_vector
        , detail::with_droppable<boost::mpl::_1>
        >
      droppable_terms_count;

      typedef
        typename boost::mpl::greater<
          typename config::normalized_terms_count
        , boost::mpl::int_<0>
        >::type
      has_normalized_terms;

      typedef
        typename boost::mpl::and_<
          typename boost::mpl::greater<
            typename config::droppable_terms_count
          , boost::mpl::int_<0>
          >
        , is_satisfied<
            tag::convexity
          , point_type
          >
        >::type
      has_droppable_terms;
    };

    typedef typename config::computers_mpl_vector computers_mpl_vector;
    typedef typename config::computers_type computers_type;

    typedef pre_compute_normalized_terms<config> PreComputeNormalizedTerms;
    typedef compute_terms<config> ComputeTerms;

    typedef
      boost::parameter::parameters<
        boost::parameter::required<tag::computer>
        // ... and others which are not specified here...
      >
    params;

  };

} // namespace detail

} } } // namespace pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_DETAIL_WEIGHT_COMPUTER_H */

