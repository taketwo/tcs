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

#ifndef PCL_GRAPH_WEIGHT_COMPUTERS_NORMALIZED_COMPUTER_H
#define PCL_GRAPH_WEIGHT_COMPUTERS_NORMALIZED_COMPUTER_H

#include <boost/mpl/apply.hpp>

namespace pcl { namespace graph { namespace weight {

template <typename Computer>
struct normalized_computer_base
  : Computer
{

  public:

    typedef normalized_computer_base base;
    typedef boost::mpl::true_ is_normalized;
    typedef typename Computer::point_type point_type;

    template <typename Args>
    normalized_computer_base (const Args& args)
    : Computer (args)
    {
    }

    void
    init (size_t num_edges, size_t num_vertices)
    {
      edge_weights.resize (num_edges, 0.0f);
      vertex_sums.resize (num_vertices, 0.0f);
      vertex_degrees.resize (num_vertices, 0);
    }

    float
    round1 (const point_type& p1,
            const point_type& p2,
            size_t edge_id,
            size_t vertex1_id,
            size_t vertex2_id)
    {
      float weight = Computer::operator() (p1, p2);
      edge_weights[edge_id] = weight;
      vertex_sums[vertex1_id] += weight;
      vertex_sums[vertex2_id] += weight;
      ++vertex_degrees[vertex1_id];
      ++vertex_degrees[vertex2_id];
      return (weight);
    }

    void
    extract ()
    {
      for (size_t i = 0; i < vertex_sums.size (); ++i)
        if (vertex_degrees[i])
          vertex_sums[i] /= vertex_degrees[i];
    }

    float
    round2 (size_t edge_id,
            size_t vertex1_id,
            size_t vertex2_id) const
    {
      float n = (vertex_sums[vertex1_id] + vertex_sums[vertex2_id]) / 2.0;
      float weight = (n > 0.0f && Computer::scale_ > 0.0f) ? edge_weights[edge_id] / n / Computer::scale_: 0.0f;
      return (weight);
    }

    std::string to_str () const
    {
      std::stringstream str;
      str << "{normalized_computer} << " << Computer::to_str ();
      str << "\n  - edges: ";
      for (size_t i = 0; i < edge_weights.size (); ++i)
        str << edge_weights[i] << " ";
      str << "\n  - vertices: ";
      for (size_t i = 0; i < vertex_sums.size (); ++i)
        str << vertex_sums[i] << " ";
      return str.str ();
    }

  private:

    std::vector<float> edge_weights;
    std::vector<float> vertex_sums;
    std::vector<size_t> vertex_degrees;

};

template <typename Computer>
struct normalized_computer
  : normalized_computer_base<Computer>
{

  template <typename Args>
  normalized_computer (const Args& args)
  : normalized_computer::base (args)
  {
  }

  normalized_computer (const normalized_computer& that)
  : normalized_computer::base (*static_cast<typename normalized_computer::base const *> (&that))
  {
  }

};

namespace detail
{

  // Fusion function objects for normalized computers

  struct normalized_computer_init_visitor
  {

    public:

      explicit normalized_computer_init_visitor (size_t num_edges, size_t num_vertices)
      : num_edges_ (num_edges)
      , num_vertices_ (num_vertices)
      {
      }

      template <typename Computer> void
      operator () (Computer& computer) const
      {
        computer.init (num_edges_, num_vertices_);
      }

    private:

      size_t num_edges_;
      size_t num_vertices_;

  };

  template <typename Point>
  struct normalized_computer_round1_visitor
  {

    public:

      typedef float result_type;

      explicit normalized_computer_round1_visitor (const Point& p1,
                                                   const Point& p2,
                                                   size_t edge_id,
                                                   size_t vertex1_id,
                                                   size_t vertex2_id)
      : p1_ (p1)
      , p2_ (p2)
      , edge_ (edge_id)
      , vertex1_ (vertex1_id)
      , vertex2_ (vertex2_id)
      {
      }

      template <typename Computer> void
      operator () (Computer& computer) const
      {
        computer.round1 (p1_, p2_, edge_, vertex1_, vertex2_);
      }

    private:

      normalized_computer_round1_visitor& operator= (const normalized_computer_round1_visitor&);

      const Point& p1_;
      const Point& p2_;
      size_t edge_;
      size_t vertex1_;
      size_t vertex2_;

  };

  struct normalized_computer_extract_visitor
  {

    public:

      template <typename Computer> void
      operator () (Computer& computer) const
      {
        computer.extract ();
      }

  };

} // namespace detail

namespace tag
{

  template <typename Term>
  struct normalized
    : as_term<Term>::type
  {
    typedef typename as_term<Term>::type term_type;
    struct impl
    {
      template <typename Point>
      struct apply
      {
        typedef
          normalized_computer<
            typename boost::mpl::apply<typename term_type::impl, Point>::type
          >
        type;
      };
    };
  };

  template <typename Term>
  struct as_normalized
  {
    typedef normalized<Term> type;
  };

  template <typename Term>
  struct as_normalized<normalized<Term> >
  {
    typedef normalized<Term> type;
  };

} // namespace tag

template <typename Term>
struct term_of<tag::normalized<Term> >
  : term_of<Term>
{
};

} } } // namespace pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_COMPUTERS_NORMALIZED_COMPUTER_H */

