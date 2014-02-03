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

#ifndef PCL_GRAPH_WEIGHT_COMPUTERS_COMPUTER_BASE_H
#define PCL_GRAPH_WEIGHT_COMPUTERS_COMPUTER_BASE_H

#include <boost/utility.hpp>

namespace pcl { namespace graph { namespace weight {

struct computer_base
{

  typedef boost::mpl::false_ is_normalized;
  typedef boost::mpl::false_ is_droppable;

  std::string to_str () const
  {
    return "{computer_base}";
  }

};

namespace detail
{

  // Fusion function object to be used to invoke various computers and
  // accumulate the result.

  template <typename Point, typename WeightingFunction>
  struct computer_visitor
  {

    public:

      typedef float result_type;

      explicit computer_visitor (const Point& p1,
                                 const Point& p2,
                                 size_t edge_id,
                                 size_t vertex1_id,
                                 size_t vertex2_id)
      : p1_ (p1)
      , p2_ (p2)
      , edge_ (edge_id)
      , vertex1_ (vertex1_id)
      , vertex2_ (vertex2_id)
      , weighting_function_ ()
      {
      }

      // Overload for computers with normalization (normalized computers)

      template <typename Computer>
      typename boost::enable_if<detail::with_normalization<Computer>, result_type>::type
      operator () (const float& val, const Computer& computer) const
      {
        return (val * weighting_function_ (computer.round2 (edge_, vertex1_, vertex2_)));
      }

      // Overload for computers without normalization (base computers)

      template <typename Computer>
      typename boost::disable_if<detail::with_normalization<Computer>, result_type>::type
      operator () (const float& val, const Computer& computer) const
      {
        return (val * weighting_function_ (computer (p1_, p2_)));
      }

    private:

      computer_visitor& operator= (const computer_visitor&);

      const Point& p1_;
      const Point& p2_;
      size_t edge_;
      size_t vertex1_;
      size_t vertex2_;
      const WeightingFunction weighting_function_;

  };

} // namespace detail

} } } // namespace pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_COMPUTERS_COMPUTER_BASE_H */

