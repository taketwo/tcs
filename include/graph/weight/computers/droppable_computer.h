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

#ifndef PCL_GRAPH_WEIGHT_COMPUTERS_DROPPABLE_COMPUTER_H
#define PCL_GRAPH_WEIGHT_COMPUTERS_DROPPABLE_COMPUTER_H

#include <boost/mpl/apply.hpp>

namespace pcl { namespace graph { namespace weight {

template <typename Computer>
struct droppable_computer
  : Computer
{

  typedef boost::mpl::true_ is_droppable;
  typedef typename Computer::point_type point_type;

  template <typename Args>
  droppable_computer (const Args& args)
  : Computer (args)
  {
  }

  std::string to_str () const
  {
    return "{droppable_computer} << " + Computer::to_str ();
  }

};

namespace tag
{

  template <typename Term>
  struct drop_if_convex
    : as_term<Term>::type
  {
    typedef typename as_term<Term>::type term_type;
    struct impl
    {
      template <typename Point>
      struct apply
      {
        typedef
          droppable_computer<
            typename boost::mpl::apply<typename term_type::impl, Point>::type
          >
        type;
      };
    };
  };

} // namespace tag

template <typename Term>
struct term_of<tag::drop_if_convex<Term> >
  : term_of<Term>
{
};

} } } // namespace pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_COMPUTERS_DROPPABLE_COMPUTER_H */

