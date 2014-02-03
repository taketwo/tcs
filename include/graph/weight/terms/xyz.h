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

#ifndef PCL_GRAPH_WEIGHT_TERMS_XYZ_H
#define PCL_GRAPH_WEIGHT_TERMS_XYZ_H

#include "graph/weight/computers/computer_base.h"

namespace pcl { namespace graph { namespace weight {

BOOST_PARAMETER_NESTED_KEYWORD(tag, xyz_scale, scale)

namespace impl
{

  template <typename Point>
  struct xyz_impl
    : computer_base
  {

    typedef Point point_type;

    template <typename Args>
    xyz_impl (const Args& args)
    : scale_ (args[xyz_scale | 1.0f])
    {
    }

    float operator () (const point_type& p1, const point_type& p2) const
    {
      return (scale_ > 0.0f ? (p2.getVector3fMap () - p1.getVector3fMap ()).squaredNorm () / scale_ : 0.0f);
    }

    std::string to_str () const
    {
      return "{xyz_impl}";
    }

    float scale_;

  };

} // namespace impl

namespace tag
{

  struct xyz
    : detail::requires_all<pcl::fields::x, pcl::fields::y, pcl::fields::z>
    , xyz_scale
  {
    typedef impl::xyz_impl<boost::mpl::_1> impl;
  };

} // namespace tag

} } } // pcl::graph::weight

#endif /* PCL_GRAPH_WEIGHT_TERMS_XYZ_H */

