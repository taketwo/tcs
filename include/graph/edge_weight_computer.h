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

#ifndef PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H
#define PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H

namespace pcl
{

  namespace graph
  {

    class EdgeWeightComputer
    {

      public:

        enum SmallWeightPolicy
        {
          SMALL_WEIGHT_DO_NOTHING,
          SMALL_WEIGHT_COERCE_TO_THRESHOLD,
          SMALL_WEIGHT_REMOVE_EDGE,
        };

        typedef boost::shared_ptr<EdgeWeightComputer> Ptr;

        EdgeWeightComputer ()
        : policy_ (SMALL_WEIGHT_DO_NOTHING)
        , threshold_ (0.0f)
        , distance_weight_ (0.0f)
        , normal_weight_ (0.0f)
        , curvature_weight_ (0.0f)
        , color_weight_ (0.0f)
        , convex_discount_ (1.0f)
        {
        }

        template <class Graph, class EdgeWeightMap> void
        compute (Graph& graph, EdgeWeightMap weights);

        inline void
        setSmallWeightPolicy (SmallWeightPolicy policy)
        {
          policy_ = policy;
        }

        inline void
        setSmallWeightThreshold (float threshold)
        {
          threshold_ = threshold;
        }

        inline void
        setConvexDiscount (float convex_discount)
        {
          convex_discount_ = convex_discount;
        }

        inline void
        setDistanceWeight (float weight)
        {
          distance_weight_ = weight;
        }

        inline void
        setNormalWeight (float weight)
        {
          normal_weight_ = weight;
        }

        inline void
        setCurvatureWeight (float weight)
        {
          curvature_weight_ = weight;
        }

        inline void
        setColorWeight (float weight)
        {
          color_weight_ = weight;
        }

      private:

        SmallWeightPolicy policy_;
        float threshold_;

        float distance_weight_;
        float normal_weight_;
        float curvature_weight_;
        float color_weight_;

        float convex_discount_;

    };

  }

}

#include "graph/impl/edge_weight_computer.hpp"

#endif /* PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H */

