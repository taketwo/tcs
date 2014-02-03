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

#ifndef PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H
#define PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H

#include <boost/mpl/at.hpp>
#include <boost/mpl/map.hpp>
#include <boost/bimap.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ref.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

#include "graph/pointcloud_adjacency_list.h"
#include "graph/octree_adjacency_graph_builder.h"

namespace pcl
{

  namespace segmentation
  {

    template <typename PointT>
    class PCL_EXPORTS RandomWalkerSegmentation : public pcl::PCLBase<PointT>
    {

      private:

        using pcl::PCLBase<PointT>::initCompute;
        using pcl::PCLBase<PointT>::deinitCompute;
        using pcl::PCLBase<PointT>::indices_;
        using pcl::PCLBase<PointT>::input_;

        /* The piece of MPL code below is used to relate the possible point
         * types (with which the class template may be instantiated) with
         * corresponding point types augmented with normal+curvature fields. */
        typedef boost::mpl::map
                <
                  boost::mpl::pair<pcl::PointXYZ,          pcl::PointNormal>,
                  boost::mpl::pair<pcl::PointNormal,       pcl::PointNormal>,
                  boost::mpl::pair<pcl::PointXYZRGB,       pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZRGBA,      pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>,
                  boost::mpl::pair<pcl::PointXYZI,         pcl::PointXYZINormal>,
                  boost::mpl::pair<pcl::PointXYZINormal,   pcl::PointXYZINormal>
                > PointTypeAssociations;
        BOOST_MPL_ASSERT ((boost::mpl::has_key<PointTypeAssociations, PointT>));

      public:

        /* Public typedefs for point and point cloud types. */
        typedef PointT                                                       Point;
        typedef pcl::PointCloud<Point>                                       PointCloud;
        typedef typename PointCloud::Ptr                                     PointCloudPtr;
        typedef typename PointCloud::ConstPtr                                PointCloudConstPtr;
        typedef typename boost::mpl::at<PointTypeAssociations, PointT>::type PointWithNormal;
        typedef pcl::PointCloud<PointWithNormal>                             PointWithNormalCloud;
        typedef typename PointWithNormalCloud::Ptr                           PointWithNormalCloudPtr;

        typedef pcl::search::Search<PointWithNormal> Search;
        typedef typename Search::Ptr SearchPtr;

        /* Public typedefs related with graph. */
        typedef boost::subgraph
                <boost::pointcloud_adjacency_list
                <PointWithNormal,
                 boost::vecS,
                 boost::undirectedS,
                 boost::property<boost::vertex_color_t, uint32_t>,
                 boost::property<boost::edge_weight_t, float,
                 boost::property<boost::edge_index_t, int>>>>              Graph;
        typedef typename boost::graph_traits<Graph>::vertex_descriptor     VertexId;
        typedef typename boost::graph_traits<Graph>::edge_descriptor       EdgeId;
        typedef typename boost::graph_traits<Graph>::vertex_iterator       VertexIterator;
        typedef typename boost::graph_traits<Graph>::edge_iterator         EdgeIterator;
        typedef typename boost::graph_traits<Graph>::adjacency_iterator    AdjacencyIterator;
        typedef boost::shared_ptr<Graph>                                   GraphPtr;
        typedef boost::shared_ptr<const Graph>                             GraphConstPtr;
        typedef boost::reference_wrapper<Graph>                            GraphRef;

        RandomWalkerSegmentation (bool compute_potentials = false);

        virtual
        ~RandomWalkerSegmentation ();


/**********************************************************************
 *                     Functions to provide input                     *
 *********************************************************************/


        /** Provide a pointer to the input dataset. */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);


        /******************************
         *  Provide graph explicitly  *
         ******************************/


        /** Provide graph explicitly. */
        void
        setInputGraph (const GraphPtr& graph);


        /*******************
         *  Provide seeds  *
         *******************/


        void
        setSeeds (const pcl::PointCloud<PointXYZL>::ConstPtr& seeds);
        //
        // This version allows to input approximate seeds, also with
        // multiple seed points per label. Internally this will have
        // to do a lookup for closest points.
        // Should it preserve exactly the same labels in the output?
        // If yes then make sure that it does not have 0 labels.
        // Note that label 0xFFFFFFFF is not allowed


        // void setSeeds (const std::vector<PointIndices>& seed_indices)
        //
        // This version allows to input exact seeds (referencing them
        // by their indices). Also with multiple seed points per label.


        // void setSeeds (const PointIndices& seed_indices)
        //
        // This version only allows one seed per label, but might be
        // simpler to use.


/*********************************************************************
 *                              Getters                              *
 *********************************************************************/


        /** \brief Returns the graph that was built (or provided with
          * setInputGraph()) to perform random walker segmentation. */
        inline GraphConstPtr
        getGraph () const
        {
          return (graph_);
        }


/*********************************************************************
 *                   Functions to run segmentation                   *
 *********************************************************************/


        /** \brief Build a graph based on the input dataset.
          *
          * This function constructs a graph and assigns edge weights. The user
          * has to call this function explicitly only if he needs to access the
          * constructed graph before running segmentation (e.g. to select seeds
          * among graph vertices).
          *
          * Example usage:
          *
          * \code
          * RandomWalkerSegmentation<PointT> rws;
          * rws.setInputCloud (cloud);
          * rws.preComputeGraph ();
          * auto graph = rws.getGraph ();
          * // Visualize graph or graph point cloud
          * // Ask the user to select seeds
          * rws.setSeeds (seeds);
          * rws.segment (clusters);
          * \endcode
          *
          * \note It is not necessary to call this function explicitly before
          * performing segmentation with segment()! */
        void
        preComputeGraph ();


        /** \brief Perform random walker segmentation.
          *
          * \param[out] clusters the resultant set of indices, indexing the
          * points of the input cloud that correspond to the clusters */
        void
        segment (std::vector<pcl::PointIndices>& clusters);


/*********************************************************************
 *                      Functions to get output                      *
 *********************************************************************/


        //pcl::PointCloud<pcl::PointXYZL>::Ptr
        //getLabeledCloud () const;
        //
        // Get the same cloud as was input, but with labels. If the
        // seeds were provided with labels, then the labels should match.
        // Otherwise just label from 1 to num_labels. (Reasonable to
        // reserve 0 for unlabeled points).


        // typename PointCloud<PointXYZRGB>::Ptr getColoredCloud ();
        //
        // Same as before, but just use random colors insetad of labels.

        const Eigen::MatrixXf&
        getPotentials () const;

      private:

        typedef typename boost::property_map<Graph, boost::vertex_color_t >::type VertexColorMap;

        bool input_as_cloud_;

        GraphPtr graph_;
        std::vector<GraphRef> graph_components_;

        pcl::PointCloud<PointXYZL>::ConstPtr seeds_;

        pcl::graph::OctreeAdjacencyGraphBuilder<PointT, Graph> graph_builder_;

        /// Maintains bi-directional mapping beetwen seed labels and color
        /// identifiers (which are used in random walker segmentation).
        boost::bimap<uint32_t, uint32_t> label_color_bimap_;

        bool compute_potentials_;
        Eigen::MatrixXf potentials_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  }

}

#ifdef PCL_NO_PRECOMPILE
#include "impl/random_walker_segmentation.hpp"
#endif

#endif /* PCL_SEGMENTATION_RANDOM_WALKER_SEGMENTATION_H */
