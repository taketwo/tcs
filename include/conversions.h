#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// Based on geom_utils.h from object discovery source code.

template <typename PointT>
void meshToPointsAndNormals (const pcl::PolygonMesh::ConstPtr& mesh, typename pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  // Unfold mesh, which is stored as ROS message
  pcl::fromPCLPointCloud2 (mesh->cloud, *cloud);
  std::vector<int> counts (cloud->points.size (), 0);
  normals->points.resize (cloud->points.size ());

  for (size_t i = 0; i < mesh->polygons.size (); i++)
  {
    const pcl::Vertices& vv = mesh->polygons[i];

    // Get the 3 points
    int i1 = vv.vertices[0];
    int i2 = vv.vertices[1];
    int i3 = vv.vertices[2];
    PointT& p1 = cloud->points[i1];
    PointT& p2 = cloud->points[i2];
    PointT& p3 = cloud->points[i3];

    // Convert to eigen points
    Eigen::Vector3d pe1 (p1.x, p1.y, p1.z);
    Eigen::Vector3d pe2 (p2.x, p2.y, p2.z);
    Eigen::Vector3d pe3 (p3.x, p3.y, p3.z);

    // Find normal
    Eigen::Vector3d normal = (pe2 - pe1).cross (pe3 - pe1);
    normal = normal / normal.norm ();
    pcl::Normal pnormal (normal[0], normal[1], normal[2]);

    // Smoothly blend with the old normal estimate at this point.
    // Basically each face votes for the normal of the verteces around it.
    float v;
    pcl::Normal a;
    a = normals->points[i1];
    v = 1.0 / (counts[i1] + 1.0);
    normals->points[i1] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    a = normals->points[i2];
    v = 1.0 / (counts[i2] + 1.0);
    normals->points[i2] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    a = normals->points[i3];
    v= 1.0 / (counts[i3] + 1.0);
    normals->points[i3] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    counts[i1]++;
    counts[i2]++;
    counts[i3]++;
  }
}


#endif /* CONVERSIONS_H */

