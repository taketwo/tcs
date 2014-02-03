#include <iostream>
#include <fstream>

#include <boost/algorithm/string/predicate.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>

#include "io.h"
#include "conversions.h"

template <typename PointT> bool
load (const std::string& filename,
      typename pcl::PointCloud<PointT>::Ptr& cloud,
      typename pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::console::print_highlight ("Loading file \"%s\"... ", filename.c_str ());
  cloud.reset (new pcl::PointCloud<PointT>);
  normals.reset (new pcl::PointCloud<pcl::Normal>);
  if (boost::algorithm::ends_with (filename, ".pcd"))
  {
    if (pcl::io::loadPCDFile<PointT> (filename, *cloud))
    {
      pcl::console::print_error ("error!\n");
      return false;
    }
  }
  else if (boost::algorithm::ends_with (filename, ".ply"))
  {
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    if (!pcl::io::loadPolygonFile (filename, *mesh))
    {
      pcl::console::print_error ("error!\n");
      return false;
    }
    meshToPointsAndNormals<PointT> (mesh, cloud, normals);
  }
  pcl::console::print_info ("%zu points.\n", cloud->size ());
  return true;
}

void
save (const std::string& filename,
      const Eigen::SparseMatrix<float>& M)
{
  std::ofstream file (filename);
  if (file.is_open ())
  {
    file << M.rows () << " " << M.cols () << " " << M.nonZeros () << "\n";
    for (int k = 0; k < M.outerSize (); ++k)
      for (Eigen::SparseMatrix<float>::InnerIterator it (M, k); it; ++it)
        file << it.row () + 1 << " " << it.col () + 1 << " " << it.value () << "\n";
    file.close();
  }
}

void
load (const std::string& filename,
      Eigen::SparseMatrix<float>& M)
{
  std::ifstream stream (filename);
  if (stream)
  {
    typedef Eigen::Triplet<float> T;
    std::vector<T> triplets;
    std::string line;
    int rows = -1;
    int cols = -1;
    while (getline (stream, line))
    {
      if (!line.size () || line[0] == '%')
      {
      }
      else if (rows == -1 && cols == -1)
      {
        std::stringstream (line) >> rows >> cols;
      }
      else
      {
        int row, col;
        float value;
        std::stringstream (line) >> row >> col >> value;
        triplets.push_back (T (row - 1, col - 1, value));
      }
    }
    M.resize (rows, cols);
    if (triplets.size ())
      M.setFromTriplets (triplets.begin (), triplets.end ());
  }
  stream.close ();
}

template bool
load <pcl::PointXYZ>
(const std::string& filename,
 typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
 typename pcl::PointCloud<pcl::Normal>::Ptr normals);

template bool
load <pcl::PointXYZRGB>
(const std::string& filename,
 typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
 typename pcl::PointCloud<pcl::Normal>::Ptr normals);

template bool
load <pcl::PointXYZRGBA>
(const std::string& filename,
 typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
 typename pcl::PointCloud<pcl::Normal>::Ptr normals);
