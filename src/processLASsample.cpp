#include <unistd.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/impl/mls.hpp>

#include "utils/las_io.h"

using namespace pcl;
using namespace std;

int main (int argc, char** argv)
{

  if(argc != 3)
  {
    std::cout << "Wrong parameters. Usage : processLASsample <path_to_las_file> <path_to_ply_file>" << std::endl;
    return 0;
  }

  std::string lasPath(argv[1]);
  std::string plyPath(argv[2]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Read point cloud from LAS file
  LASReader lasReader;
  lasReader.read(lasPath, *cloud);

  cout << "loaded" << endl;

  cout << "begin passthrough filter" << endl;
  PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
  PassThrough<PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.filter(*filtered);
  cout << "passthrough filter complete" << endl;

  cout << "begin moving least squares" << endl;
  MovingLeastSquares<PointXYZ, PointXYZ>* mls = new MovingLeastSquares<PointXYZ, PointXYZ>(); // destruction causes segfault
  mls->setInputCloud(filtered);
  mls->setSearchRadius(0.01);
  mls->setPolynomialFit(true);
  mls->setPolynomialOrder(2);
  mls->setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
  mls->setUpsamplingRadius(0.005);
  mls->setUpsamplingStepSize(0.003);

  PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
  mls->process(*cloud_smoothed);
  cout << "MLS complete" << endl;

  cout << "begin normal estimation" << endl;
  NormalEstimationOMP<PointXYZ, Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(filtered);
  ne.setRadiusSearch(0.1);
  //Eigen::Vector4f centroid;
  //compute3DCentroid(*filtered, centroid);
  //ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
  ne.compute(*cloud_normals);
  cout << "normal estimation complete" << endl;
  cout << "reverse normals' direction" << endl;

  // for(size_t i = 0; i < cloud_normals->size(); ++i){
  // 	cloud_normals->points[i].normal_x *= -1;
  // 	cloud_normals->points[i].normal_y *= -1;
  // 	cloud_normals->points[i].normal_z *= -1;
  // }

  cout << "combine points and normals" << endl;
  PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
  concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

  cout << "begin poisson reconstruction" << endl;
  Poisson<PointNormal> poisson;
  poisson.setDepth(9);
  poisson.setInputCloud(cloud_smoothed_normals);
  PolygonMesh mesh;
  poisson.reconstruct(mesh);
  cout << "end" << endl;

  io::savePLYFile(plyPath, mesh);

  cout << "File " << plyPath << " saved." << endl;

  return 0;
}
