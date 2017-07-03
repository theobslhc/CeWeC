#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

using namespace pcl;

int
 main (int argc, char** argv)
{

  if(argc != 4)
  {
    std::cout << "Wrong parameters. Usage : processLASsample <path_to_ply_to_align> <path_to_target_ply> <path_to_ply_to_save>" << std::endl;
    return 0;
  }

  std::string plyToAlign(argv[1]);
  std::string targetPly(argv[2]);
  std::string plyToSave(argv[3]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PLYReader Reader;
  Reader.read(plyToAlign, *cloud_in);
  Reader.read(targetPly, *cloud_target);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_target);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  io::savePLYFile(plyToSave, Final);

 return (0);
}
