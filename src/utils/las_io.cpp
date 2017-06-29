#include "utils/las_io.h"
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include <pcl/PCLPointCloud2.h>

#include <fstream>  // std::ifstream
#include <iostream> // std::cout

LASReader::LASReader() {}

LASReader::~LASReader() {}

int LASReader::read(const std::string & file_name,
                         pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::ifstream ifs;
  ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);

  liblas::Reader reader(ifs);

  // Setting the is_dense property to true by default
  cloud.is_dense = true;
  cloud.height = 1;
  cloud.width = reader.GetHeader().GetPointRecordsCount();
  cloud.points.resize (cloud.width * cloud.height);

  for(uint64_t i=0; reader.ReadNextPoint(); i++)
  {
      liblas::Point const& p = reader.GetPoint();
      cloud.points[i].x = p.GetX();
      cloud.points[i].y = p.GetY();
      cloud.points[i].z = p.GetZ();
      //cloud.points[i].intensity = p.GetIntensity();
  }
  return cloud.width*cloud.height;
}
