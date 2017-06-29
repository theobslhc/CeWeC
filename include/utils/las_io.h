#ifndef LAS_IO_H_
#define LAS_IO_H_

#include <pcl/io/file_io.h>

class LASReader {

public:
  LASReader();
  ~LASReader();

  int read(const std::string &file_name, pcl::PointCloud<pcl::PointXYZ> &cloud);
};

#endif /* LAS_IO_H_ */
