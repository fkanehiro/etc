#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " [ascii pcd]" << std::endl;
    return 1;
  }

  pcl::PCDReader reader;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloud);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("bin.pcd", *cloud, true);
  
  return 0;
}
