#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>

int
main (int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << " PCD_filename" << std::endl;
    return 0;
  }
  std::string filename = argv[1];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read (filename, *cloud);

  std::string image = filename.substr(0, filename.size()-4) + ".png";
  pcl::io::savePNGFile(image, *cloud, "rgb");
}
