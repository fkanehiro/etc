#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  if (argc < 3){
    std::cerr << "Usage:" << argv[0] << " PCD_filename grid_size" << std::endl;
    return 0;
  }

  std::string filename = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read (filename, *cloud);

  double size = atof(argv[2]);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize(size, size, size);
  sor.filter(*cloud_filtered);
  std::cout << "original:" << cloud->points.size() << ", filtered:" << cloud_filtered->points.size() << std::endl;

  pcl::PCDWriter writer;
  std::string result = filename.substr(0, filename.size()-4) + "_filtered.pcd";
  writer.write<pcl::PointXYZ> (result, *cloud_filtered, false);
}
