#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << "[pcd files]" << std::endl;
    return 1;
  }

  pcl::PCDReader reader;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr integrated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  integrated_cloud->width = 0;
  integrated_cloud->height = 1;

  for (int i=1; i<argc; i++){
    reader.read (argv[i], *cloud);
    if (cloud->height != 1){
      std::cerr << "height of " << argv[i] << " is not equal to 1" << std::endl;
      return 2;
    }
    int width_old = integrated_cloud->width;
    integrated_cloud->width = width_old + cloud->width;
    integrated_cloud->points.resize(integrated_cloud->width);
    for (int j=0; j<cloud->width; j++){
      integrated_cloud->points[width_old+j] = cloud->points[j]; 
    }
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("result.pcd", *integrated_cloud, true);
  
  return 0;
}
