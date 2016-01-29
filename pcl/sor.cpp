#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << "[pcd file]" << std::endl;
    return 1;
  }

  const char *filename = "result.pcd";
  int meanK = 50;
  double thd = 1.0;
  for (int i=2; i<argc; i++){
    if(strcmp(argv[i], "-o")==0){
      filename = argv[++i];
    }else if(strcmp(argv[i], "-meanK")==0){
      meanK = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-thd")==0){
      thd = atof(argv[++i]);
    }
  }
  std::cout << "output filename[-o] = " << filename << std::endl;
  std::cout << "meanK[-meanK] = " << meanK << std::endl;
  std::cout << "StddevMulThresh[-thd] = " << thd << std::endl;

  pcl::PCDReader reader;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  reader.read (argv[1], *cloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (thd);
  sor.filter (*filtered_cloud);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("result.pcd", *filtered_cloud, true);
  
  return 0;
}
