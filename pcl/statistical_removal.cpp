#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_n(new pcl::PointCloud<pcl::PointXYZRGB>);

  const char *filename = "pointCloud.txt";
  if (argc >= 2){
    filename = argv[1];
  }
  std::ifstream ifs(filename);
  char buf[1024];
  for (int i=0; i<5; i++) {
    ifs.getline(buf, 1024);
  }
  int npoint;
  sscanf(buf, "n=%d", &npoint);
  std::cout << "npoint = " << npoint << std::endl;
  
  original->width = npoint;
  original->height = 1;
  original->points.resize (npoint);
  float x,y,z;
  for (int i=0; i<npoint; i++){
    ifs >> x >> y >> z; 
    original->points[i].x = x;
    original->points[i].y = y;
    original->points[i].z = z;
    original->points[i].r = 255;
    original->points[i].g = 255;
    original->points[i].b = 255;
  }
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (original);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*filtered_p);
  sor.setNegative (true);
  sor.filter (*filtered_n);
  for (int i=0; i<filtered_n->points.size(); i++){
    filtered_n->points[i].g = 0;
    filtered_n->points[i].b = 0;
  }
  
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  viewer.addPointCloud( filtered_p, "filtered_p" );
  viewer.addPointCloud( filtered_n, "filtered_n" );

  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }
  
  return (0);
}
