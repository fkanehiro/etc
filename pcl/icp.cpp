#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
main (int argc, char** argv)
{
  if (argc < 3){
    std::cerr << "Usage: " << argv[0] << " [PCD 1] [PCD 2]" << std::endl;
    return 1;
  }

  double maxCorrespondenceDistance = 0.05;
  int maxIterations = 50;
  double epsilon = 1e-8;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;

  reader.read(argv[1], *cloud_in);
  reader.read(argv[2], *cloud_out);
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
  icp.setMaximumIterations(maxIterations);
  icp.setTransformationEpsilon(epsilon);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  return (0);
}

