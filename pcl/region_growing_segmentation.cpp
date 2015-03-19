#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

int
main (int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " [PCD file] [-s smoothness] [-c curvature] [-n neighbors]" << std::endl; 
    return 1;
  }

  const char *filename = argv[1];
  double smoothness = 7.0 / 180.0 * M_PI;
  double curvature = 1.0;
  int neighbors = 30;

  for (int i=2; i<argc; i++){
    if (strcmp(argv[i], "-s")==0){
      smoothness = atof(argv[++i])/180.0*M_PI;
    }else if(strcmp(argv[i], "-c")==0){
      curvature = atof(argv[++i]);
    }else if(strcmp(argv[i], "-n")==0){
      neighbors = atoi(argv[++i]);
    }
  }

  std::cout << "smoothness = " << smoothness*180.0/M_PI << "[deg]" << std::endl;
  std::cout << "curvature = " << curvature << std::endl;
  std::cout << "neighbors = " << neighbors << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read (filename, *cloud);
  int npoint = cloud->points.size();
  std::cout << "npoint = " << npoint << std::endl;


  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighbors);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothness);
  reg.setCurvatureThreshold (curvature);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  while (counter < 5 || counter > clusters[0].indices.size ())
    {
      std::cout << clusters[0].indices[counter] << std::endl;
      counter++;
    }

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
    {
    }

  return (0);
}
