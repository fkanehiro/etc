#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << " PCD_filename [-radius 0.03]"
	      << std::endl;
    return 0;
  }
  const char *filename = argv[1];

  double radius=0.03; 
  for (int i=2; i<argc; i++){
    if (strcmp("-radius",argv[i])==0){
      radius=atof(argv[++i]);
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read (filename, *cloud);
  int npoint = cloud->points.size();
  std::cout << "npoint = " << npoint << std::endl;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal> normals;

  ne.setRadiusSearch (radius);
  ne.compute(normals);

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  viewer.addPointCloud( cloud , "cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>( cloud, normals.makeShared(), 10, 0.1, "normals");
      
  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }

  return (0);
}
