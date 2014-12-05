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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << " PCD_filename [-smoothness 7.0] [-distance 0.1]"
	      << std::endl;
    return 0;
  }
  const char *filename = argv[1];

  double smoothness=7.0, distance=0.01;
  for (int i=2; i<argc; i++){
    if (strcmp("-smoothness",argv[i])==0){
      smoothness=atof(argv[++i]);
    }else if(strcmp("-distance", argv[i])==0){
      distance=atof(argv[++i]);
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read (filename, *cloud);
  int npoint = cloud->points.size();
  std::cout << "npoint = " << npoint << std::endl;


  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (40000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothness / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  int count = 1;
  for (unsigned int i=0; i<clusters.size(); i++){
    std::cout << "cluster" << i << std::endl;
    extract.setInputCloud( cloud );
    *inliers = clusters[i];
    extract.setIndices( inliers );
    extract.setNegative( false );
    extract.filter( *cluster );
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cluster;
    
    while(cloud->points.size() > (int)(cluster->points.size())*0.1){
      std::cout << "count = " << count << std::endl;
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
      
      if (inliers->indices.size () == 0)
	{
	  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	  return (-1);
	}
      
      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
      std::cerr << "roll:" << -asin(coefficients->values[1])
		<< "[rad](" << -asin(coefficients->values[1])*180/M_PI
		<< "[deg]), pitch:" << asin(coefficients->values[0])
		<< "[rad](" << asin(coefficients->values[0])*180/M_PI
		<< "[deg])" << std::endl;
      
      extract.setInputCloud( cloud );
      extract.setIndices( inliers );
      extract.setNegative( false );
      extract.filter( *cloud_p );
      for (unsigned int i=0; i<cloud_p->points.size(); i++){
	cloud_p->points[i].r = count&1 ? 255 : 0;
	cloud_p->points[i].g = count&2 ? 255 : 0;
	cloud_p->points[i].b = count&4 ? 255 : 0;
      }
      
      std::stringstream   ss;
      ss << "cloud_" << count;
      viewer.addPointCloud( cloud_p, ss.str() );
      
      // 平面として選ばれなかった側の点に対する処理
      extract.setNegative( true );
      extract.filter( *cloud_f );
      cloud = cloud_f;
      
      // カウントアップ
      count++;
    }
  }

  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }

  return (0);
}
