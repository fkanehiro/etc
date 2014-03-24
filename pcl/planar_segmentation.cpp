#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::ifstream ifs("pointCloud.txt");
  int npoint = 17286;
  
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
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = original;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  
  int count = 1;
  
  while(cloud->points.size() > (int)(original->points.size())*0.1){
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
    
    extract.setInputCloud( cloud );
    extract.setIndices( inliers );
    extract.setNegative( false );
    extract.filter( *cloud_p );
    for (unsigned int i=0; i<cloud_p->points.size(); i++){
      cloud_p->points[i].r = count&1 ? 255 : 0;
      cloud_p->points[i].g = count&2 ? 255 : 0;
      cloud_p->points[i].b = count&4 ? 255 : 0;
    }

#if 0
    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_p);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
	      << cloud_projected->points.size () << " data points." << std::endl;

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConcaveHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);

    std::cerr << "Concave hull has: " << cloud_hull->points.size ()
	      << " data points." << std::endl;
#endif
    
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
  
  viewer.addPointCloud( cloud_f, "rest" );

  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }
  
  return (0);
}
