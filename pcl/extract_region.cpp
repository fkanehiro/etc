#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>

int
main (int argc, char** argv)
{
  if (argc < 6){
    std::cerr << "Usage:" << argv[0] << " PCD_filename x y width height" << std::endl;
    return 0;
  }

  std::string filename = argv[1];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read (filename, *cloud);

  int x,y,width,height;
  x = atoi(argv[2]);
  y = atoi(argv[3]);
  width = atoi(argv[4]);
  height = atoi(argv[5]);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  *extracted_cloud = *cloud; // copy attributes

  extracted_cloud->width = width;
  extracted_cloud->height = height;
  extracted_cloud->points.resize(width*height);
  for (int i=0; i<height; i++){
    for (int j=0; j<width; j++){
      extracted_cloud->points[i*width + j] 
	= cloud->points[(i+y)*cloud->width + j + x];
    }
  }

  pcl::PCDWriter writer;
  std::string result = filename.substr(0, filename.size()-4) + "_extracted.pcd";
  writer.write<pcl::PointXYZRGB> (result, *extracted_cloud, false);
}
