#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << "[pcd file]" << std::endl;
    return 1;
  }

  double dmax=std::numeric_limits<double>::infinity();
  double dmin=-std::numeric_limits<double>::infinity();
  double xmax=std::numeric_limits<double>::infinity();
  double xmin=-std::numeric_limits<double>::infinity();
  double ymax=std::numeric_limits<double>::infinity();
  double ymin=-std::numeric_limits<double>::infinity();
  double zmax=std::numeric_limits<double>::infinity();
  double zmin=-std::numeric_limits<double>::infinity();
  const char *filename = "result.pcd";
  for (int i=2; i<argc; i++){
    if (strcmp(argv[i], "-dmin")==0){
      dmin = atof(argv[++i]);
    }else if(strcmp(argv[i], "-dmax")==0){
      dmax = atof(argv[++i]);
    }else if (strcmp(argv[i], "-xmin")==0){
      xmin = atof(argv[++i]);
    }else if(strcmp(argv[i], "-xmax")==0){
      xmax = atof(argv[++i]);
    }else if (strcmp(argv[i], "-ymin")==0){
      ymin = atof(argv[++i]);
    }else if(strcmp(argv[i], "-ymax")==0){
      ymax = atof(argv[++i]);
    }else if (strcmp(argv[i], "-zmin")==0){
      zmin = atof(argv[++i]);
    }else if(strcmp(argv[i], "-zmax")==0){
      zmax = atof(argv[++i]);
    }else if(strcmp(argv[i], "-o")==0){
      filename = argv[++i];
    }
  }

  pcl::PCDReader reader;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  reader.read (argv[1], *cloud);

  if (cloud->height != 1){
    std::cerr << "height of " << argv[1] << " is not equal to 1" << std::endl;
    return 2;
  }
  
  filtered_cloud->points.resize(cloud->points.size());

  int cnt = 0;
  for (int i=0; i<cloud->points.size(); i++){
    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;
    double d = sqrt(x*x + y*y + z*z);
    if (d < dmin || d > dmax || x < xmin || x > xmax 
	|| y < ymin || y > ymax || z < zmin || z > zmax) continue;
    filtered_cloud->points[cnt++] = cloud->points[i];
  }

  filtered_cloud->height = 1;
  filtered_cloud->width = cnt;
  filtered_cloud->points.resize(cnt);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (filename, *filtered_cloud, true);
  
  return 0;
}
