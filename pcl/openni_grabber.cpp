#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

const char *filename = "test.pcd";

void grabberCallbackDepthAndColor(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  pcl::io::savePCDFileASCII(filename, *cloud);
  exit(0);
}

int main(int argc, char *argv[])
{
  if (argc > 1){
    filename = argv[1];
  }

  pcl::Grabber *grabber = new pcl::io::OpenNI2Grabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(grabberCallbackDepthAndColor, _1);
  grabber->registerCallback(f);
  grabber->start();
  while(1){
    sleep(1);
  }
}
