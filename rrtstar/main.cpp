#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rrtstar.h"

int main(int argc, char *argv[])
{
  rrtstar p;

  cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  p.initPlan(nodePtr(new node(0,0)));
  for (int j=0; j<100; j++){
    int w=500;
    cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);
    for (int k=0; k<100; k++) p.oneStep();
    std::vector<nodePtr>& vertices = p.vertices();
    for (unsigned int i=0; i<vertices.size(); i++){
      nodePtr t = vertices[i];
      nodePtr f = t->parent();
      if (!f) continue;
      cv::line(img, cv::Point(f->x()*w/2+w/2, f->y()*w/2+w/2), 
	       cv::Point(t->x()*w/2+w/2, t->y()*w/2+w/2), 
	       cv::Scalar(0,0,200), 1, CV_AA);
    }
    cv::imshow("drawing", img);
    cv::waitKey(0);
  }

  return 0;
}
