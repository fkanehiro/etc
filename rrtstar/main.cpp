#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rrtstar.h"

int main(int argc, char *argv[])
{
  double gamma=0.2;
  for (int i=0; i<argc; i++){
    if (strcmp(argv[i],"-gamma")==0){
      gamma = atof(argv[++i]);
    }
  }

  rrtstar p;
  p.setGamma(gamma);

  cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  region g(0.5, 0.5, 0.1, 0.1);
  p.setGoalRegion(g);
  p.initPlan(nodePtr(new node(-0.5,-0.5)));
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
    std::vector<nodePtr>& verticesInGoalRegion = p.verticesInGoalRegion();
    for (unsigned int i=0; i<verticesInGoalRegion.size(); i++){
      nodePtr t = verticesInGoalRegion[i];
      nodePtr f = t->parent();
      while (f){
	cv::line(img, cv::Point(f->x()*w/2+w/2, f->y()*w/2+w/2), 
		 cv::Point(t->x()*w/2+w/2, t->y()*w/2+w/2), 
		 cv::Scalar(200,0,0), 1, CV_AA);
	t = f;
	f = t->parent();
      }
    }
    cv::imshow("drawing", img);
    cv::waitKey(0);
  }

  return 0;
}
