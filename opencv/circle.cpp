#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int
main(int argc, char *argv[])
{
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " [image file]" << std::endl;
    return 1;
  } 
  cv::Mat src_img = cv::imread(argv[1], 1);
  if(!src_img.data) {
    std::cerr << "failed to open the image(" << argv[1] << ")" << std::endl; 
    return -1; 
  }

  cv::Mat dst_img, work_img;
  dst_img = src_img.clone();
  cv::cvtColor(src_img, work_img, CV_BGR2GRAY);

  // Hough変換のための前処理（画像の平滑化を行なわないと誤検出が発生しやすい）
  //cv::GaussianBlur(work_img, work_img, cv::Size(11,11), 2, 2);
  cv::GaussianBlur(work_img, work_img, cv::Size(5,5), 2, 2);
  
  // Hough変換による円の検出と検出した円の描画
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(work_img, circles, CV_HOUGH_GRADIENT, 1, 100, 20, 30, 10, 30);

  std::vector<cv::Vec3f>::iterator it = circles.begin();
  for(; it!=circles.end(); ++it) {
    cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
    int radius = cv::saturate_cast<int>((*it)[2]);
    std::cout << "radius=" << radius << std::endl;
    cv::circle(dst_img, center, radius, cv::Scalar(0,0,255), 2);
  }

  cv::namedWindow("HoughCircles", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("Blurred", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::imshow("HoughCircles", dst_img);
  cv::imshow("Blurred", work_img);
  cv::waitKey(0);
}
