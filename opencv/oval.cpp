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

  cv::Mat gray_img, bin_img;
  cv::cvtColor(src_img, gray_img, CV_BGR2GRAY);

  std::vector<std::vector<cv::Point> > contours;
  // 画像の二値化
  cv::threshold(gray_img, bin_img, 110, 255, cv::THRESH_BINARY);
  // 輪郭の検出
  cv::findContours(bin_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  
  for(int i = 0; i < contours.size(); ++i) {
    size_t count = contours[i].size();
    if(count < 50 || count > 1000) continue; // （小さすぎる|大きすぎる）輪郭を除外

    cv::Mat pointsf;
    cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
    // 楕円フィッティング
    cv::RotatedRect box = cv::fitEllipse(pointsf);
    // 楕円の描画
    cv::ellipse(src_img, box, cv::Scalar(0,0,255), 2, CV_AA);
  }

  cv::namedWindow("fit ellipse", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("bin image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::imshow("fit ellipse", src_img);
  cv::imshow("bin image", bin_img);
  cv::waitKey(0);
}
