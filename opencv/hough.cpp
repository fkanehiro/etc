#include <cv.h>
#include <highgui.h>
#include <math.h>

int
main (int argc, char **argv)
{
  int i;
  float *line, rho, theta;
  double a, b, x0, y0;
  IplImage *src_img_prob = 0, *src_img_gray = 0;
  CvMemStorage *storage;
  CvSeq *lines = 0;
  CvPoint *point, pt1, pt2;

  // (1)画像の読み込み
  if (argc >= 2)
    src_img_gray = cvLoadImage (argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  if (src_img_gray == 0)
    return -1;
  src_img_prob = cvLoadImage (argv[1], CV_LOAD_IMAGE_COLOR);

  // (2)ハフ変換のための前処理
  cvCanny (src_img_gray, src_img_gray, 50, 200, 3);
  storage = cvCreateMemStorage (0);

  // (4)確率的ハフ変換による線分の検出と検出した線分の描画
  lines = 0;
  lines = cvHoughLines2 (src_img_gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 120, 10);
  for (i = 0; i < lines->total; i++) {
    point = (CvPoint *) cvGetSeqElem (lines, i);
    cvLine (src_img_prob, point[0], point[1], CV_RGB (255, 0, 0), 3, 8, 0);
    std::cout << "len = " << sqrt((point[0].x - point[1].x)*(point[0].x - point[1].x)+(point[0].y - point[1].y)*(point[0].y - point[1].y)) << std::endl;
  }

  // (5)検出結果表示用のウィンドウを確保し表示する
  cvNamedWindow ("Hough_line_probabilistic", CV_WINDOW_AUTOSIZE);
  cvShowImage ("Hough_line_probabilistic", src_img_prob);
  cvWaitKey (0);

  cvDestroyWindow ("Hough_line_probabilistic");
  cvReleaseImage (&src_img_prob);
  cvReleaseImage (&src_img_gray);
  cvReleaseMemStorage (&storage);

  return 0;
}
