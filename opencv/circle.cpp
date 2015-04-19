#include <stdio.h>
#include <cv.h>
#include <highgui.h>

int
main (int argc, char **argv)
{
  int i;
  float *p;
  IplImage *src_img = 0, *src_img_gray = 0;
  CvMemStorage *storage;
  CvSeq *circles = 0;

  // (1)画像の読み込み
  if (argc >= 2)
    src_img_gray = cvLoadImage (argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  if (src_img_gray == 0)
    exit (-1);
  src_img = cvLoadImage (argv[1], CV_LOAD_IMAGE_COLOR);


  // (2)ハフ変換のための前処理（画像の平滑化を行なわないと誤検出が発生しやすい）
  cvSmooth (src_img_gray, src_img_gray, CV_GAUSSIAN, 11, 11, 0, 0);
  storage = cvCreateMemStorage (0);

  // (3)ハフ変換による円の検出と検出した円の描画
  circles = cvHoughCircles (src_img_gray, storage, CV_HOUGH_GRADIENT,
                            1, 100, 20, 50, 10, MAX (src_img_gray->width, src_img_gray->height));
  for (i = 0; i < circles->total; i++) {
    p = (float *) cvGetSeqElem (circles, i);
    cvCircle (src_img, cvPoint (cvRound (p[0]), cvRound (p[1])), 3, CV_RGB (0, 255, 0), -1, 8, 0);
    cvCircle (src_img, cvPoint (cvRound (p[0]), cvRound (p[1])), cvRound (p[2]), CV_RGB (255, 0, 0), 3, 8, 0);
  }

  // (4)検出結果表示用のウィンドウを確保し表示する
  cvNamedWindow ("circles", 1);
  cvShowImage ("circles", src_img);
  cvWaitKey (0);

  cvDestroyWindow ("circles");
  cvReleaseImage (&src_img);
  cvReleaseImage (&src_img_gray);
  cvReleaseMemStorage (&storage);

  return 0;
}
