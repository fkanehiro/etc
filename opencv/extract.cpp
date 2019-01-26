/*
  Original: 
  http://d.hatena.ne.jp/shokai/20090203/1233609981
 */
#include <stdio.h>
#include <highgui.h>
#include <cv.h>


IplImage *img = NULL;
IplImage *imgR, *imgG, *imgB, *imgThreshold_R, *imgThreshold_G, *imgThreshold_B, *imgResult, *imgTmp;
CvMoments moments;

void onMouse(int event, int x, int y, int flags, void* param){
  printf("x:%d y:%d r:%d g:%d b:%d %s", x, y, // マウス座標とRGBを出力
         (unsigned char)imgR->imageDataOrigin[x+y*imgR->width],
         (unsigned char)imgG->imageDataOrigin[x+y*imgG->width],
         (unsigned char)imgB->imageDataOrigin[x+y*imgB->width],
         "\n");
}

int main(int argc, char** argv) {
  bool isStop = false;

  if (argc < 2){
    fprintf(stderr, "usage: %s [image file]\n", argv[0]);
    return 1;
  }

  img = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
  const int w = img->width;
  const int h = img->height;

  imgR = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgG = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgB = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgThreshold_R = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgThreshold_G = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgThreshold_B = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgResult = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
  imgTmp = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

  char winNameSource[] = "Source";
  char winNameResult[] = "Result";

  cvNamedWindow(winNameSource, CV_WINDOW_AUTOSIZE);
  cvNamedWindow(winNameResult, CV_WINDOW_AUTOSIZE);

  cvSetMouseCallback(winNameSource, onMouse, 0);
  cvSetMouseCallback(winNameResult, onMouse, 0);

  cvSplit(img, imgB, imgG, imgR, NULL); // BGRを分解

  // 赤の要素が100以上で、緑と青より1.5倍以上あるピクセルを抽出
  cvThreshold(imgR, imgThreshold_R, 100, 255, CV_THRESH_BINARY);
  cvDiv(imgR, imgG, imgTmp, 10); // 10倍
  cvThreshold(imgTmp, imgThreshold_G, 15, 255, CV_THRESH_BINARY);
  cvDiv(imgR, imgB, imgTmp, 10);
  cvThreshold(imgTmp, imgThreshold_B, 15, 255, CV_THRESH_BINARY);
  cvAnd(imgThreshold_G, imgThreshold_B, imgTmp, NULL);
  cvAnd(imgTmp, imgThreshold_R, imgResult, NULL);
  
  cvMoments(imgResult, &moments, 0);
  double m00 = cvGetSpatialMoment(&moments, 0, 0);
  double m10 = cvGetSpatialMoment(&moments, 1, 0);
  double m01 = cvGetSpatialMoment(&moments, 0, 1);
  int gX = m10/m00;
  int gY = m01/m00;
  cvCircle(img, cvPoint(gX, gY), 10, CV_RGB(0,0,255), 6, 8, 0);
  
  int waitKey;
  while (1) {
    if(!isStop){
      cvShowImage(winNameSource, img);
      cvShowImage(winNameResult, imgResult);
    }

    waitKey = cvWaitKey(33);
    if(waitKey == 'q') break;
    if(waitKey == ' '){
      isStop = !isStop;
      if(isStop) printf("stop\n");
      else printf("start\n");
    }
  }

  cvDestroyWindow(winNameSource);
  cvDestroyWindow(winNameResult);
  return 0;
}
