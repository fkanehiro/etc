#include <cv.h>
#include <highgui.h>
#include <ctype.h>

int
main (int argc, char **argv)
{
  CvCapture *capture = 0;
  IplImage *frame = 0;
  double w = 320, h = 240;
  int c, n = 0;

  for (int i=1; i<argc; i++){
    if (strcmp("-n", argv[i])==0){
      n = atoi(argv[++i]);
    }else if(strcmp("-w", argv[i])==0){
      w = atoi(argv[++i]);
    }else if(strcmp("-h", argv[i])==0){
      h = atoi(argv[++i]);
    }
  }
  capture = cvCreateCameraCapture (n);

  /* この設定は，利用するカメラに依存する */
  // (2)キャプチャサイズを設定する．
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, h);

  cvNamedWindow ("Capture", CV_WINDOW_AUTOSIZE);

  // (3)カメラから画像をキャプチャする
  int cnt=0;
  while (1) {
    frame = cvQueryFrame (capture);
    cvShowImage ("Capture", frame);
    c = cvWaitKey (2);
    if (c == '\x1b'){
      break;
    }else if (c == 'c'){
      char buf[256];
      sprintf(buf, "capture%d.png", cnt++);
      cvSaveImage(buf, frame);
    } 
  }

  cvReleaseCapture (&capture);
  cvDestroyWindow ("Capture");

  return 0;
}
