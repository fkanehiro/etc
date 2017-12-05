#include <cv.h>
#include <highgui.h>
#include <math.h>

int main(int argc, char *argv[])
{
  IplImage* orig = cvLoadImage("test.jpg");
  if (!orig)
    {
      std::cerr << "failed to load test.jpg" << std::endl;
      return -1;
    }
  printf("Orig dimensions: %dx%d\n", orig->width, orig->height);
  
  cvSetImageROI(orig, cvRect(0, 250, 150, 200));
  
  IplImage *tmp = cvCreateImage(cvGetSize(orig),
                                orig->depth,
                                orig->nChannels);
  
  cvCopy(orig, tmp, NULL);
  cvResetImageROI(orig);
  
  orig = cvCloneImage(tmp);
  printf("Orig dimensions after crop: %dx%d\n", orig->width, orig->height);
  
  cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );
  cvShowImage( "result", orig);
  cvWaitKey( 0 );
  cvDestroyWindow( "result" );
}
