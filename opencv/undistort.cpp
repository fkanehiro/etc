#include <cv.h>
#include <highgui.h>

int
main (int argc, char *argv[])
{
  IplImage *src_img, *dst_img;
  CvMat *intrinsic, *distortion;
  CvFileStorage *fs;
  CvFileNode *param;

  // (1)補正対象となる画像の読み込み
  if (argc < 2 || (src_img = cvLoadImage (argv[1], CV_LOAD_IMAGE_COLOR)) == 0)
    return -1;
  dst_img = cvCloneImage (src_img);

  // (2)パラメータファイルの読み込み
  fs = cvOpenFileStorage ("camera.xml", 0, CV_STORAGE_READ);
  param = cvGetFileNodeByName (fs, NULL, "intrinsic");
  intrinsic = (CvMat *) cvRead (fs, param);
  param = cvGetFileNodeByName (fs, NULL, "distortion");
  distortion = (CvMat *) cvRead (fs, param);
  cvReleaseFileStorage (&fs);

  // (3)歪み補正
  cvUndistort2 (src_img, dst_img, intrinsic, distortion);

  // (4)画像を表示，キーが押されたときに終了
  cvNamedWindow ("Distortion", CV_WINDOW_AUTOSIZE);
  cvShowImage ("Distortion", src_img);
  cvNamedWindow ("UnDistortion", CV_WINDOW_AUTOSIZE);
  cvShowImage ("UnDistortion", dst_img);
  cvWaitKey (0);

  cvDestroyWindow ("Distortion");
  cvDestroyWindow ("UnDistortion");
  cvReleaseImage (&src_img);
  cvReleaseImage (&dst_img);
  cvReleaseMat (&intrinsic);
  cvReleaseMat (&distortion);

  return 0;
}
