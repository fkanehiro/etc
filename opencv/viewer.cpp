#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>

/* main */
int main(int argc, char *argv[]) {
    IplImage* image;

    /* 静止画像を読み込む */
    image = cvLoadImage(argv[1],CV_LOAD_IMAGE_ANYCOLOR);
    if (image == NULL) {
        fprintf(stderr, "読込みに失敗しました.");
        return EXIT_FAILURE;
    }

    /* ウインドウを準備して画像を表示する */
    cvNamedWindow("Image",CV_WINDOW_AUTOSIZE);
    cvShowImage("Image",image);

    /* キー入力があるまで待つ */
    cvWaitKey(0); /* これがないと、1瞬だけ表示されて終わる */

    /* メモリを開放する */
    cvReleaseImage(&image);

    /* ウィンドウを破棄する */
    cvDestroyWindow("Image");

    return EXIT_SUCCESS;
}
