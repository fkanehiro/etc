#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>
#define STR(s) #s

using namespace std;
using namespace zbar;

int main (int argc, char **argv)
{
    if(argc < 2) return(1);

    // create a reader
    ImageScanner scanner;

    // configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    // obtain image data
    cv::Mat src = cv::imread(argv[1], 1);
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);

    // wrap image data
    Image image(gray.cols, gray.rows, "Y800", gray.data, 
		gray.cols * gray.rows);

    // scan the image for barcodes
    int n = scanner.scan(image);
    std::cout << "n=" << n << std::endl;

    // extract results
    for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) {
        // do something useful with results
        cout << "decoded " << symbol->get_type_name()
             << " symbol \"" << symbol->get_data() << '"' << endl;
    }

    // clean up
    image.set_data(NULL, 0);

    return(0);
}
