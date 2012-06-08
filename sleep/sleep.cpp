#include <iostream>
#include <sys/time.h>

int main(int argc, char *argv[])
{
    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);
    usleep(5000);
    gettimeofday(&tv2, NULL);
    std::cout << "usleep(5000) : " 
              << (tv2.tv_sec - tv1.tv_sec)*1000+(tv2.tv_usec - tv1.tv_usec)/1000.0 << "[ms]" << std::endl;

    gettimeofday(&tv1, NULL);
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 5000*1000;
    nanosleep(&ts,NULL);
    gettimeofday(&tv2, NULL);
    std::cout << "nanosleep(5000*1000) : " 
              << (tv2.tv_sec - tv1.tv_sec)*1000+(tv2.tv_usec - tv1.tv_usec)/1000.0 << "[ms]" << std::endl;

    return 0;
}
