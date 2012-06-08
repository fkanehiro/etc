#include <iostream>
#include <sys/time.h>
#include <cstdio>

int main(int argc, char *argv[])
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
    char time[20];
    sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
    std::cout << time << std::endl;
    char buf[20];
    strftime(buf, 20, "%Y-%m-%d", tm_);
    std::cout << buf << std::endl;
}
