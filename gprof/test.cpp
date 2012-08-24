#include <iostream>

int test(int n)
{
    int ret = 0;
    for (int i=0; i<n; i++){
        ret += i;
    }
    return ret;
}

int main(int argc, char *argv[])
{
    int ret = 0;
    for (int i=0; i<10000; i++){
        ret += test(i);
    }
    std::cout << "ret = " << ret << std::endl;
    return 0;
}

