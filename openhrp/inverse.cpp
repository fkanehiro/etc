#include <iostream>
#include <sys/time.h>
#include <hrpUtil/Tvmet3d.h>

using namespace hrp;

int main(int argc, char *argv[])
{
    Matrix33 M, Minv;

    M << 0,0,0,0,0,0,0,0,0;
    std::cout << "M:" << std::endl << M << std::endl;
    try{
        Minv = inverse(M);
        std::cout << "Minv:" << std::endl << Minv << std::endl;
    }catch(std::string &ex){
        std::cout << ex << std::endl;
    }

    M << 1,0,0,0,1,0,0,0,1;
    std::cout << "M:" << std::endl << M << std::endl;
    try{
        Minv = inverse(M);
        std::cout << "Minv:" << std::endl << Minv << std::endl;
    }catch(std::string &ex){
        std::cout << ex << std::endl;
    }


    return 0;
}
