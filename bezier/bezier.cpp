#include <iostream>

int main(int argc, char *argv[])
{
    double b0[] = {0,0};
    double b1[] = {3,5};
    double b2[] = {6,4};
    double b3[] = {8,1};
    double p[2]; 
    for (double t = 0; t<=1.0; t+=0.01){
        double s = 1 - t;
        for (int i=0; i<2; i++){
            p[i] = b0[i]*s*s*s + 3*s*s*t*b1[i] + 3*s*t*t*b2[i] + t*t*t*b3[i];
        }
        std::cout << p[0] << " " << p[1]  << std::endl;  
    }
    return 0;
}
