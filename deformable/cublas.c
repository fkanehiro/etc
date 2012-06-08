#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cublas.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sys/time.h>

#if 0
#define NF 8451
#define ND 24
#else
#define NF 1000
#define ND 2
#endif

//#define DPRECISION

#ifdef DPRECISION
#define matrixX MatrixXd
#define vectorX VectorXd
#define dtype double
#define func cublasDgemm
#else
#define matrixX MatrixXf
#define vectorX VectorXf
#define dtype float
#define func cublasSgemm
#endif

using namespace Eigen;

int main(int argc, char *argv[])
{
    matrixX fK(NF, NF), dK(NF,ND);
    vectorX uD(ND,1), eF(NF,1), uL(NF, 1);

    int val=0;
    for (int i=0; i<NF; i++){
        for (int j=0; j<NF; j++){
            fK(i,j) = val%5;
            val++;
        }
    }
    val = 0;
    for (int i=0; i<NF; i++){
        for (int j=0; j<ND; j++){
            dK(i,j) = val%10;
            val++; 
        }
    }
    val = 0;
    for (int i=0; i<ND; i++){
        uD(i,0) = val%10; 
        val++;
    }
    val = 0;
    for (int i=0; i<NF; i++){
        eF(i,0) = val%10;
        val++; 
    }

    //std::cout << "dK:" << std::endl << dK << std::endl;
    //std::cout << "uD:" << std::endl << uD << std::endl;
    //std::cout << "eF:" << std::endl << eF << std::endl;

    struct timeval t1, t2;
    matrixX ans;
    gettimeofday(&t1, NULL);
    ans = fK*(eF - dK*uD);
    gettimeofday(&t2, NULL);
    std::cout << "cpu:" << (t2.tv_sec - t1.tv_sec)*1e3 + (t2.tv_usec - t1.tv_usec)*1e-3 << "[ms]" << std::endl; 

    cublasStatus stat;
    dtype *devPtrfK, *devPtrdK, *devPtruD, *devPtreF, *devPtruL;
    
    cublasInit();

    stat = cublasAlloc (fK.cols()*fK.rows(), sizeof(dtype), (void**)&devPtrfK);
    if (stat != CUBLAS_STATUS_SUCCESS) {
        printf ("device memory allocation failed");
        return 1;
    }
    stat = cublasAlloc (dK.cols()*dK.rows(), sizeof(dtype), (void**)&devPtrdK);
    if (stat != CUBLAS_STATUS_SUCCESS) {
        printf ("device memory allocation failed");
        return 1;
    }
    stat = cublasAlloc (uD.cols()*uD.rows(), sizeof(dtype), (void**)&devPtruD);
    if (stat != CUBLAS_STATUS_SUCCESS) {
        printf ("device memory allocation failed");
        return 1;
    }
    stat = cublasAlloc (eF.cols()*eF.rows(), sizeof(dtype), (void**)&devPtreF);
    if (stat != CUBLAS_STATUS_SUCCESS) {
        printf ("device memory allocation failed");
        return 1;
    }
    stat = cublasAlloc (uL.cols()*uL.rows(), sizeof(dtype), (void**)&devPtruL);
    if (stat != CUBLAS_STATUS_SUCCESS) {
        printf ("device memory allocation failed");
        return 1;
    }


    cublasSetMatrix (fK.rows(), fK.cols(), sizeof(dtype), fK.data(), fK.rows(), devPtrfK, fK.rows());
    cublasSetMatrix (dK.rows(), dK.cols(), sizeof(dtype), dK.data(), dK.rows(), devPtrdK, dK.rows());

    gettimeofday(&t1, NULL);
    cublasSetMatrix (uD.rows(), uD.cols(), sizeof(dtype), uD.data(), uD.rows(), devPtruD, uD.rows());
    cublasSetMatrix (eF.rows(), eF.cols(), sizeof(dtype), eF.data(), eF.rows(), devPtreF, eF.rows());

    func('N', 'N', dK.rows(), uD.cols(), dK.cols(), -1.0, 
         devPtrdK, dK.rows(), devPtruD, uD.rows(), 1.0, 
         devPtreF, eF.rows());
    func('N', 'N', fK.rows(), eF.cols(), fK.cols(), 1.0, 
         devPtrfK, fK.rows(), devPtreF, eF.rows(), 0.0, 
         devPtruL, uL.rows());

    cublasGetMatrix (uL.rows(), uL.cols(), sizeof(dtype), devPtruL, uL.rows(),
                     uL.data(), uL.rows());
    gettimeofday(&t2, NULL);
    std::cout << "gpu:" << (t2.tv_sec - t1.tv_sec)*1e3 + (t2.tv_usec - t1.tv_usec)*1e-3 << "[ms]" << std::endl; 

    //std::cout << "ans:" << std::endl << ans << std::endl;
    //std::cout << "result:" << std::endl << uL << std::endl;
    dtype err =0;  
    for (int i=0; i<uL.rows(); i++) err+=(uL(i,0) - ans(i,0))*(uL(i,0) - ans(i,0));
    std::cout << "err:" << err << std::endl;

    cublasFree (devPtrfK);
    cublasFree (devPtrdK);
    cublasFree (devPtruD);
    cublasFree (devPtreF);
    cublasFree (devPtruL);
    cublasShutdown();
    
    return 0;
}
