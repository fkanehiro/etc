#include <boost/numeric/ublas/matrix.hpp>
#include <hrpUtil/uBlasCommonTypes.h>
#include <Eigen/Sparse>
//#include "constraintConfig.h"

static const bool ENABLE_STATIC_FRICTION = true;
static const bool ONLY_STATIC_FRICTION_FORMULATION = (true && ENABLE_STATIC_FRICTION);
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;

using namespace boost::numeric::ublas;
using namespace hrp;
using namespace std;
using namespace Eigen;

//#define USE_EIGEN

typedef boost::numeric::ublas::matrix<double, row_major> rmdmatrix;

class LCPSolver
{
    public:
        LCPSolver();
        void setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError);
        void setMaxNumIteration(int maxNumIteration);
        void initial();
        void setSize(int globalNumContactNormalVectors, int globalNumConstraintVectors, int globalNumFrictionVectors);
        bool solveMCPByProjectedGaussSeidel(const rmdmatrix& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x);
        void solveMCPByProjectedGaussSeidelInitial(const rmdmatrix& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x, const int numIteration);
#ifdef USE_EIGEN
        void solveMCPByProjectedGaussSeidelMain(const MatrixXd& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, VectorXd& x, const int numIteration);
        double solveMCPByProjectedGaussSeidelErrorCheck(const MatrixXd& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, VectorXd& x);
#else
        void solveMCPByProjectedGaussSeidelMain(const rmdmatrix& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x, const int numIteration);
        double solveMCPByProjectedGaussSeidelErrorCheck(const rmdmatrix& M, const dvector& b, 
            const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x);
#endif
        void checkLCPResult(const rmdmatrix& M, const dvector& b, const dvector& x);
        bool checkMCPResult(const rmdmatrix& M, const dvector& b, const dvector& contactIndexToMu, const dvector& x, bool verbose=false);
        bool checkMCPResult(const double *x, const double *z, const dvector& contactIndexToMu, bool verbose=false);
        
        bool solveSparseLEByGaussSeidel(
            const DynamicSparseMatrix<double, RowMajor>& A,
            const VectorXd& b, VectorXd& x,
            int numIteration=1); 
        bool solveSparseMCPByProjectedGaussSeidel(
            const SparseMatrix<double, RowMajor>& B, const VectorXd& c, 
            const std::vector<int>& frictionIndexToContactIndex, 
            const dvector& contactIndexToMu, VectorXd& f, 
            int numIteration=1);
#ifdef NEW_CONSTRAINT
        bool solveMCPByPATH(const rmdmatrix& M, const dvector& b, const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x);
#endif
  private:
        int  maxNumGaussSeidelIteration;
        int  numGaussSeidelInitialIteration;
        double gaussSeidelMaxRelError;

        int globalNumConstraintVectors;
        int globalNumFrictionVectors;
        int globalNumContactNormalVectors;
        dvector mcpHi;

        int numGaussSeidelTotalLoops;
        int numGaussSeidelTotalCalls;
};

#ifdef USE_EIGEN
static inline double computeSum(const MatrixXd& M, const VectorXd& x, int j)
#else
static inline double computeSum(const rmdmatrix& M, const dvector& x, int j)
#endif
{
#ifdef VECTORIZE
    double sum = M.col(j).transpose()*x - M(j,j)*x(j);
#else
    double sum = -M(j, j) * x(j);
    for(int k=0; k < x.size(); ++k){
#ifdef USE_EIGEN
        sum += M(k, j) * x(k);
#else
        sum += M(j, k) * x(k);
#endif
    }
#endif
    return sum;
}

