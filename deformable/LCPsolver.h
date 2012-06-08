#include <hrpUtil/uBlasCommonTypes.h>

namespace hrp{
    typedef boost::numeric::ublas::matrix<double, ublas::row_major> rmdmatrix;

    bool solveMCPByProjectedGaussSeidel(const rmdmatrix& M, const hrp::dvector& b, hrp::dvector& x);
};


