#include "LCPSolver.h"



static const bool ENABLE_TRUE_FRICTION_CONE =
    (true && ONLY_STATIC_FRICTION_FORMULATION && STATIC_FRICTION_BY_TWO_CONSTRAINTS);

static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 5000;
static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 10;
static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-8;

static const bool CFS_MCP_DEBUG = false;
static const bool CFS_MCP_DEBUG_SHOW_ITERATION_STOP = false;

LCPSolver::LCPSolver()
{
    maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
    numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
    gaussSeidelMaxRelError = DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR;
}

void LCPSolver::setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError)
{
    maxNumGaussSeidelIteration = maxNumIteration;
    numGaussSeidelInitialIteration = numInitialIteration;
    gaussSeidelMaxRelError = maxRelError;
}

void LCPSolver::setMaxNumIteration(int maxNumIteration)
{
    maxNumGaussSeidelIteration = maxNumIteration;
}

void LCPSolver::initial()
{
    if(CFS_MCP_DEBUG){
        numGaussSeidelTotalCalls = 0;
        numGaussSeidelTotalLoops = 0;
    }
}

void LCPSolver::setSize(int _globalNumContactNormalVectors, int _globalNumConstraintVectors, int _globalNumFrictionVectors)
{
    globalNumContactNormalVectors = _globalNumContactNormalVectors;
    globalNumConstraintVectors = _globalNumConstraintVectors;
    globalNumFrictionVectors = _globalNumFrictionVectors;
}

bool LCPSolver::solveMCPByProjectedGaussSeidel(const rmdmatrix& M, const dvector& b, 
        const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x)
{
    static const int loopBlockSize = DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK;

    if(numGaussSeidelInitialIteration > 0){
        solveMCPByProjectedGaussSeidelInitial(M, b, frictionIndexToContactIndex, contactIndexToMu, x, numGaussSeidelInitialIteration);
    }

    int numBlockLoops = maxNumGaussSeidelIteration / loopBlockSize;
    if(numBlockLoops==0) numBlockLoops = 1;

    if(CFS_MCP_DEBUG) cout << "Iteration ";

#ifdef USE_EIGEN
    MatrixXd Me(M.size1(), M.size2());
    for (unsigned int i=0; i<M.size1(); i++){
        for (unsigned int j=0; j<M.size2(); j++){
            Me(j,i) = M(i,j); // transpose to make column major 
        }
    }
    VectorXd xe(x.size());
    for (unsigned int i=0; i<x.size(); i++) xe[i] = x[i];
#endif
    double error = 0.0;
    bool isConverged = false;
    int i=0;
    while(i < numBlockLoops){
        i++;
#ifdef USE_EIGEN
        solveMCPByProjectedGaussSeidelMain(Me, b, frictionIndexToContactIndex, contactIndexToMu, xe, loopBlockSize - 1);
#else
        solveMCPByProjectedGaussSeidelMain(M, b, frictionIndexToContactIndex, contactIndexToMu, x, loopBlockSize - 1);
#endif

#ifdef USE_EIGEN
        error = solveMCPByProjectedGaussSeidelErrorCheck(Me, b, frictionIndexToContactIndex, contactIndexToMu, xe);
#else
        error = solveMCPByProjectedGaussSeidelErrorCheck(M, b, frictionIndexToContactIndex, contactIndexToMu, x);
#endif
        if(error < gaussSeidelMaxRelError){
            if(CFS_MCP_DEBUG_SHOW_ITERATION_STOP) cout << "stopped at " << i * loopBlockSize << endl;
            isConverged = true;
            //break;
        }
    }

    if(CFS_MCP_DEBUG){
        int n = loopBlockSize * i;
        numGaussSeidelTotalLoops += n;
        numGaussSeidelTotalCalls++;
        cout << n;
        cout << ", avarage = " << (numGaussSeidelTotalLoops / numGaussSeidelTotalCalls);
        cout << ", error = " << error;
        cout << endl;
    }
#ifdef USE_EIGEN
    for (unsigned int i=0; i<x.size(); i++) x[i] = xe[i];
#endif

    return isConverged;
}

void LCPSolver::solveMCPByProjectedGaussSeidelInitial(const rmdmatrix& M, const dvector& b, 
    const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x, const int numIteration)
{
    mcpHi.resize(globalNumContactNormalVectors, false);
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    const double rstep = 1.0 / (numIteration * size);
    double r = 0.0;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

             double xx;
            if(M(j,j)==numeric_limits<double>::max())
                xx=0.0;
            else{
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                xx = (-b(j) - sum) / M(j, j);
            }
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = r * xx;
            }
            r += rstep;
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

            if(M(j,j)==numeric_limits<double>::max())
                x(j) = 0.0;
            else{
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                x(j) = r * (-b(j) - sum) / M(j, j);
            }
            r += rstep;
        }

        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double fx0;
                if(M(j,j)==numeric_limits<double>::max())
                    fx0 = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    fx0 = (-b(j) - sum) / M(j, j);
                }
                double& fx = x(j);

                ++j;

                 double fy0;
                if(M(j,j)==numeric_limits<double>::max())
                    fy0 = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    fy0 = (-b(j) - sum) / M(j, j);
                }
                double& fy = x(j);

                const double fmax = mcpHi[contactIndex];
                const double fmax2 = fmax * fmax;
                const double fmag2 = fx0 * fx0 + fy0 * fy0;

                if(fmag2 > fmax2){
                    const double s = r * fmax / sqrt(fmag2);
                    fx = s * fx0;
                    fy = s * fy0;
                } else {
                    fx = r * fx0;
                    fy = r * fy0;
                }
                r += (rstep + rstep);
            }

        } else {

            int frictionIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

                double xx;
                if(M(j,j)==numeric_limits<double>::max())
                    xx = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    xx = (-b(j) - sum) / M(j, j);
                }

                const int contactIndex = frictionIndexToContactIndex[frictionIndex];
                const double fmax = mcpHi[contactIndex];
                const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

                if(xx < fmin){
                    x(j) = fmin;
                } else if(xx > fmax){
                    x(j) = fmax;
                } else {
                    x(j) = xx;
                }
                x(j) *= r;
                r += rstep;
            }
        }
    }
}

#ifdef USE_EIGEN
void LCPSolver::solveMCPByProjectedGaussSeidelMain(const MatrixXd& M, const dvector& b, 
    const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, VectorXd& x, const int numIteration)
#else
void LCPSolver::solveMCPByProjectedGaussSeidelMain(const rmdmatrix& M, const dvector& b, 
    const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x, const int numIteration)
#endif
{
    mcpHi.resize(globalNumContactNormalVectors, false);
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double xx;
            if(M(j,j)==numeric_limits<double>::max())
                xx=0.0;
            else{
                double sum = computeSum(M,x,j);
                xx = (-b(j) - sum) / M(j, j);
            }
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = xx;
            }
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

#if 0
        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

            if(M(j,j)==numeric_limits<double>::max())
                x(j)=0.0;
            else{
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                x(j) = (-b(j) - sum) / M(j, j);
            }
        }
#endif


        if(ENABLE_TRUE_FRICTION_CONE){

            int frictionIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

                double fx0;
                if(M(j,j)==numeric_limits<double>::max())
                    fx0=0.0;
                else{
                    double sum = computeSum(M,x,j);
                    fx0 = (-b(j) - sum) / M(j, j);
                }
                double& fx = x(j);

                ++j;
                ++frictionIndex;

                double fy0;
                if(M(j,j)==numeric_limits<double>::max())
                    fy0=0.0;
                else{
                    double sum = computeSum(M,x,j);
                    fy0 = (-b(j) - sum) / M(j, j);
                }
                double& fy = x(j);
                
                const int contactIndex = frictionIndexToContactIndex[frictionIndex];
                const double fmax = mcpHi[contactIndex];
                const double fmax2 = fmax * fmax;
                const double fmag2 = fx0 * fx0 + fy0 * fy0;

                if(fmag2 > fmax2){
                    const double s = fmax / sqrt(fmag2);
                    fx = s * fx0;
                    fy = s * fy0;
                } else {
                    fx = fx0;
                    fy = fy0;
                }
            }

        } else {

            int frictionIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

                double xx;
                if(M(j,j)==numeric_limits<double>::max())
                    xx=0.0;
                else{
                    double sum = computeSum(M,x,j);
                    xx = (-b(j) - sum) / M(j, j);
                }

                const int contactIndex = frictionIndexToContactIndex[frictionIndex];
                const double fmax = mcpHi[contactIndex];
                const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

                if(xx < fmin){
                    x(j) = fmin;
                } else if(xx > fmax){
                    x(j) = fmax;
                } else {
                    x(j) = xx;
                }
            }
        }
    }
}

#ifdef USE_EIGEN
double LCPSolver::solveMCPByProjectedGaussSeidelErrorCheck(const MatrixXd& M, const dvector& b,
    const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, VectorXd& x)
#else
double LCPSolver::solveMCPByProjectedGaussSeidelErrorCheck(const rmdmatrix& M, const dvector& b,
    const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x)
#endif
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    double error = 0.0;

    for(int j=0; j < globalNumConstraintVectors; ++j){

        double sum = computeSum(M,x,j);
        double xx = (-b(j) - sum) / M(j, j);

        if(j < globalNumContactNormalVectors){
            if(xx < 0.0){
                xx = 0.0;
            }
            mcpHi[j] = contactIndexToMu[j] * xx;
        }
        double d = fabs(xx - x(j));
        if(xx > numeric_limits<double>::epsilon()){
            d /= xx;
        }
        if(d > error){
            error = d;
        }
        x(j) = xx;
    }

    if(ENABLE_TRUE_FRICTION_CONE){

        int frictionIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

            double sum = computeSum(M,x,j);
            double fx0 = (-b(j) - sum) / M(j, j);
            double& fx = x(j);

            ++j;
            ++frictionIndex;

            sum = computeSum(M,x,j);
            double fy0 = (-b(j) - sum) / M(j, j);
            double& fy = x(j);

            const int contactIndex = frictionIndexToContactIndex[frictionIndex];
            const double fmax = mcpHi[contactIndex];
            const double fmax2 = fmax * fmax;
            const double fmag2 = fx0 * fx0 + fy0 * fy0;

            if(fmag2 > fmax2){
                const double s = fmax / sqrt(fmag2);
                fx0 *= s;
                fy0 *= s;
            }
			
            double d = fabs(fx0 - fx);
            const double afx0 = fabs(fx0);
            if(afx0 > numeric_limits<double>::epsilon()){
                d /= afx0;
            }
            if(d > error){
                error = d;
            }
            d = fabs(fy0 - fy);
            const double afy0 = fabs(fy0);
            if(afy0 > numeric_limits<double>::epsilon()){
                d /= afy0;
            }
            if(d > error){
                error = d;
            }
            fx = fx0;
            fy = fy0;
        }

    } else {

        int frictionIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

            double sum = computeSum(M,x,j);
            double xx = (-b(j) - sum) / M(j, j);

            const int contactIndex = frictionIndexToContactIndex[frictionIndex];
            const double fmax = mcpHi[contactIndex];
            const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

            if(xx < fmin){
                xx = fmin;
            } else if(xx > fmax){
                xx = fmax;
            }
            double d = fabs(xx - x(j));
            if(xx > numeric_limits<double>::epsilon()){
                d /= xx;
            }
            if(d > error){
                error = d;
            }
            x(j) = xx;
        }
    }

    return error;
}


void LCPSolver::checkLCPResult(const rmdmatrix& M, const dvector& b, const dvector& x)
{
    std::cout << "check LCP result\n";
    std::cout << "-------------------------------\n";

    dvector z = prod(M, x) + b;

    int n = x.size();
    for(int i=0; i < n; ++i){
        std::cout << "(" << x(i) << ", " << z(i) << ")";

        if(x(i) < 0.0 || z(i) < 0.0 || x(i) * z(i) != 0.0){
            std::cout << " - X";
        }
        std::cout << "\n";

        if(i == globalNumConstraintVectors){
            std::cout << "-------------------------------\n";
        } else if(i == globalNumConstraintVectors + globalNumFrictionVectors){
            std::cout << "-------------------------------\n";
        }
    }

    std::cout << "-------------------------------\n";


    std::cout << std::endl;
}

bool LCPSolver::checkMCPResult(const rmdmatrix& M, const dvector& b, const dvector& contactIndexToMu, const dvector& x, bool verbose)
{
    dvector z = prod(M, x) + b;
    return checkMCPResult(&x[0], &z[0], contactIndexToMu, verbose);
}

bool LCPSolver::checkMCPResult(const double *x, const double *z, const dvector& contactIndexToMu, bool verbose)
{
    bool ret = true;
    if (verbose){
        std::cout << "check MCP result\n";
        std::cout << "-------------------------------\n";
    }

    for(int i=0; i < globalNumConstraintVectors; ++i){
        if (verbose) std::cout << "(" << x[i] << ", " << z[i] << ")";

        if(x[i] < 0.0 || z[i] < -1.0e-6){
            ret = false;
            if (verbose) std::cout << " - X";
        } else if(x[i] > 0.0 && fabs(z[i]) > 1.0e-6){
            ret = false;
            if (verbose) std::cout << " - X";
        } else if(z[i] > 1.0e-6 && fabs(x[i]) > 1.0e-6){
            ret = false;
            if (verbose) std::cout << " - X";
        }


        if (verbose) std::cout << "\n";
    }

    if (verbose) std::cout << "-------------------------------\n";

    int j = 0;
    for(int i=globalNumConstraintVectors; i < globalNumConstraintVectors + globalNumFrictionVectors; ++i, ++j){

        double hi = contactIndexToMu[j] * x[j];
        double z1 = z[i], z2 = z[i+1];
        double ft = x[i], fs = x[++i];
        double normf = sqrt(ft*ft+fs*fs);
        
        if (verbose) {
            std::cout << "(" << normf << ", " << z1 << ", " << z2 << ")"
                      << " hi = " << hi;
        }
        if(normf > hi+1.0e-6){
            ret = false;
            if (verbose) std::cout << " - X";
        } else if(normf < hi-1.0e-6 && (fabs(z1) > 1.0e-6 || fabs(z2) > 1.0e-6)){
            ret = false;
            if (verbose) std::cout << " - X";
        }
        if (verbose) std::cout << "\n";
    }

    if (verbose) std::cout << "-------------------------------\n";

    if (verbose) std::cout << std::endl;

    return ret;
}

// solve Ax + b = 0
bool LCPSolver::solveSparseLEByGaussSeidel(
    const DynamicSparseMatrix<double, RowMajor>& A,
    const VectorXd& b, VectorXd& x,
    int numIteration)
{
    if (b.size() != A.outerSize()){
        std::cout << "b.size() != A.outerSize()" << std::endl;
    }

    for(int i=0; i < numIteration; ++i){
        for(int j=0; j < A.outerSize(); ++j){
            double sum = 0, diagvalue=0;
            for(DynamicSparseMatrix<double, RowMajor>::InnerIterator it(A,j); it; ++it){
                int c = it.col();
                if (c == j){
                    diagvalue = it.value();
                }else{
                    sum += it.value()*x(c);
                }
            }
            if (!diagvalue) std::cout << "diagonal value = 0" << std::endl;
            x(j) = (-b(j) - sum) / diagvalue;
        }

    }
    return true;
}

bool LCPSolver::solveSparseMCPByProjectedGaussSeidel(
    const SparseMatrix<double, RowMajor>& B, const VectorXd& c, 
    const std::vector<int>& frictionIndexToContactIndex, 
    const dvector& contactIndexToMu, VectorXd& f,
    int numIteration)
{
    mcpHi.resize(globalNumContactNormalVectors, false);
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double xx;
            double sum = 0, diagvalue = 0;
            for(SparseMatrix<double, RowMajor>::InnerIterator it(B, j); it; ++it){
                int c = it.col();
                if (c == j){
                    diagvalue = it.value();
                }else{
                    sum += it.value() * f(c);
                }
            }
            if (!diagvalue) std::cout << "diagonal value = 0" << std::endl;
            xx = (-c(j) - sum) / diagvalue;
            if(xx < 0.0){
                f(j) = 0.0;
            } else {
                f(j) = xx;
            }
            mcpHi[j] = contactIndexToMu[j] * f(j);
        }

        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double sum = 0, diagvalue = 0;
                for(SparseMatrix<double, RowMajor>::InnerIterator it(B, j); it; ++it){
                    int c = it.col();
                    if (c == j){
                        diagvalue = it.value();
                    }else{
                        sum += it.value() * f(c);
                    }
                }
                if (!diagvalue) std::cout << "diagonal value = 0" << std::endl;
                double fx0 = (-c(j) - sum) / diagvalue;
                double& fx = f(j);

                ++j;

                sum = 0; diagvalue = 0;
                for(SparseMatrix<double, RowMajor>::InnerIterator it(B, j); it; ++it){
                    int c = it.col();
                    if (c == j){
                        diagvalue = it.value();
                    }else{
                        sum += it.value() * f(c);
                    }
                }
                if (!diagvalue) std::cout << "diagonal value = 0" << std::endl;
                double fy0 = (-c(j) - sum) / diagvalue;
                double& fy = f(j);

                const double fmax = mcpHi[contactIndex];
                const double fmax2 = fmax * fmax;
                const double fmag2 = fx0 * fx0 + fy0 * fy0;

                if(fmag2 > fmax2){
                    const double s = fmax / sqrt(fmag2);
                    fx = s * fx0;
                    fy = s * fy0;
                } else {
                    fx = fx0;
                    fy = fy0;
                }
            }

        } else {
#if 0 // todo : support if necessary

            int frictionIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

                double xx;
                if(B(j,j)==numeric_limits<double>::max())
                    xx=0.0;
                else{
                    double sum = -B(j, j) * f(j);
                    for(int k=0; k < size; ++k){
                        sum += B(j, k) * f(k);
                    }
                    xx = (-c(j) - sum) / B(j, j);
                }

                const int contactIndex = frictionIndexToContactIndex[frictionIndex];
                const double fmax = mcpHi[contactIndex];
                const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

                if(xx < fmin){
                    f(j) = fmin;
                } else if(xx > fmax){
                    f(j) = fmax;
                } else {
                    f(j) = xx;
                }
            }
#endif
        }
    }
    return true;
}

#ifdef NEW_CONSTRAINT
#include "SimpleLCP.h"
bool LCPSolver::solveMCPByPATH(const rmdmatrix& M, const dvector& b, 
        const std::vector<int>& frictionIndexToContactIndex, const dvector& contactIndexToMu, dvector& x)
{
     MCP_Termination termination;

     int variables;

     double *x_end;

     int m_nnz;
     int *m_i;
     int *m_j;
     double *m_ij;
     double *q;

     double *lb;
     double *ub;
    
     int i;

     variables = b.size();

     x_end = (double *)malloc(sizeof(double)*variables + 1);

     m_nnz = 0;
     for(int i=0; i<M.size1(); i++){
         for(int j=0; j<M.size2(); j++)
             if(M(i,j)!=0.0)
                 m_nnz++;
     }

     m_i = (int *)malloc(sizeof(int)*m_nnz + 1);
     m_j = (int *)malloc(sizeof(int)*m_nnz + 1);
     m_ij = (double *)malloc(sizeof(double)*m_nnz + 1);
     q = (double *)malloc(sizeof(double)*variables + 1);

     int ii = 0;
     for(int i=0; i<M.size1(); i++)
         for(int j=0; j<M.size2(); j++)
             if(M(i,j)!=0.0){
                m_i[ii] = i+1;
                m_j[ii] = j+1;
                m_ij[ii] = M(i,j);
                ii++;
             }

     for (i = 0; i < variables; i++)
     {
        x_end[i] = 0.0;
        q[i] = b(i);
     }
 
     lb = (double *)malloc(sizeof(double)*variables + 1);
     ub = (double *)malloc(sizeof(double)*variables + 1);

     for (i = 0; i < variables; i++)
     {
        lb[i] = 0.0;
        ub[i] = 1e20;
     }

     SimpleLCP(variables, m_nnz, m_i, m_j, m_ij, q, lb, ub, 
               &termination, x_end);

     if (termination == MCP_Error)
     { 
         std::cout << "Error in the solution." <<std::endl;
         return false;
     } else if (termination == MCP_Solved)
     {
       //   std::cout << "LCP Solved."<<std::endl;
     } else
     {
          std::cout << "Other error: " << termination <<std::endl;
          return false;
     }

     for (i = 0; i < variables; i++)
     {
        x(i) = x_end[i];
     }

     free(x_end);

     free(m_i);
     free(m_j);
     free(m_ij);
     free(q);

     free(lb);
     free(ub);
     return true;
}
#endif
