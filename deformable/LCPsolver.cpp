#include "LCPsolver.h"
#include <iostream>

using namespace std;
using namespace hrp;

static const bool ENABLE_STATIC_FRICTION = true;
static const bool ONLY_STATIC_FRICTION_FORMULATION = (true && ENABLE_STATIC_FRICTION);
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;
static const bool ENABLE_TRUE_FRICTION_CONE =
    (true && ONLY_STATIC_FRICTION_FORMULATION && STATIC_FRICTION_BY_TWO_CONSTRAINTS);

static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 10;
static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 500;
//static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-8;
static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-3;

static const bool CFS_MCP_DEBUG = false;
static const bool CFS_MCP_DEBUG_SHOW_ITERATION_STOP = false;

int numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
int maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
double gaussSeidelMaxRelError = DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR;

int numGaussSeidelTotalLoops = 0;
int numGaussSeidelTotalCalls = 0;

int globalNumContactNormalVectors;
int globalNumFrictionVectors=0;
int globalNumConstraintVectors = globalNumContactNormalVectors + globalNumFrictionVectors;
std::vector<int> frictionIndexToContactIndex;
dvector contactIndexToMu(globalNumContactNormalVectors);
dvector mcpHi(globalNumContactNormalVectors);

void solveMCPByProjectedGaussSeidelInitial
(const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration)
{
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


void solveMCPByProjectedGaussSeidelMain
(const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

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
                x(j) = xx;
            }
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

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


        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double fx0;
                if(M(j,j)==numeric_limits<double>::max())
                    fx0=0.0;
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
                    fy0=0.0;
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
            }
        }
    }
}


double solveMCPByProjectedGaussSeidelErrorCheck
(const rmdmatrix& M, const dvector& b, dvector& x)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    double error = 0.0;

    for(int j=0; j < globalNumConstraintVectors; ++j){

        double sum = -M(j, j) * x(j);
        for(int k=0; k < size; ++k){
            sum += M(j, k) * x(k);
        }
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

        int contactIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            double fx0 = (-b(j) - sum) / M(j, j);
            double& fx = x(j);

            ++j;

            sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            double fy0 = (-b(j) - sum) / M(j, j);
            double& fy = x(j);

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

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }

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

bool hrp::solveMCPByProjectedGaussSeidel(const rmdmatrix& M, const dvector& b, dvector& x)
{
    globalNumContactNormalVectors = M.size1();
    globalNumConstraintVectors = globalNumContactNormalVectors + globalNumFrictionVectors;
    contactIndexToMu.resize(globalNumContactNormalVectors);
    mcpHi.resize(globalNumContactNormalVectors);

    static const int loopBlockSize = DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK;

    if(numGaussSeidelInitialIteration > 0){
        solveMCPByProjectedGaussSeidelInitial(M, b, x, numGaussSeidelInitialIteration);
    }

    int numBlockLoops = maxNumGaussSeidelIteration / loopBlockSize;
    if(numBlockLoops==0) numBlockLoops = 1;

    if(CFS_MCP_DEBUG) cout << "Iteration ";

    double error = 0.0;
    int i=0;
    while(i < numBlockLoops){
        i++;
        solveMCPByProjectedGaussSeidelMain(M, b, x, loopBlockSize - 1);

        error = solveMCPByProjectedGaussSeidelErrorCheck(M, b, x);
	std::cout << i << ": error = " << error << std::endl;
        if(error < gaussSeidelMaxRelError){
            if(CFS_MCP_DEBUG_SHOW_ITERATION_STOP) cout << "stopped at " << i * loopBlockSize << endl;
            break;
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
    return loopBlockSize*i != maxNumGaussSeidelIteration;
}


#if 0
int main(int argc, char *argv)
{
    rmdmatrix M(2,2);
    dvector b(2);
    dvector x(2);

    M(0,0) = 1.0; M(0,1) = 2.0;
    M(1,0) = 3.0; M(1,1) = 4.0;

    b(0) = -2.0;
    b(1) = -4.0;

    x.clear();

    solveMCPByProjectedGaussSeidel(M, b, x);

    std::cout << "x = " << x << std::endl;
    std::cout << "Mx+b = " << prod(M,x)+b << std::endl;

    return 0;
}
#endif
