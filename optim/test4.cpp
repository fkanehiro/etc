#include <iostream>
#include <Math/MathFunction.h>
#include <Solver/LinearSystem.h>
#include <Solver/QPsolver.h>
#include <hrpUtil/uBlasCommonTypes.h>
using namespace hrp;

int main(int argc, char *argv[])
{
    double x,v,a,t=0,dt=0.005;
    LinearSystem objective, vconst, aconst;

    int nparam = 1;
    QPsolver solver(nparam);
    objective.jacobian() = didentity(nparam,nparam);
    objective.value() = dvector(nparam);

    vconst.jacobian() = dzeromatrix(2,nparam);
    vconst.jacobian()(0,0) =  1.0; // upper bound
    vconst.jacobian()(1,0) = -1.0; // lower bound
    vconst.value() = dvector(2);

    aconst.jacobian() = vconst.jacobian();
    aconst.value() = dvector(2);

    solver.addObjectiveFunction(&objective);
    solver.addIneqConstraint(&vconst);
    //solver.addIneqConstraint(&aconst);

    dvector solution(nparam);
    double xx = 0,vv=0,aa;

    const double xlimit = 0.8, vlimit = 1.5, alimit = 10.0;

    while(t < 1.0){
        x = polynomial5(t);
        v = dpolynomial5(t);
        a = ddpolynomial5(t);

        //objective.value()(0) = vv+a*dt;
        objective.value()(0) = v*dt;
        for (int i=0; i<nparam; i++) solution(i) = 0.0;
        solver.clearBounds();

        double vmax = vlimit, vmin = vlimit; // absolute value
#if 1
        // velocity damper
        const double x_i = 0.6, x_s = xlimit;
        if (xx > x_i){
            vmax = vlimit * (x_s - xx)/(x_s - x_i);
        }else if (xx < -x_i){
            vmin = vlimit * (x_s + xx)/(x_s - x_i);
        }
#endif
        //std::cerr << "vlimit:(" << vmax << ", -" << vmin << ")" << std::endl;
        vconst.value()(0) = vmax*dt;
        vconst.value()(1) = vmin*dt;

        // acceleration limit
        double amax = alimit , amin = alimit; // absolute value 
#if 0
        // acceleration damper
        const double v_i = 1.0;
        if (vv > v_i){
            amax = alimit * (vlimit - vv)/(vlimit - v_i);
        }else if (vv < -v_i){
            amin = alimit * (vlimit + vv)/(vlimit - v_i);
        }
#endif
        //std::cerr << "[" << -vmin <<"," << vmax << "] <-> [" << (vv-amin*dt) << "," << (vv+amax*dt) <<"]" << std::endl;
        aconst.value()(0) =  (vv + amax*dt)*dt;
        aconst.value()(1) = -(vv - amin*dt)*dt;
        if (vv+amax*dt < -vmin||vmax < -(vv - amin*dt)){
            std::cerr << "vlimit and alimit are incompatible" << std::endl;
        }

        int ret = solver.solve(solution);
        if (ret != 0){
            std::cerr << "t = " << t << ", information from QPsolver = " << ret << std::endl;
            std::cerr << "J of obj:" << objective.jacobian() << std::endl;
            std::cerr << "v of obj:" << objective.value() << std::endl;
            std::cerr << "J of vconst:" << vconst.jacobian() << std::endl;
            std::cerr << "v of vconst:" << vconst.value() << std::endl;
            std::cerr << "J of aconst:" << aconst.jacobian() << std::endl;
            std::cerr << "v of aconst:" << aconst.value() << std::endl;
	    std::cerr << "vv:" << vv << std::endl;
            break;
        }
        std::cerr << "solution:" << solution << std::endl;

        double DT = dt;
        double vv_old = vv;
        vv = solution(0)/DT;
        aa = (vv - vv_old)/DT;
        xx += solution(0);

        std::cout << t << " " << x << " " << v << " " << a << " " 
                  << xx << " " << vv << " " << aa << std::endl; 
        
        t += DT;

    }
    return 0;
}
