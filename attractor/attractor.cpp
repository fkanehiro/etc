#include <iostream>

int main(int argc, char *argv[])
{
    int K = 3, k = 0;
    double xstar[] = {0,1,-2};
    double T = 5, dt = 0.01, t = 0;
    double tsec = T/(K-1);
    double Tmc = 0.2;
    double xi = 1;
    double a = dt*dt/(Tmc*Tmc + 2*Tmc*dt*xi + dt*dt);
    double b = Tmc*Tmc/(Tmc*Tmc + 2*Tmc*dt*xi + dt*dt);
    double x=xstar[0], x_old=xstar[0], r=xstar[0];

    while (t < T){
        std::cout << t << " " << k << " " << r << " " << x << std::endl;
        t += dt;
        int k = t/tsec;
        double tau = (t - k*tsec)/tsec;
        r = (1-tau)*xstar[k] + tau*xstar[k+1];
        double pi = a*(r - x) + b*(x - x_old);
        x_old = x;
        x = x + pi;
    }

    return 0;
}
