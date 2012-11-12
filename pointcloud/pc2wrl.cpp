#include <vector>
#include <iostream>
#include <fstream>
#include <hrpUtil/Eigen3d.h>

using namespace hrp;

void ratio2color(double v, Vector3 &color)
{
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    if (v < 0.25){
        color << 0, v*4, 1.0;
    }else if (v < 0.5){
        color << 0, 1, (1.0-(v-0.25)*4);
    }else if (v < 0.75){
        color << (v-0.5)*4, 1, 0;
    }else{
        color << 1, (1.0-(v-0.75)*4), 0;
    }
}


int main(int argc, char *argv[])
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << "[point cloud data]" << std::endl;
        return 1;
    } 

    std::ifstream ifs(argv[1]);
    if (!ifs.is_open()){
        std::cerr << "failed to open(" << argv[1] << ")" << std::endl;
        return 2;
    }

    Vector3 p;
    Matrix33 R;
    Eigen::Quaternion<double> q;
    int h, min, max, res, nline=0;
    double v, deg, rad, minZ=std::numeric_limits<double>::max(), maxZ=std::numeric_limits<double>::min();
    Vector3 rel,abs;
    std::vector<Vector3> pc;
    while(!ifs.eof()){
        ifs >> p[0] >> p[1] >> p[2];
        for (int i=0; i<3; i++) ifs >> v;
        ifs >> q.x() >> q.y() >> q.z() >> q.w();
        R = q.matrix();
        //std::cerr << R << std::endl;
        for (int i=0; i<11; i++) ifs >> v;
        ifs >> h >> min >> max >> res;
        if (res != 1440) {
            std::cerr << "parse error?" << std::endl;
            break;
        }
        for (int i=0; i<1440; i++){
            ifs >> v;
            if (v>1){
                deg = -135+0.25*i;
                rad = deg*M_PI/180;
                v /= 1000;
                rel << v*cos(rad), v*sin(rad), 0;
                //std::cerr << rel.transpose() << std::endl;
                abs = p + R*rel;
                if (abs[2] > maxZ) maxZ = abs[2];
                if (abs[2] < minZ) minZ = abs[2];
                pc.push_back(abs);
            }
        }
        //std::cout << "(" << px << " " << py << " " << pz << "), (" << qx << " " << qy << " " << qz << " " << qw << ")" << std::endl;
        //std::cout << "h=" << h << std::endl;
        //std::cout << "min:" << min << ", max:" << max << ", res:" << res << std::endl; 
        nline++;
        //if (nline > 0) break;
    }
    // setup colors
    double dz = maxZ - minZ;
    std::vector<Vector3> colors;
    Vector3 color;
    for (size_t i=0; i<pc.size(); i++){
        ratio2color((pc[i][2]-minZ)/dz, color);
        colors.push_back(color);
    }
    std::cout << "#VRML V2.0 utf8" << std::endl;
    std::cout << "Shape {" << std::endl;
    std::cout << "  geometry PointSet {" << std::endl;
    std::cout << "    coord Coordinate {" << std::endl;
    std::cout << "      point [" << std::endl;
    for (size_t i=0; i<pc.size(); i++){
        std::cout << pc[i].transpose() << "," << std::endl;
    }
    std::cout << "      ]" << std::endl;
    std::cout << "    }" << std::endl;
    std::cout << "    color Color { color [" << std::endl;
    for (size_t i=0; i<colors.size(); i++){
        std::cout << colors[i].transpose() << "," << std::endl;
    }
    std::cout << "    ] }" << std::endl;
    std::cout << "  }" << std::endl;
    std::cout << "}" << std::endl;

    return 0;
}
