#include <hrpUtil/Eigen3d.h>
#include <vector>

void loadPointArray(const char *i_filename, 
                    std::vector<hrp::Vector3> &o_points);

void addObstaclePoints(std::vector<hrp::Vector3>& points, 
                       const hrp::Vector3& p);
