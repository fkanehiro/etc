#ifndef __EXEC_UTIL_H__
#define __EXEC_UTIL_H__

#include <Model/HumanoidBodyUtil.h>
#include <Math/Interpolator/ClampedCubicSplineArray.h>
#include <Math/Interpolator/PointToPointArray.h>

class DistanceConstraint;
class DistanceGeometry;
class SphereTree;
class Filter;

#if 1
typedef motion_interpolator::ClampedCubicSplineArray<int, hrp::dvector> dvInterpolator;
typedef motion_interpolator::ClampedCubicSplineArray<int, hrp::Vector3> v3Interpolator;
#else
typedef motion_interpolator::PointToPointArray<int, hrp::dvector> dvInterpolator;
typedef motion_interpolator::PointToPointArray<int, hrp::Vector3> v3Interpolator;
#endif

void createDistanceConstraints(SphereTree &i_stree,
                               const std::vector<DistanceGeometry *>& i_shapes,
                               std::vector<DistanceConstraint *>& o_consts);

void loadPath(const char *filename, int dof, int &arm,
              std::vector<hrp::dvector> &qPath,
              std::vector<hrp::Vector3> &pPath, 
              std::vector<hrp::Vector3> &rpyPath);

bool loadShape(const char *filename, motion_generator::HumanoidBodyPtr robot,
               std::vector<DistanceGeometry *>& shapes);

void setupSelfCollisionCheckPairs(const char *filename, 
                                  const std::vector<DistanceGeometry *>& shapes,
                                  Filter *filter);

int setupInterpolator(const std::vector<hrp::dvector>& qPath, 
                      const std::vector<hrp::Vector3>& pPath,
                      const std::vector<hrp::Vector3>& rpyPath,
                      dvInterpolator *qTrj,
                      v3Interpolator *pTrj,
                      v3Interpolator *rpyTrj);

#endif
