#include <hrpModel/Body.h>
#include "CdShape.h"
#include "SphereTree.h"

void loadCdShapes(hrp::BodyPtr i_body, const char *i_filename,
                  std::vector<CdShape> &o_shapes);

void loadCdPairs(const std::vector<CdShape> &i_shapes,
                 const char *i_filename, 
                 std::vector<std::pair<const CdShape *, const CdShape *> > &o_pairs);

bool checkCollision(const std::vector<std::pair<const CdShape *, const CdShape *> > &i_pairs);
bool checkCollision(const SphereTree &i_st, const std::vector<CdShape>& i_shapes);
bool checkCollision(const SphereTree &i_st, const std::vector<CdShape>& i_shapes,
                    const std::vector<std::pair<const CdShape *, const CdShape *> > &i_pairs);




