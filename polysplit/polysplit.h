#include <hrpModel/Body.h>
#include <hrpCollision/ColdetModel.h>

class anchor{
public:
    anchor(hrp::Link *i_link, const hrp::Vector3& i_point);
    hrp::Vector3 position() const;
    hrp::Link *link;
    hrp::Vector3 point;
};

void convertToConvexHull(hrp::BodyPtr i_body);
void convertToConvexHull(hrp::Link *i_link);
hrp::ColdetModelPtr convertToConvexHull(
    const std::vector<hrp::Link *>& i_links,
    const std::vector<anchor>& i_anchors);
std::vector<int> findAnchors(hrp::ColdetModelPtr coldetModel, 
                             const std::vector<anchor>& i_anchors);

