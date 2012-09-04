#include <hrpModel/Body.h>
#include <hrpCollision/ColdetModel.h>

class Anchor{
public:
    Anchor(hrp::Link *i_link, const hrp::Vector3& i_point);
    hrp::Vector3 position() const;
    hrp::Link *link;
    hrp::Vector3 point;
};

typedef std::pair<Anchor *, Anchor *> String;
typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> LineSegment;
typedef std::vector<LineSegment> LineSegmentArray;

class Plane{
public:
    Plane(const hrp::Vector3& p1, const hrp::Vector3& p2, double th);
    Eigen::Vector3f normal, point;
};

hrp::ColdetModelPtr convertToConvexHull(
    const std::vector<hrp::Link *>& i_links,
    const String& i_string);
bool checkAnchors(hrp::ColdetModelPtr coldetModel, 
                  const std::vector<Anchor>& i_anchors);

LineSegmentArray split(hrp::ColdetModelPtr coldetModel,
                       const Plane& i_plane);



