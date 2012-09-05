#include <hrpModel/Body.h>
#include <hrpCollision/ColdetModel.h>

class Anchor{
public:
    Anchor(hrp::Link *i_link, const hrp::Vector3& i_point);
    hrp::Vector3 position() const;
    hrp::Link *link;
    hrp::Vector3 point;
};

typedef std::pair<hrp::Vector3, hrp::Vector3> LineSegment;

class PolyLine{
public:
    double length();
    void add(const LineSegment& segment);
    void clear();
    typedef std::vector<LineSegment> LineSegmentArray;
    LineSegmentArray lines;
};

class String{
public:
    String(Anchor *i_start, Anchor *i_end, 
           const std::vector<hrp::Link *>& i_links);
    bool update();
    double length();
    std::vector<hrp::Link *> links;
    Anchor *start, *end;
    PolyLine polyLine;
};

class Plane{
public:
    Plane(const hrp::Vector3& p1, const hrp::Vector3& p2, double th);
    double distance(const hrp::Vector3 &v) const;
    hrp::Vector3 normal, point;
};

hrp::ColdetModelPtr convertToConvexHull(
    const std::vector<hrp::Link *>& i_links,
    const String& i_string);

bool checkAnchor(hrp::ColdetModelPtr coldetModel, Anchor *i_anchor);
PolyLine split(hrp::ColdetModelPtr coldetModel, 
               const Plane& i_plane1, const Plane& i_plane2);



