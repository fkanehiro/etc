#include <iostream>
#include <hrpModel/Link.h>
#include "polysplit.h"

extern "C" {
#include <qhull/qhull_a.h>
}

using namespace hrp;

ColdetModelPtr convertToConvexHull(const std::vector<hrp::Link *>& i_links,
                                   const String& i_string)
{
    int numVertices=2; // the number of anchors
    for (size_t i=0; i<i_links.size(); i++){
        Link *i_link = i_links[i];
        if (i_link->coldetModel){
            numVertices += i_link->coldetModel->getNumVertices();
        }
    }

    ColdetModelPtr coldetModel(new ColdetModel());
    coldetModel->setName("chull");
    coldetModel->setPrimitiveType(ColdetModel::SP_MESH);
    // qhull
    double points[numVertices*3];
    float v[3];
    int vindex=0;

    const Vector3 p1 = i_string.start->position();
    points[vindex*3+0] = p1[0];
    points[vindex*3+1] = p1[1];
    points[vindex*3+2] = p1[2];
    vindex++;
    const Vector3 p2 = i_string.end->position();
    points[vindex*3+0] = p2[0];
    points[vindex*3+1] = p2[1];
    points[vindex*3+2] = p2[2];
    vindex++;

    for (size_t i=0; i<i_links.size(); i++){
        Link *i_link = i_links[i];
        Matrix33 att = i_link->attitude();
        if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) continue;

        for (int i=0; i<i_link->coldetModel->getNumVertices(); i++){
            i_link->coldetModel->getVertex(i, v[0],v[1],v[2]);
            const Vector3 localP(v[0], v[1], v[2]);
            const Vector3 absP = i_link->p + att*localP;
            points[vindex*3+0] = absP[0];
            points[vindex*3+1] = absP[1];
            points[vindex*3+2] = absP[2];
            vindex++;
        }
    }

    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc");
    if (qh_new_qhull (3,numVertices,points,ismalloc,flags,NULL,stderr)){
        return coldetModel;
    }

    qh_triangulate();
    qh_vertexneighbors();
    
    coldetModel->setNumVertices(qh num_vertices);
    coldetModel->setNumTriangles(qh num_facets);
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
        int p = qh_pointid(vertex->point);
        index[p] = vertexIndex;
        coldetModel->setVertex(vertexIndex++, points[p*3+0], points[p*3+1], points[p*3+2]);
    }
    facetT *facet;
    int num = qh num_facets;
    int triangleIndex = 0;
    FORALLfacets {
        int j = 0, p[3];
        setT *vertices = qh_facet3vertex (facet);
        vertexT **vertexp;
        FOREACHvertexreverse12_ (vertices) {
            if (j<3) {
                p[j] = index[qh_pointid(vertex->point)];
            } else {
                fprintf(stderr, "extra vertex %d\n",j);
            }
            j++;
        }
        coldetModel->setTriangle(triangleIndex++, p[0], p[1], p[2]);
    }

    coldetModel->build();
#if 0
    std::cout << "build qhull:" << numVertices
              << " -> " << coldetModel->getNumVertices() << std::endl;
#endif
    
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }

    return coldetModel;
}

#define ANCHOR_THD 1e-3
bool checkAnchor(ColdetModelPtr coldetModel, Anchor *i_anchor)
{
    Vector3 p = i_anchor->position();
    float v[3];
    for (int j=0; j<coldetModel->getNumVertices(); j++){
        coldetModel->getVertex(j, v[0], v[1], v[2]);
        double len = (p - Vector3(v[0],v[1],v[2])).norm();
        if (len < ANCHOR_THD){
            return true;
        }
    }
    return false;
}

Anchor::Anchor(hrp::Link *i_link, const hrp::Vector3& i_point) :
    link(i_link),
    point(i_point)
{
}

Vector3 Anchor::position() const
{
    return link->p + link->attitude()*point;
}

PolyLine split(hrp::ColdetModelPtr coldetModel, 
               const Plane& i_plane1, const Plane& i_plane2)
{
    PolyLine ret;
    for (int i=0; i<coldetModel->getNumTriangles(); i++){
        int vindex0,vindex1,vindex2;
        coldetModel->getTriangle(i, vindex0, vindex1, vindex2);
        float fv0[3], fv1[3],fv2[3];
        coldetModel->getVertex(vindex0, fv0[0], fv0[1], fv0[2]);
        coldetModel->getVertex(vindex1, fv1[0], fv1[1], fv1[2]);
        coldetModel->getVertex(vindex2, fv2[0], fv2[1], fv2[2]);
        hrp::Vector3 v0(fv0[0],fv0[1],fv0[2]);
        hrp::Vector3 v1(fv1[0],fv1[1],fv1[2]);
        hrp::Vector3 v2(fv2[0],fv2[1],fv2[2]);
        double d0,d1,d2;
        d0 = i_plane1.distance(v0);
        d1 = i_plane1.distance(v1);
        d2 = i_plane1.distance(v2);
        double d0d1 = d0*d1;
        double d1d2 = d1*d2;
        double d2d0 = d2*d0;
        double absd0 = fabs(d0);
        double absd1 = fabs(d1);
        double absd2 = fabs(d2);
        if ((d0d1 > 0 && d1d2 < 0 && d2d0 < 0)
            || (d0d1 < 0 && d1d2 > 0 && d2d0 > 0)){
            // v1v2 and v2v0 are intersecting
            hrp::Vector3 p1 = (absd2*v1+absd1*v2)/(absd1+absd2);
            hrp::Vector3 p2 = (absd0*v2+absd2*v0)/(absd2+absd0);
            if (i_plane2.distance(p1) >= -ANCHOR_THD 
                && i_plane2.distance(p2) >= -ANCHOR_THD){
                ret.add(LineSegment(p1,p2));
            }
        }else if ((d0d1 < 0 && d1d2 > 0 && d2d0 < 0)
                  || (d0d1 > 0 && d1d2 < 0 && d2d0 > 0)){
            // v0v1 and v2v0 are intersecting
            hrp::Vector3 p1 = (absd1*v0+absd0*v1)/(absd0+absd1);
            hrp::Vector3 p2 = (absd0*v2+absd2*v0)/(absd2+absd0);
            if (i_plane2.distance(p1) >= -ANCHOR_THD 
                && i_plane2.distance(p2) >= -ANCHOR_THD){
                ret.add(LineSegment(p1, p2));
            }
        }else if ((d0d1 < 0 && d1d2 < 0 && d2d0 > 0)
                  || (d0d1 > 0 && d1d2 > 0 && d2d0 < 0)){
            // v0v1 and v1v2 are intersecting
            hrp::Vector3 p1 = (absd1*v0+absd0*v1)/(absd0+absd1);
            hrp::Vector3 p2 = (absd2*v1+absd1*v2)/(absd1+absd2);
            if (i_plane2.distance(p1) >= -ANCHOR_THD 
                && i_plane2.distance(p2) >= -ANCHOR_THD){
                ret.add(LineSegment(p1, p2));
            }
        }else if (absd0 < ANCHOR_THD*2 && d1d2 <= 0){
            // v0 and v1v2 are intersecting
            hrp::Vector3 p = (absd2*v1+absd1*v2)/(absd1+absd2);
            if (i_plane2.distance(p) > 0){
                ret.add(LineSegment(v0, p));
            }
        }else if (absd1 < ANCHOR_THD*2 && d2d0 <= 0){
            // v1 and v2v0 are intersecting
            hrp::Vector3 p = (absd0*v2+absd2*v0)/(absd2+absd0);
            if (i_plane2.distance(p) > 0){
                ret.add(LineSegment(v1, p));
            }
        }else if (absd2 < ANCHOR_THD*2 && d0d1 <= 0){
            // v2 and v0v1 are intersecting
            hrp::Vector3 p = (absd1*v0+absd0*v1)/(absd0+absd1);
            if (i_plane2.distance(p) > 0){
                ret.add(LineSegment(v2, p));
            }
        }else if ((d0d1 > 0 && d1d2 > 0 && d2d0 > 0)
                  || (d0d1 < 0 && d1d2 < 0 && d2d0 < 0)){
            // no intersection
        }else if ((d0 == 0 && d1d2 > 0)
                  ||(d1 == 0 && d2d0 > 0)
                  ||(d2 == 0 && d0d1 > 0)){
            // 
        }else{
            std::cerr << "unexpected case:" << d0 << "," << d1 << "," << d2
                      << std::endl;
            //std::cout << v0.transpose() << "," << v1.transpose() << "," << v2.transpose() << std::endl;
        }
    }
    return ret;
}

Plane::Plane(const hrp::Vector3& p1, const hrp::Vector3& p2, double th)
{
    Vector3 v1=p2-p1;
    Vector3 v2(cos(th), sin(th), 0);
    normal = v1.cross(v2).normalized();
    point = p1;
#if 0
    std::cout << "Plane: normal = " << normal.transpose() << std::endl;
    std::cout << "Plane: point = " << point.transpose() << std::endl;
#endif
}

double Plane::distance(const hrp::Vector3& v) const
{
    return normal.dot(v - point);
}

double PolyLine::length()
{
    double len=0;
    for (size_t i=0; i<lines.size(); i++){
        const LineSegment& ls = lines[i];
        len += (ls.first - ls.second).norm();
    }
    return len;
}

void PolyLine::add(const LineSegment& segment)
{
    lines.push_back(segment);
}

void PolyLine::clear()
{
    lines.clear();
}

String::String(Anchor *i_start, Anchor *i_end, 
               const std::vector<hrp::Link *>& i_links) :
    start(i_start),
    end(i_end),
    links(i_links)
{
}

bool String::update()
{
    polyLine.clear();
    ColdetModelPtr chull = convertToConvexHull(links, *this);
    if (!checkAnchor(chull, start) || !checkAnchor(chull, end)){
        std::cerr << "at least one of anchors is not on the surface of the convex hull" << std::endl;
        return false;
    }

    Plane plane1(start->position(), end->position(), 0);
    Plane plane2(start->position(), end->position(), -M_PI/2);
    polyLine = split(chull, plane1, plane2);
#if 0
    std::cout << "the number of lines:" << pl.lines.size() << std::endl;
    std::cout << "length of polyline:" <<  pl.length() << std::endl;
    for (size_t i=0; i<pl.lines.size(); i++){
        std::cout << pl.lines[i].first.transpose() << "<->" 
                  << pl.lines[i].second.transpose() << std::endl;
    }
#endif
    return true;
}

double String::length()
{
    return polyLine.length();
}
