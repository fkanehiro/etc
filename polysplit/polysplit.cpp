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

    const Vector3 p1 = i_string.first->position();
    points[vindex*3+0] = p1[0];
    points[vindex*3+1] = p1[1];
    points[vindex*3+2] = p1[2];
    vindex++;
    const Vector3 p2 = i_string.second->position();
    points[vindex*3+0] = p2[0];
    points[vindex*3+1] = p2[1];
    points[vindex*3+2] = p2[2];
    vindex++;

    for (size_t i=0; i<i_links.size(); i++){
        Link *i_link = i_links[i];
        if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) continue;

        for (int i=0; i<i_link->coldetModel->getNumVertices(); i++){
            i_link->coldetModel->getVertex(i, v[0],v[1],v[2]);
            const Vector3 localP(v[0], v[1], v[2]);
            const Vector3 absP = i_link->p + i_link->R*localP;
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
    std::cout << "build qhull:" << numVertices
              << " -> " << coldetModel->getNumVertices() << std::endl;
    
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }

    return coldetModel;
}

bool checkAnchors(ColdetModelPtr coldetModel, 
                  const std::vector<Anchor>& i_anchors)
{
    std::vector<int> ret;
    for (size_t i=0; i<i_anchors.size(); i++){
        Vector3 p = i_anchors[i].position();
        float v[3];
        int j;
        for (j=0; j<coldetModel->getNumVertices(); j++){
            coldetModel->getVertex(j, v[0], v[1], v[2]);
            double len = (p - Vector3(v[0],v[1],v[2])).norm();
            if (len < 1e-4){
                ret.push_back(j); 
                break;
            }
        }
        if (j == coldetModel->getNumVertices()){
            return false;
        }
    }
    return true;
}

Anchor::Anchor(hrp::Link *i_link, const hrp::Vector3& i_point) :
    link(i_link),
    point(i_point)
{
}

Vector3 Anchor::position() const
{
    return link->p + link->R*point;
}

LineSegmentArray split(hrp::ColdetModelPtr coldetModel,
                       const Plane& i_plane)
{
    LineSegmentArray ret;
    for (int i=0; i<coldetModel->getNumTriangles(); i++){
        int vindex0,vindex1,vindex2;
        coldetModel->getTriangle(i, vindex0, vindex1, vindex2);
        Eigen::Vector3f v0, v1, v2;
        coldetModel->getVertex(vindex0, v0[0], v0[1], v0[2]);
        coldetModel->getVertex(vindex0, v1[0], v1[1], v1[2]);
        coldetModel->getVertex(vindex0, v2[0], v2[1], v2[2]);
        double d0,d1,d2;
        d0 = i_plane.normal.dot(v0 - i_plane.point);
        d1 = i_plane.normal.dot(v1 - i_plane.point);
        d2 = i_plane.normal.dot(v2 - i_plane.point);
        double d0d1 = d0*d1;
        double d1d2 = d1*d2;
        double d2d0 = d2*d0;
        double absd0 = fabs(d0);
        double absd1 = fabs(d1);
        double absd2 = fabs(d2);
        if (d0d1 > 0 && d1d2 < 0 && d2d0 < 0){
            // v1v2 and v2v0 are intersecting
            ret.push_back(LineSegment((absd2*v1+absd1*v2)/(absd1+absd2),
                                      (absd0*v2+absd2*v0)/(absd2+absd0)));
        }else if (d0d1 < 0 && d1d2 > 0 && d2d0 < 0){
            // v0v1 and v2v0 are intersecting
            ret.push_back(LineSegment((absd1*v0+absd0*v1)/(absd0+absd1),
                                      (absd0*v2+absd2*v0)/(absd2+absd0)));
        }else if (d0d1 < 0 && d1d2 < 0 && d2d0 > 0){
            // v0v1 and v1v2 are intersecting
            ret.push_back(LineSegment((absd1*v0+absd0*v1)/(absd0+absd1),
                                      (absd2*v1+absd1*v2)/(absd1+absd2)));
        }else if ((d0d1 > 0 && d1d2 > 0 && d2d0 > 0)
                  || (d0d1 < 0 && d1d2 < 0 && d2d0 < 0)){
            // no intersection
        }else{
            std::cerr << "unexpected case:" << d0 << "," << d1 << "," << d2
                      << std::endl;
        }
    }
    return ret;
}

Plane::Plane(const hrp::Vector3& p1, const hrp::Vector3& p2, double th)
{
    Vector3 v1=p2-p1;
    Vector3 v2(cos(th), sin(th), 0);
    Vector3 n = v1.cross(v2).normalized();
    normal << n[0], n[1], n[2];
    point << p1[0], p1[1], p2[1];
}
