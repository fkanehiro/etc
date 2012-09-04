#include <iostream>
#include <hrpModel/Link.h>
#include "polysplit.h"

extern "C" {
#include <qhull/qhull_a.h>
}

using namespace hrp;

void convertToConvexHull(hrp::BodyPtr i_body)
{
    for (int i=0; i<i_body->numLinks(); i++){
        convertToConvexHull(i_body->link(i));
    }
}

void convertToConvexHull(hrp::Link *i_link)
{
    if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) return;

    int ptype = i_link->coldetModel->getPrimitiveType();
    if (ptype == ColdetModel::SP_PLANE || ptype == ColdetModel::SP_SPHERE){
        return;
    }

    ColdetModelPtr coldetModel(new ColdetModel());
    coldetModel->setName(i_link->name.c_str());
    coldetModel->setPrimitiveType(ColdetModel::SP_MESH);
    // qhull
    int numVertices = i_link->coldetModel->getNumVertices();
    double points[numVertices*3];
    float v[3];
    for (int i=0; i<numVertices; i++){
        i_link->coldetModel->getVertex(i, v[0],v[1],v[2]);
        points[i*3+0] = v[0];
        points[i*3+1] = v[1];
        points[i*3+2] = v[2];
    }
    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc");
    if (qh_new_qhull (3,numVertices,points,ismalloc,flags,NULL,stderr)) return;

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
    std::cout << i_link->name << ":" << i_link->coldetModel->getNumVertices()
              << " -> " << coldetModel->getNumVertices() << std::endl;
    i_link->coldetModel =  coldetModel;
    
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }
    //
    //std::cerr << i_link->name << " reduce triangles from " << numTriangles << " to " << num << std::endl;
}

ColdetModelPtr convertToConvexHull(const std::vector<hrp::Link *>& i_links,
                                   const std::vector<anchor>& i_anchors)
{
    int numVertices=i_anchors.size();
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

    for (size_t i=0; i<i_anchors.size(); i++){
        const anchor &anc = i_anchors[i];
        const Vector3 p = anc.position();
        points[vindex*3+0] = p[0];
        points[vindex*3+1] = p[1];
        points[vindex*3+2] = p[2];
        vindex++;
    }

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

std::vector<int> findAnchors(ColdetModelPtr coldetModel, 
                             const std::vector<anchor>& i_anchors)
{
    std::vector<int> ret;
    for (size_t i=0; i<i_anchors.size(); i++){
        const anchor& anc = i_anchors[i];
        Vector3 p = anc.position();
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
            std::cerr << "can't find " << i << "th anchor" << std::endl;
        }
    }
}

anchor::anchor(hrp::Link *i_link, const hrp::Vector3& i_point) :
    link(i_link),
    point(i_point)
{
}

Vector3 anchor::position() const
{
    return link->p + link->R*point;
}
