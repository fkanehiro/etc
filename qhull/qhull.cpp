#include <iostream>
#include <fstream>
extern "C" {
#include <qhull/qhull_a.h>
}
#include <hrpModel/Link.h>
#include <hrpModel/Body.h>
#include <hrpModel/ModelLoaderUtil.h>

void convertToConvexHull(hrp::BodyPtr i_body);
void convertToConvexHull(hrp::Link *i_link);

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

    std::string fname = i_link->name + ".wrl";
    std::ofstream ofs(fname.c_str());

    ofs << "#VRML V2.0 utf8" << std::endl;
    ofs << "Shape {" << std::endl;
    ofs << "  appearance Appearance {" << std::endl;
    ofs << "    material Material {" << std::endl;
    ofs << "      diffuseColor 0.8 0.8 0.8" << std::endl;
    ofs << "      transparency 0.5" << std::endl;
    ofs << "    }" << std::endl;
    ofs << "  }" << std::endl;
    ofs << "  geometry IndexedFaceSet {" << std::endl;
    ofs << "    coord Coordinate {" << std::endl;
    ofs << "      point [" << std::endl;

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
    
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
        int p = qh_pointid(vertex->point);
        index[p] = vertexIndex;
	ofs << "        " << points[p*3+0] << " " <<  points[p*3+1] << " " <<  points[p*3+2] << "," << std::endl;
        vertexIndex++;
    }

    ofs << "      ]" << std::endl;
    ofs << "    }" << std::endl;
    ofs << "    coordIndex [" << std::endl;

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
	ofs << "      " << p[0] << ", " <<  p[1] << ", " <<  p[2] << ", -1," << std::endl;
        triangleIndex++;
    }

    std::cout << i_link->name << ":" << i_link->coldetModel->getNumTriangles()
	      << "->" << triangleIndex++ << std::endl;

    ofs << "    ]" << std::endl;
    ofs << "  }" << std::endl;
    ofs << "}" << std::endl;

    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }
    //
    //std::cerr << i_link->name << " reduce triangles from " << numTriangles << " to " << num << std::endl;
}

int main(int argc, char *argv[])
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << " [VRML model]" << std::endl;
    return 1;
  }

  hrp::BodyPtr body(new hrp::Body());
  loadBodyFromModelLoader(body, argv[1], argc, argv, true);
  convertToConvexHull(body);

  return 0;
}
