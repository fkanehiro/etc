#include <iostream>
#include <fstream>
#include <hrpModel/Link.h>
#include <hrpModel/Body.h>
#include <hrpModel/ModelLoaderUtil.h>

void convertToBoundingBox(hrp::BodyPtr i_body);
void convertToBoundingBox(hrp::Link *i_link);

using namespace hrp;

void convertToBoundingBox(hrp::BodyPtr i_body)
{
    for (int i=0; i<i_body->numLinks(); i++){
        convertToBoundingBox(i_body->link(i));
    }
}

void convertToBoundingBox(hrp::Link *i_link)
{
    if (!i_link->coldetModel || !i_link->coldetModel->getNumVertices()) return;

    int ptype = i_link->coldetModel->getPrimitiveType();
    if (ptype == ColdetModel::SP_PLANE || ptype == ColdetModel::SP_SPHERE){
        return;
    }

    // bounding box
    int numVertices = i_link->coldetModel->getNumVertices();
    float maxv[3],minv[3];
    for (int i=0; i<numVertices; i++){
      if (i==0){
        i_link->coldetModel->getVertex(i, maxv[0],maxv[1],maxv[2]);
        i_link->coldetModel->getVertex(i, minv[0],minv[1],minv[2]);
      }else{
	float v[3];
        i_link->coldetModel->getVertex(i, v[0],v[1],v[2]);
	for (int j=0; j<3; j++){
	  if (v[j] > maxv[j]) maxv[j] = v[j]; 
	  if (v[j] < minv[j]) minv[j] = v[j]; 
	}
      }
    }

    std::string fname = i_link->name + ".wrl";
    std::ofstream ofs(fname.c_str());

    ofs << "#VRML V2.0 utf8" << std::endl;
    ofs << "Transform {" << std::endl;
    ofs << "  translation ";
    for (int i=0; i<3; i++){
      ofs << (maxv[i] + minv[i])/2 << " ";
    }
    ofs << std::endl;
    ofs << "  children Shape {" << std::endl;
    ofs << "    appearance Appearance {" << std::endl;
    ofs << "      material Material {" << std::endl;
    ofs << "        diffuseColor 0.8 0.8 0.8" << std::endl;
    ofs << "        transparency 0.5" << std::endl;
    ofs << "      }" << std::endl;
    ofs << "    }" << std::endl;
    ofs << "    geometry Box {" << std::endl;
    ofs << "      size ";
    for (int i=0; i<3; i++){
      ofs << (maxv[i] - minv[i]) << " ";
    }
    ofs << std::endl;
    ofs << "    } #Box" << std::endl;
    ofs << "  } #Shape" << std::endl;
    ofs << "} #Transform" << std::endl;
}


int main(int argc, char *argv[])
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << " [VRML model]" << std::endl;
    return 1;
  }

  hrp::BodyPtr body(new hrp::Body());
  loadBodyFromModelLoader(body, argv[1], argc, argv, true);
  convertToBoundingBox(body);

  return 0;
}
