#include <iostream>
#include <fstream>
#include <hrpModel/Link.h>
#include <hrpModel/Body.h>
#include <hrpModel/ModelLoaderUtil.h>

using namespace Eigen;

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

    MatrixXd mat(numVertices, 3);
    float v[3];
    for (int i=0; i<numVertices; i++){
      i_link->coldetModel->getVertex(i, v[0], v[1], v[2]);
      for (int j=0; j<3; j++) mat(i,j) = v[j];
    }

    MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    MatrixXd cov = centered.adjoint()*centered;

    SelfAdjointEigenSolver<MatrixXd> eig(cov);

    MatrixXd ev = eig.eigenvectors();

    hrp::Matrix33 R;
    hrp::Vector3 xaxis, yaxis, zaxis;
    xaxis[0] = ev(0,0); xaxis[1] = ev(0,1); xaxis[2] = ev(0,2);
    yaxis[0] = ev(1,0); yaxis[1] = ev(1,1); yaxis[2] = ev(1,2);
    zaxis = xaxis.cross(yaxis);
    //ev(2,0) = zaxis[0]; ev(2,1) = zaxis[1]; ev(2,2) = zaxis[2]; 
    for (int i=0; i<3; i++){
      R(i,0) = xaxis[i];
      R(i,1) = yaxis[i];
      R(i,2) = zaxis[i];
    }

    MatrixXd rotated = ev*mat.transpose();
    VectorXd maxCoeff = rotated.rowwise().maxCoeff();
    VectorXd minCoeff = rotated.rowwise().minCoeff();
    VectorXd center = ev.transpose()*(maxCoeff+minCoeff)/2;
    VectorXd extent = maxCoeff - minCoeff;

    hrp::Vector3 omega = hrp::omegaFromRot(R);
    double th = omega.norm();
    hrp::Vector3 axis = omega.normalized();
    //std::cout << i_link->name << std::endl << ev << std::endl << axis.transpose() << ", " << th << std::endl;

    std::string fname = i_link->name + ".wrl";
    std::ofstream ofs(fname.c_str());

    ofs << "#VRML V2.0 utf8" << std::endl;
    ofs << "Transform {" << std::endl;
    ofs << "  translation ";
    for (int i=0; i<3; i++){
      ofs << center[i] << " ";
    }
    ofs << std::endl;
    ofs << "  rotation ";
    for (int i=0; i<3; i++){
      ofs << axis[i] << " ";
    }
    ofs << th << std::endl;
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
      ofs << extent[i] << " ";
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
