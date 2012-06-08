// ---------------------------------------------------------------- 
// Project:			ra-simulator
// Copyright:		CNRS-AIST, 2010
//
// File:			DeformableLink.cpp
// Author:			Abderrahmane Kheddar
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Implementation of class DeformableLink.h
//
// ----------------------------------------------------------------


#include "EigenCoreIncludeUtil.h"
#include <Eigen/Geometry>
using namespace Eigen;

#include <hrpCollision/ColdetModel.h>
#include <hrpCollision/ColdetModelSharedDataSet.h>
#include <hrpCollision/Opcode/Opcode.h>
#include "DeformableLink.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

using namespace hrp;

int DeformableLink::EXTEND_MODE = NONE;
double DeformableLink::SCALE_LENGTH = 0.001;



DeformableLink::DeformableLink(const afstate::FEM & fem)
{
    // nodes, triangles and mapping table
    nodes_        = fem.getNodes(); 
    triList_      = fem.getTriangles();
    indexDefNode_ = fem.getIndexNode();

    // Bahavior matrices
    fK_ = fem.getcK();  
    dK_ = fem.getdK(); 

    // Coloring
    tColors_     = fem.getColors(); 
    indexColors_ = fem.getTagColor(); // useless since triangles are sorted TBR

    // Sizings of imposed displacement uD_ and forces eF_
    uD_.resize(dK_.cols());
    if(uD_.size() > 0) uD_.setZero();
    
    eF_.resize(fK_.rows());
    if(eF_.size() > 0) eF_.setZero();
		
    uL_.resize(fK_.rows());
    if(uL_.size() > 0) uL_.setZero();

    triColorIndex_.resize(triList_.rows());
    for (int i=0; i<triList_.rows(); i++){
        int faceId = triList_(i,0);
        for (int j=0; j<indexColors_.size(); j++){
            if (indexColors_[j] == faceId){
                triColorIndex_[i] = j;
                break;
            }
        }
    }

    setNormalsMode(NORMAL_PER_TRIANGLE);
#if 0
    std::cout << "nodes_ : " << nodes_.rows() << ", " << nodes_.cols() << std::endl;
    std::cout << "triList_ : "  << triList_.rows() << ", " << triList_.cols() << std::endl;
    std::cout << "indexDefNode_ : " << indexDefNode_.size() << std::endl;
    std::cout << "tColors_ : " << tColors_.rows() << ", " << tColors_.cols() << std::endl;
    std::cout << "indexColors_ : " << indexColors_ << std::endl;
    std::cout << "countColors_ : " << countColors_ << std::endl;
#endif

    coldetModel = ColdetModelPtr(new ColdetModel());
    coldetModel->setNumVertices(indexDefNode_.rows());
    coldetModel->setNumTriangles(triList_.rows());
    for (int i=0; i<triList_.rows(); i++){
        coldetModel->setTriangle(i, 
                                 triList_(i,1)-1,
                                 triList_(i,2)-1, 
                                 triList_(i,3)-1);
    }
    updateNodes();
    coldetModel->build();

    b = 0;
    Rs = 1,0,0,0,1,0,0,0,1;
    jointType = Link::FREE_JOINT;
    m = fem.mass();
    for (int i=0; i<3; i++) c[i] = fem.com()[i];
    for (int i=0; i<3; i++){
	for (int j=0; j<3; j++){
	    I(i,j) = fem.I()(i*3+j);
	}
    }

}

DeformableLink::~DeformableLink(){
}

void DeformableLink::setDisplacement(VectorXd const& uD){
    if (uD_.size() == uD.size())
        uD_ = uD;
    else
        std::cerr << " DeformableLink::setDisplacement expects uD of size " << uD_.size() << std::endl;
}

void DeformableLink::setForce(VectorXd const& eF){
    if (eF_.size() == eF.size())
        eF_ = eF;
    else
        std::cerr << " DeformableLink::setForce expects eF of size " << eF_.size() << std::endl;
}

void DeformableLink::solveU(){
    // compute deformation knowing imposed forces and displacements
    // Note: uD_ will add as is to its nodes in simulation 
#if 0
    std::cout << "uL_:(" << uL_.rows()  << "," << uL_.cols() << ")" << std::endl;
    std::cout << "fK_:(" << fK_.rows()  << "," << fK_.cols() << ")" << std::endl;
    std::cout << "eF_:(" << eF_.rows()  << "," << eF_.cols() << ")" << std::endl;
    std::cout << "dK_:(" << dK_.rows()  << "," << dK_.cols() << ")" << std::endl;
    std::cout << "uD_:(" << uD_.rows()  << "," << uD_.cols() << ")" << std::endl;
#endif            
    // if there are applied forces and displacements
    if(eF_.size() > 0 && uD_.size() > 0) uL_ = fK_*(eF_ - dK_*uD_);
    // else if there are applied forces but no displacement
    else if(eF_.size() > 0 && uD_.size() == 0) uL_ = fK_*eF_;
    // else if there are no applied forces
    else uL_.setZero();
}

void DeformableLink::setNormalsMode(const NormalsMode & nM)
{
    std::vector<v3f>& n_ = normals;

    nM_ = nM;
    switch (nM)
        {
        case NORMAL_PER_TRIANGLE:
            n_.resize(triList_.rows());
            break;
        case NORMAL_PER_VERTEX:
            n_.resize(3*triList_.rows());
            break;
        default:
            std::cerr << "DeformableLink::setNormalsMode-> not recognized NormalsMode" << std::endl;
        }
}

void RefitAABB(Opcode::MeshInterface *mesh, Opcode::AABBCollisionNode *node)
{
    if (node->IsLeaf()){
        // compute AABB of a triangle
        IceMaths::Point Min(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
        IceMaths::Point Max(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);

	// Loop through triangles
        Opcode::VertexPointers VP;
        // Get current triangle-vertices
        mesh->GetTriangle(VP, node->GetPrimitive());
        // Update global box
        Min.Min(*VP.Vertex[0]).Min(*VP.Vertex[1]).Min(*VP.Vertex[2]);
        Max.Max(*VP.Vertex[0]).Max(*VP.Vertex[1]).Max(*VP.Vertex[2]);
	node->mAABB.SetMinMax(Min, Max);

    }else{
        RefitAABB(mesh, (Opcode::AABBCollisionNode *)node->GetPos());
        RefitAABB(mesh, (Opcode::AABBCollisionNode *)node->GetNeg());
        // compute AABB of two AABBs
        IceMaths::Point Min, Max;
        IceMaths::Point Min_, Max_;
        node->GetPos()->mAABB.GetMin(Min);
        node->GetPos()->mAABB.GetMax(Max);
        
        node->GetNeg()->mAABB.GetMin(Min_);
        node->GetNeg()->mAABB.GetMax(Max_);
        
        Min.Min(Min_);
        Max.Max(Max_);
        
        node->mAABB.SetMinMax(Min, Max);
    }
}

void DeformableLink::updateNodes()
{
    mapIndex_eF_.clear();
    // update nodes
    for (int i=0; i<nodes_.rows(); i++){
        int index = indexDefNode_(i);
        if (index == -1) {
            // dirichlet with zero displacement fixed.
            coldetModel->setVertex(i, nodes_(i,0), nodes_(i,1), nodes_(i,2));
        }else{
            if (index < fK_.rows()/3){ // free nodes
                coldetModel->setVertex(i, 
                                       nodes_(i,0) + uL_(3*index),
                                       nodes_(i,1) + uL_(3*index+1),
                                       nodes_(i,2) + uL_(3*index+2));
                if( eF_(3*index) != 0   ||
                    eF_(3*index+1) != 0 ||
                    eF_(3*index+2) != 0 )
                {
                    mapIndex_eF_[i] = hrp::Vector3(eF_(3*index), eF_(3*index+1), eF_(3*index+2));
                }
            }else if (uD_.size() > 0){
                index -= fK_.rows()/3; // nodes under displacement constraints
                coldetModel->setVertex(i, 
                                       nodes_(i,0) - uD_(3*index),
                                       nodes_(i,1) - uD_(3*index+1),
                                       nodes_(i,2) - uD_(3*index+2));
            }
        }
    }
    // compute normals
    switch(nM_){
    case NORMAL_PER_TRIANGLE:
        for (int i=0; i<triList_.rows(); i++){
            Vector3f v1,v2,v3;
            coldetModel->getVertex(triList_(i,1)-1, v1(0), v1(1), v1(2));
            coldetModel->getVertex(triList_(i,2)-1, v2(0), v2(1), v2(2));
            coldetModel->getVertex(triList_(i,3)-1, v3(0), v3(1), v3(2));
            Vector3f e1(v2 - v1), e2(v3 - v2), n;
            n = e1.cross(e2);
            n.normalize();
            normals[i](0) = n(0);
            normals[i](1) = n(1);
            normals[i](2) = n(2);
        }
        break;
    case NORMAL_PER_VERTEX:
        std::cerr << "NORMAL_PER_VERTEX mode is not implemented" << std::endl;
        break;
    }

    // update AABB
    if (coldetModel->isValid()){
        Opcode::AABBCollisionTree *tree 
            = (Opcode::AABBCollisionTree *)coldetModel->getDataSet()->model.GetTree();
        Opcode::MeshInterface *mesh = &coldetModel->getDataSet()->iMesh;
        Opcode::AABBCollisionNode *root = (Opcode::AABBCollisionNode *)tree->GetNodes();
        RefitAABB(mesh, root);
    }
}

DeformableLink::NormalsMode DeformableLink::getNormalsMode() const
{
    return nM_;
}

int DeformableLink::getSizeF() const{
    //return uL_.size();
    return eF_.size();
}

int DeformableLink::getSizeD() const{
    return uD_.size();
}

void DeformableLink::draw(GLenum mode){
    glPushMatrix();
    double T[16];
    T[ 0]=R(0,0);T[ 4]=R(0,1);T[ 8]=R(0,2);T[12]=p(0);
    T[ 1]=R(1,0);T[ 5]=R(1,1);T[ 9]=R(1,2);T[13]=p(1);
    T[ 2]=R(2,0);T[ 6]=R(2,1);T[10]=R(2,2);T[14]=p(2);
    T[ 3]=0;     T[ 7]=0;     T[11]=0;     T[15]=1;
    glMultMatrixd(T);

    if (mode == GL_TRIANGLES) glBegin(GL_TRIANGLES);
    unsigned int rank;
    for (int i=0; i<triList_.rows(); i++){
        if (mode == GL_LINE_LOOP) glBegin(GL_LINE_LOOP);
        if (nM_ == NORMAL_PER_TRIANGLE){
            glNormal3fv(normals[i].data()); 
        }
        float color[] = {tColors_(triColorIndex_[i],0),
                         tColors_(triColorIndex_[i],1),
                         tColors_(triColorIndex_[i],2),
                         tColors_(triColorIndex_[i],3)};
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
        for (unsigned int j=0; j<3; j++){
            rank = i*3+j;
            if (nM_ == NORMAL_PER_VERTEX){
                glNormal3fv(normals[rank].data()); 
            }
            int node = triList_(i,j+1)-1;
            float x,y,z;
            coldetModel->getVertex(node, x, y, z);
            glVertex3f(x,y,z); 
        }
        if (mode == GL_LINE_LOOP) glEnd();
    }
    if (mode == GL_TRIANGLES) glEnd();

    if(EXTEND_MODE & DRAW_DEFORM_FORCE){
        for( std::map<int, hrp::Vector3>::const_iterator ite =  mapIndex_eF_.begin();
             ite != mapIndex_eF_.end(); ++ite)                 
        {
            glDisable(GL_LIGHTING);
            glColor3d(1,1,1);
            glBegin(GL_LINES);
            float x,y,z;
            coldetModel->getVertex(ite->first, x, y, z);
            glVertex3f(x, y, z);
            hrp::Vector3 local3d = ite->second;
            glVertex3f( x + local3d[0] * SCALE_LENGTH,
                        y + local3d[1] * SCALE_LENGTH,
                        z + local3d[2] * SCALE_LENGTH ); 
            glEnd();
            glEnable(GL_LIGHTING);
        }
    }
    drawConstraintForce();

    glPopMatrix();
}

const VectorXd& DeformableLink::getDeformation() const
{
    return uL_;
}

void DeformableLink::setDeformation(const VectorXd& i_uL)
{
    uL_ = i_uL;
}

const std::map<int, hrp::Vector3>& DeformableLink::getMapIndex_eF() const
{
    return mapIndex_eF_;
}

void DeformableLink::setMapIndex_eF(const std::map<int, hrp::Vector3>& ref)
{
    mapIndex_eF_ = ref;
}

void DeformableLink::setExtendMode(int nVal)
{
    DeformableLink::EXTEND_MODE = nVal;
}

int DeformableLink::getExtendMode()
{
    return DeformableLink::EXTEND_MODE;
}

void DeformableLink::setScaleLength(double dVal)
{
    DeformableLink::SCALE_LENGTH = dVal;
}

double DeformableLink::getScaleLength()
{
    return DeformableLink::SCALE_LENGTH;
}

hrp::dmatrix DeformableLink::computeW(const std::vector<CollisionData>& i_cd){
    hrp::dmatrix HCHt;
    int nnode = i_cd.size();
    //std::cout << "nnode:" << nnode << std::endl;
    if (nnode > 0){
        H_ = hrp::dzeromatrix(nnode, nnode*3);
        for (int i=0; i<nnode; i++){
            const hrp::Vector3 &n = i_cd[i].normal;
            H_(i,i*3  ) = n[0]*R(0,0)+n[1]*R(1,0)+n[2]*R(2,0);
            H_(i,i*3+1) = n[0]*R(0,1)+n[1]*R(1,1)+n[2]*R(2,1);
            H_(i,i*3+2) = n[0]*R(0,2)+n[1]*R(1,2)+n[2]*R(2,2);
        }
        Ht_ = trans(H_);
	
        delta_free_.resize(nnode);
        // extract C from fK
        C_.resize(nnode*3, nnode*3);
        for (int i=0; i<nnode; i++){
            for (int j=0; j<3; j++){
                int dstrow = i*3+j;
                int srcrow = indexDefNode_(i_cd[i].id1)*3+j;
                for (int k=0; k<nnode; k++){
                    for (int l=0; l<3; l++){
                        int dstcol = k*3+l;
                        int srccol = indexDefNode_(i_cd[k].id1)*3+l;
                        //std::cout << "srccol:" << srccol << std::endl;
                        C_(dstrow, dstcol) = fK_(srcrow, srccol);
                    }
                }
            }
            delta_free_(i) = -i_cd[i].idepth;
        }
        hrp::dmatrix HC(prod(H_,C_));
        HCHt = prod(HC, Ht_);
    }
    return HCHt;
}
