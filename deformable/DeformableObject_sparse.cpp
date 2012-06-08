// ---------------------------------------------------------------- 
// Project:			afstate
// Copyright:		CNRS-AIST, 2010
//
// File:			DeformableObject.cpp
// Author:			Abderrahmane Kheddar
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Implementation of class DeformableObject.h
//
// ----------------------------------------------------------------

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "DeformableObject_sparse.h"
#include <Eigen/Geometry>

using namespace Eigen;

namespace afstate
{
	DeformableObject::DeformableObject(const FEM & fem)
#if 0
		: vertices(afbase::mutexed_mem_t())
		, normals(afbase::mutexed_mem_t())
		, colors(afbase::mutexed_mem_t())
#endif
		  //: LU_(MinimumDegree_AT_PLUS_A)
		  : LU_(MinimumDegree_ATA)
		  //: LU_(ColApproxMinimumDegree)
	{
		// nodes, triangles and mapping table
		nodes_        = fem.getNodes(); 
		triList_      = fem.getTriangles();
		indexDefNode_ = fem.getIndexNode();

		// Bahavior matrices
                mK_ = fem.getsK();
                fNodeSize_ = fem.fNodeSize();
                std::cout << "fNodeSize = " << fNodeSize_ << std::endl;
#if 1
                LU_.compute(mK_);
                if(!LU_.succeeded()) {
                    std::cout <<  "decomposiiton failed" << std::endl;
                    return;
                }
#endif
                const MatrixXd& KD = fem.getKD();

		// Coloring
		tColors_     = fem.getColors(); 
		indexColors_ = fem.getTagColor(); // useless since triangles are sorted TBR
		countColors_ = fem.getCountTagTriangle();

		// Sizings of imposed displacement uD_ and forces eF_
		uD_.resize(KD.cols());
		if(uD_.size() > 0) uD_.setZero();

		eF_.resize(mK_.rows());
		if(eF_.size() > 0) eF_.setZero();
		
		uL_.resize(mK_.rows());
		if(uL_.size() > 0) uL_.setZero();

		// resizings for afscene
		{
#if 0
			afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<3, double> > > lock(vertices);
			vertices.write().resize(triList_.rows()*3);
#else
                        vertices.resize(triList_.rows()*3);
#endif
		}

		nM_ = NORMAL_PER_VERTEX;
		{
#if 0
			afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<3, double> > > lock(normals);
			normals.write().resize(3*triList_.rows());
#else
                        normals.resize(3*triList_.rows());
#endif
		}

		// default: regular rendering with colors
		this->setDisplayMode(REGULAR_RENDERING);
	}

	DeformableObject::~DeformableObject(){
	}

	void DeformableObject::setDisplacement(VectorXd const& uD){
		if (uD_.size() == uD.size())
			uD_ = uD;
		else
			std::cerr << " DeformableObject::setDisplacement expects uD of size " << uD_.size() << std::endl;
	}

	void DeformableObject::setForce(VectorXd const& eF){
		if (eF_.size() == eF.size())
			eF_ = eF;
		else
			std::cerr << " DeformableObject::setForce expects eF of size " << eF_.size() << std::endl;
	}

	void DeformableObject::solveU(){
		// compute deformation knowing imposed forces and displacements
		// Note: uD_ will add as is to its nodes in simulation 
#if 1
                if(!LU_.solve(eF_,&uL_)) {
                    // solving failed
                    std::cout << "solveU() failed" << std::endl;
                    return;
                }
#endif
#if 0
                VectorXd err = mK_*uL_ - eF_;
                //std::cout << "err=" << err.norm() << std::endl;
#endif
	}

	void DeformableObject::setDisplayMode(const DisplayMode & dM)
	{
		dM_ = dM;

#if 0		
		afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<4, float> > > lock(colors);
		afbase::c_vector_array<4, float>& c_ = colors.write();
#else
                std::vector<v4f>& c_ = colors;
#endif

		c_.resize(3*triList_.rows());

		int k = 0;
		int c = 0;
		switch (dM)
		{
		case REGULAR_RENDERING:
			for (int i = 0; i < 3*triList_.rows(); ++i)
			{
				// put a color per vertex
				if (k < 3*countColors_(c)){
					for(int h = 0; h < 4; ++h){
						c_[i][h] = tColors_(c,h);
					}
					++k;
				}
				else
				{
					k = 0;
					++c;
					--i;
				}
			}
			break;
		case VON_MISES_STRESS:
			// nothing to do for now // will do it later
			break;
#if 1
                case WIRE_FRAME:
                        break;
#endif
		default:
			std::cerr << "DeformableObject::setDisplayMode-> not recognized Display Mode" << std::endl;
		}
	}

	void DeformableObject::setNormalsMode(const NormalsMode & nM)
	{
#if 0
		afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<3, double> > > lock(normals);
		afbase::c_vector_array<3, double>& n_ = normals.write();
#else
            std::vector<v3f>& n_ = normals;
#endif

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
			std::cerr << "DeformableObject::setNormalsMode-> not recognized NormalsMode" << std::endl;
		}
	}

	int DeformableObject::getNodeIndex(int i) const{
		return indexDefNode_[i];
	}

	void DeformableObject::updateNodes()
	{
#if 0
		afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<3, double> > > lock_vertices(vertices);
		afbase::c_vector_array<3, double>& v_ = vertices.write();

		afbase::scoped_lock_write<afbase::channel_mem<afbase::c_vector_array<3, double> > > lock_normals(normals);
		afbase::c_vector_array<3, double>& n_ = normals.write();
#else
            std::vector<v3f>& v_ = vertices;

            std::vector<v3f>& n_ = normals;
#endif

		int sizeTri = triList_.rows();
		
		// we need this only for NORMAL_PER_VERTEX
		MatrixXd nls(nodes_.rows(),3);
		nls.setZero();
        // counts the number of computed NORMAL_PER_VERTEX, we devide by 
		// this number (counts) to have the mean normal per vertex.
		VectorXi counts(nodes_.rows());
		counts.setZero();

		for (int i = 0; i < sizeTri; ++i)
		{
			// compute vertices new positions
			for(int j = 0; j < 3; ++j)
			{
				// which node?
				int node = triList_(i,j+1)-1;
				// node stands in uL_?
				int index = indexDefNode_(node);

				if (index == -1) // dirichlet with zero displacement fixed.
				{
					v_[3*i+j][0] = nodes_(node, 0); 
					v_[3*i+j][1] = nodes_(node, 1); 
					v_[3*i+j][2] = nodes_(node, 2);				
				}
				else
				{
					if (index < fNodeSize_) // free nodes
					{
						v_[3*i+j][0] = nodes_(node, 0) + uL_(3*index); 
						v_[3*i+j][1] = nodes_(node, 1) + uL_(3*index + 1); 
						v_[3*i+j][2] = nodes_(node, 2) + uL_(3*index + 2);
					}
					else if (uD_.size() > 0)
					{
						index -= fNodeSize_; // nodes under displacement constraints
						v_[3*i+j][0] = nodes_(node, 0) - uD_(3*index); 
						v_[3*i+j][1] = nodes_(node, 1) - uD_(3*index + 1); 
						v_[3*i+j][2] = nodes_(node, 2) - uD_(3*index + 2);
					}
				}
			}

			// Normals computation for this triangle
			Vector3d n;
			n(0) = v_[3*i+1][0] - v_[3*i][0]; 
			n(1) = v_[3*i+1][1] - v_[3*i][1]; 
			n(2) = v_[3*i+1][2] - v_[3*i][2];

			Vector3d rhs(v_[3*i+2][0] - v_[3*i+1][0], v_[3*i+2][1] - v_[3*i+1][1], v_[3*i+2][2] - v_[3*i+1][2]);
			n = n.cross(rhs);
			n.normalize();

			switch ( nM_ )
			{
			case NORMAL_PER_TRIANGLE: // simple copy to normal vector				
				n_[i][0] = n(0);
				n_[i][1] = n(1);
				n_[i][2] = n(2);
				break;
			case NORMAL_PER_VERTEX:
				// here we simply add the normal (n) to each node's intermediary (normals) 
				// associated vector, we need to know all neighbouring triangle's normals 
				// sum up their normals: only after culing all triangles we can compute 
				// the normals per vertex
				for(int j = 0; j < 3; ++j)
				{
					// which node?
					int node = triList_(i,j+1)-1;

					nls(node,0) += n(0);
					nls(node,1) += n(1);
					nls(node,2) += n(2);
					counts(node) += 1;
				}
				break;
			default:
				std::cerr << "DeformableObject::updateNormals()-> not recognized NormalsMode" << std::endl;
				return;
			}
		}
		switch ( nM_ )
		{
		case NORMAL_PER_TRIANGLE: // nothing to do
			break;
		case NORMAL_PER_VERTEX:
			// here we compute the mean of the normals
			for(int i = 0; i < nodes_.rows(); ++i)
			{
				if (counts(i) > 0)
				{
					nls(i,0) /= (double) counts(i);
					nls(i,1) /= (double) counts(i);
					nls(i,2) /= (double) counts(i);
					double n = sqrt(nls(i,0)*nls(i,0) + nls(i,1)*nls(i,1) + nls(i,2)*nls(i,2));
					nls(i,0) /= n;
					nls(i,1) /= n;
					nls(i,2) /= n;
				}
			}
			// attribute normals per vertex 
			for (int i = 0; i < sizeTri; ++i)
			{
				for(int j = 0; j < 3; ++j)
				{
					// which node?
					int node = triList_(i,j+1)-1;
					n_[3*i+j][0] = nls(node, 0); 
					n_[3*i+j][1] = nls(node, 1); 
					n_[3*i+j][2] = nls(node, 2); 
				}
			}
			break;
		default:
			std::cerr << "DeformableObject::updateNormals()-> should not reach here!" << std::endl;
		}
	}

	DeformableObject::DisplayMode DeformableObject::getDisplayMode() const
	{
		return dM_;
	}

	DeformableObject::NormalsMode DeformableObject::getNormalsMode() const
	{
		return nM_;
	}

	int DeformableObject::getSizeF() const{
		return eF_.size();
	}

	int DeformableObject::getSizeD() const{
		return uD_.size();
	}

#if 1
    void DeformableObject::draw(){
        if ((nM_ == NORMAL_PER_VERTEX && (vertices.size() != normals.size()))
            || (nM_ == NORMAL_PER_TRIANGLE && (vertices.size() != normals.size()*3))
            || (vertices.size() != colors.size())){
            std::cerr << "length mismatch:vertices = " << vertices.size()
                      << ", normals = " << normals.size() << ", colors = "
                      << colors.size() << std::endl;
            return;
        }
#if 0
        std::cout << "number of vertices = " << vertices.size()/3 << std::endl;
#endif

        if (dM_ == REGULAR_RENDERING) glBegin(GL_TRIANGLES);
        unsigned int rank;
        for (unsigned int i=0; i<vertices.size()/3; i++){
            if (dM_ == WIRE_FRAME) glBegin(GL_LINE_LOOP);
            if (nM_ == NORMAL_PER_TRIANGLE){
                glNormal3fv(normals[i].data()); 
            }
            for (unsigned int j=0; j<3; j++){
                rank = i*3+j;
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                             colors[rank].data());
                if (nM_ == NORMAL_PER_VERTEX){
                    glNormal3fv(normals[rank].data()); 
                }
                glVertex3fv(vertices[rank].data()); 
            }
            if (dM_ == WIRE_FRAME) glEnd();
        }
        if (dM_ == REGULAR_RENDERING) glEnd();
    }
#endif
	//  --- DeformableBody --------------------------------------

	DeformableBody::DeformableBody(Universe& universe, const FEM& fem)
		: DeformableObject(fem)
#if 0
		, Body(universe)
#endif
        {
	}

	bool DeformableBody::isDeformable() const{
		return true;
	}

	DeformableObject* DeformableBody::asDeformable(){
		return this;
	}

	const DeformableObject* DeformableBody::asDeformable() const{
		return this;
	}
}
