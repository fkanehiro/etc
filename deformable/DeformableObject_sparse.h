// ---------------------------------------------------------------- 
// Project:			afstate
// Copyright:		CNRS-AIST, 2010
//
// File:			DeformableObject.h
// Author:			Abderrahmane Kheddar
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Declaration of class afstate::DeformableObject.
//
// ----------------------------------------------------------------

#ifndef _AF_DEF_BODY_H_
#define _AF_DEF_BODY_H_

//#include "afstate/Body.h"
#include "FEM_sparse.h"
#include <unsupported/Eigen/SuperLUSupport>

//#include "afbase/channels.h"
//#include "afbase/c_vector_array.h"
#include <tvmet/Vector.h>
#include "CollisionData.h"
#include <hrpUtil/uBlasCommonTypes.h>
#define AFSTATE_API

namespace afstate
{	
	class Universe;

	class DeformableObject 
	{
	public:

		enum DisplayMode
		{
			REGULAR_RENDERING,			// normal color
#if 1
                        WIRE_FRAME,
#endif
			VON_MISES_STRESS			// Von Misses stress representation stress coloration 
		};

		enum NormalsMode
		{
			NORMAL_PER_TRIANGLE,
			NORMAL_PER_VERTEX,
		};

	public:

		AFSTATE_API DeformableObject(const FEM &);
		AFSTATE_API virtual ~DeformableObject();

		AFSTATE_API void setDisplacement(const VectorXd&);
		AFSTATE_API void setForce(const VectorXd&);
		AFSTATE_API void solveU();

		AFSTATE_API void updateNodes();

		AFSTATE_API int getNodeIndex(int i) const;

		AFSTATE_API void setDisplayMode(const DisplayMode &);
		AFSTATE_API void setNormalsMode(const NormalsMode &);

		AFSTATE_API DisplayMode getDisplayMode() const;
		AFSTATE_API NormalsMode getNormalsMode() const;
		AFSTATE_API int getSizeF() const;
		AFSTATE_API int getSizeD() const;

	public: // channels (e.g. for afscene rendering)
#if 1
                typedef tvmet::Vector<float, 3> v3f; 
                typedef tvmet::Vector<float, 4> v4f; 

                std::vector<v3f> vertices; // updated in updateNodes
                std::vector<v3f> normals;  // updated in updateNodes
                std::vector<v4f> colors;    // updated in setDisplayMode/updateNodes, depending on display mode
                void draw();
#else

		afbase::channel_mem<afbase::c_vector_array<3, double> > vertices; // updated in updateNodes
		afbase::channel_mem<afbase::c_vector_array<3, double> > normals;  // updated in updateNodes
		afbase::channel_mem<afbase::c_vector_array<4, float> > colors;    // updated in setDisplayMode/updateNodes, depending on display mode
#endif
                MatrixXd& getNodes() { return nodes_; }
                VectorXi& getIndexDefNode() { return indexDefNode_; }
		hrp::dmatrix& getH() { return H_; }
		hrp::dmatrix& getHt() { return Ht_; }
		hrp::dmatrix& getC() { return C_; }
		hrp::dvector& getDeltaFree() { return delta_free_; }
		hrp::dmatrix computeW(const std::vector<CollisionData>& i_cd,
				      const hrp::Matrix33 &R);
                const VectorXd& getU() const { return uL_; }
	protected:

		DisplayMode dM_;				// display mode
		NormalsMode nM_;				// normals type: per triangle or per vertex

		int nbrRigids_;					// to be prepared for later implementation of rigid parts
		MatrixXd        nodes_;			// Nodes coordinates
		VectorXi indexDefNode_;			// Those to be deformed or displaced

		MatrixXf nColors_;				// Colors per nodes when stress is used
		MatrixXf tColors_;				// Colors of triangles according to surface properties

		// Mapping vector between Indes of surface ID and colors
		VectorXi indexColors_;
		VectorXi countColors_;

                DynamicSparseMatrix<double> mK_;
                SparseLU<SparseMatrix<double>,SuperLU> LU_;
                int fNodeSize_;
                
		VectorXd uD_;							// given applied displacements
		VectorXd eF_;							// given applied forces
		MatrixXi triList_;				        // Triangles lists (3 nodes) + surface ID (LSet number)

		VectorXd uL_;							// deformations to be computed at each iteration

		hrp::dmatrix H_, C_, Ht_;
		hrp::dvector delta_free_;
	};


	class DeformableBody 
		: public DeformableObject
#if 0
		, public Body
#endif
	{
	public:

		AFSTATE_API DeformableBody(Universe& universe, const FEM&);

		AFSTATE_API virtual bool isDeformable() const;
		AFSTATE_API virtual DeformableObject* asDeformable();
		AFSTATE_API virtual const DeformableObject* asDeformable() const;
	};
}


// --- DOC --------------------------------------------------------

/*!
* \file DeformableObject.h
* \brief Declaration of classes afstate::DeformableObject, afstate::DeformableBody.
* \author 
* \version 0.0.0
* \date July 19, 2010
*
* Declaration of the afstate::Body class.
*
*/

/*!
* \class afstate::DeformableObject DeformableObject.h "Definition"
* \brief Abstract interface of a Deformable object.
*/

/*!



* \var DisplayMode afstate::DeformableObject::dM_
* \brief display mode
* \var NormalsMode afstate::DeformableObject::nM_
* \brief normals type: per triangle or per vertex

* \var int afstate::DeformableObject::nbrRigids_
* \brief to be prepared for later implementation of rigid parts
* \var MatrixXd afstate::DeformableObject::nodes_
* \brief Nodes coordinates
* \var VectorXi afstate::DeformableObject::indexDefNode_
* \brief Those to be deformed or displaced

* \var MatrixXf afstate::DeformableObject::nColors_
* \brief Colors per nodes when stress is used
* \var MatrixXf afstate::DeformableObject::tColors_
* \brief Colors of triangles according to surface properties
* \var VectorXi afstate::DeformableObject::indexColors_
* \var VectorXi afstate::DeformableObject::countColors_

* \var MatrixXd afstate::DeformableObject::fK_
* \brief Inverse of Stiffness matrix reduced
* \var MatrixXd afstate::DeformableObject::dK_
* \brief Stiffness matrix for imposed displacements if any

* \var VectorXd afstate::DeformableObject::uD_
* \brief given applied displacements
* \var VectorXd afstate::DeformableObject::eF_
* \brief given applied forces
* \var MatrixXi afstate::DeformableObject::triList_
* \brief Triangles lists (3 nodes) + surface ID (LSet number)

* \var VectorXd afstate::DeformableObject::uL_
* \brief deformations to be computed at each iteration
*/

/*!
* \fn AFSTATE_API int afstate::DeformableObject::getNodeIndex(int i)
* Returns the index of the node i in the rearranged matrix.
* The index i can be taken from the msh file for instance. Note that
* you will need to do i-1 since GMSH ids start with 1.
*/

// -------------------------------------------------------------------

/*!
* \fn AFSTATE_API afstate::DeformableBody::DeformableBody(Universe& universe, const FEM& fem);
* \brief constructor of DeformableBody.
* \param universe the Universe in which the body is.
* \param fem 
*/

/*!
* \fn AFSTATE_API bool afstate::DeformableBody::isDeformable() const
* \brief indicates if the body is deformable.
*/

/*!
* \fn AFSTATE_API DeformableObject*  afstate::DeformableBody::asDeformable()
* \brief returns the corresponding deformable body.
*/

/*!
* \fn AFSTATE_API const DeformableObject*  afstate::DeformableBody::asDeformable() const
* \brief returns the corresponding const deformable body.
*/

#endif
