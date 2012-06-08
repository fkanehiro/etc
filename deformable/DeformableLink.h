// ---------------------------------------------------------------- 
// Project:			ra-simulator
// Copyright:		CNRS-AIST, 2010
//
// File:			DeformableLink.h
// Author:			Abderrahmane Kheddar
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Declaration of class hrp::DeformableLink.
//
// ----------------------------------------------------------------

#ifndef _HRP_DEFORMABLE_LINK_H_
#define _HRP_DEFORMABLE_LINK_H_

#include "FEM.h"
#include "LinkExt.h"
#include "CollisionData.h"
#include <hrpUtil/uBlasCommonTypes.h>
#include <map>

namespace hrp
{	
    class DeformableLink : public LinkExt
    {
      public:
        enum enumExtendMode{
            NONE,               
            DRAW_DEFORM_FORCE   
        };

        enum NormalsMode
        {
            NORMAL_PER_TRIANGLE,
            NORMAL_PER_VERTEX,
        };

      public:
        DeformableLink(const afstate::FEM &);
        virtual ~DeformableLink();

        void setDisplacement(const VectorXd&);
        void setForce(const VectorXd&);
        const VectorXd& getDeformation() const;
        void setDeformation(const VectorXd& i_uL);
        void solveU();

        void updateNodes();

        void setNormalsMode(const NormalsMode &);

        NormalsMode getNormalsMode() const;
        int getSizeF() const;
        int getSizeD() const;
        const std::map<int, hrp::Vector3>& getMapIndex_eF() const;
        void setMapIndex_eF(const std::map<int, hrp::Vector3>&);

      public: // channels (e.g. for afscene rendering)
        typedef tvmet::Vector<float, 3> v3f; 
        std::vector<v3f> normals;  // updated in updateNodes
        void draw(GLenum mode);
        MatrixXd& getNodes() { return nodes_; }
        VectorXi& getIndexDefNode() { return indexDefNode_; }
        MatrixXd& getfK() { return fK_; }
        VectorXd& getuL() { return uL_; }
        hrp::dmatrix& getH() { return H_; }
        hrp::dmatrix& getHt() { return Ht_; }
        hrp::dmatrix& getC() { return C_; }
        hrp::dvector& getDeltaFree() { return delta_free_; }
        hrp::dmatrix computeW(const std::vector<CollisionData>& i_cd);
        
      protected:

        NormalsMode nM_;				// normals type: per triangle or per vertex

        MatrixXd        nodes_;			// Nodes coordinates
        VectorXi indexDefNode_;			// Those to be deformed or displaced

        MatrixXf tColors_;				// Colors of triangles according to surface properties

        // Mapping vector between Indes of surface ID and colors
        VectorXi indexColors_;
        VectorXi triColorIndex_;

        MatrixXd fK_;							// Inverse of Stiffness matrix reduced
        MatrixXd dK_;							// Stiffness matrix for imposed displacements if any

        VectorXd uD_;							// given applied displacements
        VectorXd eF_;							// given applied forces
        MatrixXi triList_;				        // Triangles lists surface ID (LSet number) + (3 nodes)

        VectorXd uL_;							// deformations to be computed at each iteration
        hrp::dmatrix H_, C_, Ht_;
        hrp::dvector delta_free_;
        std::map<int, hrp::Vector3> mapIndex_eF_; // Relate eF_ and coldetModel index container

      private:
        static int      EXTEND_MODE;
        static double   SCALE_LENGTH;
      public:
        static void setExtendMode(int nVal);
        static int getExtendMode();
        static void setScaleLength(double dVal);
        static double getScaleLength();
    };
}


// --- DOC --------------------------------------------------------

/*!
 * \file DeformableLink.h
 * \brief Declaration of classes hrp::DeformableLink, hrp::DeformableBody.
 * \author 
 * \version 0.0.0
 * \date July 19, 2010
 *
 * Declaration of the hrp::Body class.
 *
 */

/*!
 * \class hrp::DeformableLink DeformableLink.h "Definition"
 * \brief Abstract interface of a Deformable object.
 */

/*!



 * \var NormalsMode hrp::DeformableLink::nM_
 * \brief normals type: per triangle or per vertex

 * \var MatrixXd hrp::DeformableLink::nodes_
 * \brief Nodes coordinates
 * \var VectorXi hrp::DeformableLink::indexDefNode_
 * \brief Those to be deformed or displaced

 * \var MatrixXf hrp::DeformableLink::tColors_
 * \brief Colors of triangles according to surface properties
 * \var VectorXi hrp::DeformableLink::indexColors_

 * \var MatrixXd hrp::DeformableLink::fK_
 * \brief Inverse of Stiffness matrix reduced
 * \var MatrixXd hrp::DeformableLink::dK_
 * \brief Stiffness matrix for imposed displacements if any

 * \var VectorXd hrp::DeformableLink::uD_
 * \brief given applied displacements
 * \var VectorXd hrp::DeformableLink::eF_
 * \brief given applied forces
 * \var MatrixXi hrp::DeformableLink::triList_
 * \brief Triangles lists (3 nodes) + surface ID (LSet number)

 * \var VectorXd hrp::DeformableLink::uL_
 * \brief deformations to be computed at each iteration
 */


// -------------------------------------------------------------------
#endif
