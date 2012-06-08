// ---------------------------------------------------------------- 
// Project:			afstate
// Copyright:		CNRS-AIST, 2010
//
// File:			FEM.h
// Author:			Abderrahmane Kheddar and Stanislas Brossette
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Declaration of the afstate::FEM related classes
//
// ----------------------------------------------------------------

#ifndef _AF_FEM_H_
#define _AF_FEM_H_

// Using Eigen library http://eigen.tuxfamily.org
#define EIGEN2_SUPPORT
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Sparse>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// import most common Eigen types
using namespace Eigen;

namespace afstate // to be removed for a standalone usage, this is mandatory only for AMELIF
{
// This class define the very basic information that is the material of the object
// when the deformable object is heterogeneous, need to define as many material as
// those composing the object
class Material
{
public:
	 Material(double, double);
	 Material(double, double, double);
	 Material(const Material &); // copy constructor

	// set methods
	void setYoung(double); 
    void setPoisson(double);
	void setRho(double);
	void computeLame(void); // Lame's can not be set

	// get methods
	double getYoung(void) const;
	double getPoisson(void) const;
	double getRho(void) const;
	double getLameL(void) const;
	double getLameM(void) const;

private:
	double young_;			// Unit in Pascal
	double poisson_;		// Unitless
	double lameL_, lameM_;	// Unit in Pascal
	double rho_;			// Volumic mass kg/m^3
}; 

//-------------------------------------------------------------------------
// Set some constants to define the type of Elements
// More can be added, the naming obeis NAMEOFSHAPE where the number is that
// of the nodes chosen for the element -> in our simulation we are using
// mostly TETRAHEDRON4 any new element can be added here

enum ElementType
{
	TETRAHEDRON4,
	TETRAHEDRON10,
	CUBIC8,
	PRISMATIC6,
	SHELL
};

// Class FiniteElement is VIRTUAL
class FiniteElement 
{
public:
	 FiniteElement(const Material &);
	~FiniteElement();

	void setMaterial(const Material &); //set a given material
	
	const MatrixXd & getElasticityMatrix() const; // not used mostly only when constraints are computed
	const double getKe(int, int) const; // get value of the one cell from Ke 
	const int getNbrNodePerElt() const; // return number of nodes for this element, usesd in assembly

	// All following methods are virtual and need to be redefined for each type of element
	virtual void setNodesCoordinates(const MatrixXd &) = 0; // set the element's node coordinates
	virtual void setGauss() = 0; // set Gauss' points and weight
	virtual double computeJacobian() const = 0; 
	virtual void computeShapeFunctionGradient(MatrixXd &) const = 0;
	virtual void computeKe() = 0;
	virtual void buildElasticityMatrix() = 0;
	
protected:
	int nbrNodePerElt_ ;			// Number of nodes per element
	ElementType eltType_;			// Should be one of the enum ElementType
	Material mat_;					// Element's material
	VectorXd vGaussWeights_;		// Gauss weights (refer any book on FEM)
	MatrixXd mGaussPoints_;			// Gauss points (refer any book on FEM)
	MatrixXd mNodesCoordinates_;	// Nodes coordinates of an element
	MatrixXd mElasticity_;			// Elasticity matrix	
    MatrixXd mKe_;					// this finite element stiffness
};

// This defines the Tetrahedron Element i.e. TETRAHEDRON4 that we are using
// If other elements are used, their class definition can be inspired from 
// this one. Note that any element definition should derive from FiniteElement

class TetrahedronElement : public virtual FiniteElement
{
public:
	// Constructor
	TetrahedronElement(const Material &);

	// Definition of the virtual methods in FiniteElement
    void setNodesCoordinates(const MatrixXd &);
	void setGauss();
	double computeJacobian() const;
	void computeShapeFunctionGradient(MatrixXd &) const;
	void buildElasticityMatrix();
	void computeKe();
};


// This class builds the object from elementary elements
class FEM
{
public:
	 FEM();
	~FEM();  //AK> to be made virtual
	
	virtual void scanFile() = 0;  // to be defined for each type of MESH
	void assembleK();             // does the assembly of the global stiffness matrix
	void sortK();				  // ordering K into blocks of force nodes and internal nodes
	void sortT();				  // ordering triangles and colors and tags
	void computeInverseForceCondensedK();  // computes the inverse condensed K
	void computeDisplacementCondensedK();  // computes the stiffness for imposed displacements
	
	// only for debug purposes -> saves the stiffness matrix in a given file
	void saveWhatEver(const std::string &) const;

	// send back methods to AMELIF (or any other software) for deformable body
	
	const MatrixXd & getNodes() const;
	const MatrixXi & getTriangles() const;
	const VectorXi & getIndexNode() const;
	const MatrixXd & getcK() const;
        const DynamicSparseMatrix<double>& getsK() const;
        const MatrixXd & getKD() const;
#if 1
        void setcK(const MatrixXd&);
#endif
	const MatrixXd & getdK() const;
	const MatrixXf & getColors() const;
	const VectorXi & getTagColor() const;
	const VectorXi & getCountTagTriangle() const;
	double mass() const { return mass_; }
	const VectorXd& com() const { return com_; }
	const VectorXd& I() const { return I_; }
        int fNodeSize() const { return fNodeSize_; }

protected:

	int nbrDefMat_ ;      // Number of material composing the volume object (generally 1, in composite object many)
	std::vector<FiniteElement *> fe_ ;  // Creation of one FE per material

	int nbrNodeT_ ;       // Total Number of Nodes
	MatrixXd nodeCoord_ ; // Node list
	int nbrElts_ ;        // Number of elements

	VectorXi tagNodes_;   // Tags the nodes
	int fNodeSize_;		  // force imposed or free deforming nodes
	int iNodeSize_;       // internal nodes
	int dNodeSize_;       // displacement imposed nodes
	int rNodeSize_;       // rigid nodes // TO BE DONE LATER
	
	MatrixXi triangles_;  // Triangles list
	VectorXi indexNode_;  // Computed in sortK

	MatrixXi conTable_  ; // Elements list
	VectorXi conMat_ ;    // Material per element list

	MatrixXi dof_ ;       // Dof matrix
	
	int nbrNodeF_ ;       // Number of Free Nodes
	
	DynamicSparseMatrix<double> mK_;		  // THE stifness matrix
	MatrixXd mKD_;        // This matrix is computed to allow imposing displacements on Dirichlet faces
						  // If any, it is computed with the stiffness matrix -> AMELIF/software Def Object

	MatrixXd cK_;		  // Stores inverse of Stiffness matrix reduced -> AMELIF/software Def Object
	MatrixXd dK_;         // Stores Stiffness matrix for imposed displacements if any -> AMELIF/software Def Object

	MatrixXf colors_;
	VectorXi tagsColors_;
	VectorXi countTagTriangle_;

	double mass_;
	VectorXd com_, I_;
};

// Handling GSMH mesh files
// This class can inspire any other mesh format
// Should redefine the class for another format

class GMSH : public FEM
{
public:
	 GMSH(const std::string &);
	~GMSH();

	void scanFile();
	bool isDirichlet(int) const;
	bool isEffort(int) const;
	int whatMaterial(int) const;

private:
	std::ifstream fMsh_ ; // Id on .msh file
	int  phyVolId_ ;      // Id on physical volume
	MatrixXi geoVol_;     // Table of geometric volume and its material [1] = geoVolId [2] = material ID
	VectorXi dirichlet_ ;
	VectorXi effort_ ;
};
}
#endif //_AF_FEM_H_
