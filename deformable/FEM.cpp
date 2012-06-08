// ---------------------------------------------------------------- 
// Project:			afstate
// Copyright:		CNRS-AIST, 2010
//
// File:			FEM.cpp
// Author:			Abderrahmane Kheddar and Stanislas Brossette
// Version:			$Id$
// License:			no official license yet. Please do not distribute!
//
// Description:
//
// Implementation of FEM.h classes
//
// ----------------------------------------------------------------

#include "FEM.h"

using namespace std;

namespace afstate
{
///////////////////////////////////////////////////////////////////////////
// Definition of the Material Class
// Begin

Material::Material(double young, double poisson): 
   young_ (young), 
   poisson_ (poisson)
{
	rho_ = 1000.; // Default if not provided 
	computeLame();
}

Material::Material(double young, double poisson, double rho): 
   young_ (young), 
   poisson_ (poisson),
   rho_ (rho)
{
	computeLame();
}
// copy constructor
Material::Material(const Material & defMat)
{
	young_   = defMat.young_;
	poisson_ = defMat.poisson_;
	lameL_   = defMat.lameL_;
	lameM_   = defMat.lameM_;
	rho_     = defMat.rho_;
}

// Some useful setting methods 
void Material::setYoung(double young){
	young_ = young;
}

void Material::setPoisson(double poisson){
	poisson_ = poisson;
}

void Material::setRho(double rho){
	rho_ = rho;
}
// Lame's can not be set, they are only computed
void Material::computeLame()
{
	lameL_ = (young_ * poisson_) / ((1 + poisson_) * (1 - 2 * poisson_));
	lameM_ = young_ / (2 * (1 + poisson_));
}

// some useful gets
double Material::getYoung() const{
	return young_;
}

double Material::getPoisson() const{
	return poisson_;
}

double Material::getRho() const{
	return rho_;
}

double Material::getLameL() const{
	return lameL_;
}

double Material::getLameM()const{
	return lameM_;
}


// Definition of the Material Class
// End
//////////////////////////////////////////////////////////////////////////

// Defining the operator >> read ElementType constants
// because we can not istream enums 
istream& operator >> (istream& in, ElementType & et )
{
	int val;
	if ( in >> val ) 
	{
		switch ( val ) 
		{
		case TETRAHEDRON4:
			et = ElementType(val); 
			break;
		case TETRAHEDRON10: 
			et = ElementType(val); 
			break;
		case CUBIC8:
			et = ElementType(val); 
			break;
		case PRISMATIC6:
			et = ElementType(val); 
			break;
		case SHELL:
			et = ElementType(val); 
			break;
		default:
			assert(!"Invalid value for Element Type");
		}
	}
	return in;
}

///////////////////////////////////////////////////////////////////////////

// Definition of the VIRTUAL FiniteElement Class
// Begin

FiniteElement::FiniteElement(const Material & mat):
   mat_(mat){
	// Nothing more to do at this point
}

FiniteElement::~FiniteElement(){
  // Nothing to do at this stage, we assume that Eigen does well 
  // to free the space of its matrices and vectors  
}

void FiniteElement::setMaterial(const Material & mat){
	mat_ = mat;
}

const MatrixXd & FiniteElement::getElasticityMatrix() const{
   return mElasticity_;
}

const double FiniteElement::getKe(int i, int j) const{
	return mKe_(i, j);
}

const int FiniteElement::getNbrNodePerElt() const{
	return nbrNodePerElt_;
}

// Definition of the VIRTUAL FiniteElement Class
// End


///////////////////////////////////////////////////////////////////////////
// Definition of the TetrahedronElement derived from VIRTUAL FiniteElement
// Begin

TetrahedronElement::TetrahedronElement(const Material & mat):FiniteElement(mat)
{
	// introducing
	nbrNodePerElt_ = 4;  // obviously 4 nodes!
	eltType_ = TETRAHEDRON4;
	// basic sizings
	mElasticity_.resize(6,6);	// fixed size by definition
	mKe_.resize(12,12);			// fixde size by definition
    mNodesCoordinates_.resize(4,3);
	// basic settings
	setGauss();	// set Gauss points (refer to a FEM book)
	buildElasticityMatrix(); // (refer to a FEM book)
}


void TetrahedronElement::setNodesCoordinates(const MatrixXd & mEltCoordinates)
{
	mNodesCoordinates_ = mEltCoordinates;
}

void TetrahedronElement::setGauss()
{
	vGaussWeights_.resize(4);
		vGaussWeights_ <<	1./24.,
							1./24.,
							1./24.,
							1./24.;

	mGaussPoints_.resize(4,3);
	double a = (5.0 -     sqrt((double) 5))/20.;
	double b = (5.0 + 3.0*sqrt((double) 5))/20.;
	mGaussPoints_ << a, a, a,
		             a, a, b,
					 a, b, a,
					 b, a, a;
}

double TetrahedronElement::computeJacobian() const
{
	Matrix3d mJ;

	mJ << mNodesCoordinates_(0,0)-mNodesCoordinates_(3,0), mNodesCoordinates_(1,0)-mNodesCoordinates_(3,0), mNodesCoordinates_(2,0)-mNodesCoordinates_(3,0),
		  mNodesCoordinates_(0,1)-mNodesCoordinates_(3,1), mNodesCoordinates_(1,1)-mNodesCoordinates_(3,1), mNodesCoordinates_(2,1)-mNodesCoordinates_(3,1),
		  mNodesCoordinates_(0,2)-mNodesCoordinates_(3,2), mNodesCoordinates_(1,2)-mNodesCoordinates_(3,2), mNodesCoordinates_(2,2)-mNodesCoordinates_(3,2); 

	return mJ.determinant();
}

void TetrahedronElement::computeShapeFunctionGradient(MatrixXd & mBe) const
{
	MatrixXd mGSF(4,3); // Gradient of Shape Functions

	// mGSF(i,j) = d(BasicShapeTetrahedronIsosceles_i)/d(mNodesCoordinates_j) derivative of the basic form's of index i relative to mNodesCoordinates_ j
	mGSF(0,0) = mNodesCoordinates_(1,1)*mNodesCoordinates_(2,2)-mNodesCoordinates_(1,1)*mNodesCoordinates_(3,2)-mNodesCoordinates_(3,1)*mNodesCoordinates_(2,2)-mNodesCoordinates_(2,1)*mNodesCoordinates_(1,2)+mNodesCoordinates_(2,1)*mNodesCoordinates_(3,2)+mNodesCoordinates_(3,1)*mNodesCoordinates_(1,2);
	mGSF(0,1) =-mNodesCoordinates_(1,0)*mNodesCoordinates_(2,2)+mNodesCoordinates_(3,2)*mNodesCoordinates_(1,0)+mNodesCoordinates_(3,0)*mNodesCoordinates_(2,2)+mNodesCoordinates_(2,0)*mNodesCoordinates_(1,2)-mNodesCoordinates_(3,2)*mNodesCoordinates_(2,0)-mNodesCoordinates_(3,0)*mNodesCoordinates_(1,2);
	mGSF(0,2) = mNodesCoordinates_(1,0)*mNodesCoordinates_(2,1)-mNodesCoordinates_(3,1)*mNodesCoordinates_(1,0)-mNodesCoordinates_(3,0)*mNodesCoordinates_(2,1)-mNodesCoordinates_(2,0)*mNodesCoordinates_(1,1)+mNodesCoordinates_(3,1)*mNodesCoordinates_(2,0)+mNodesCoordinates_(3,0)*mNodesCoordinates_(1,1);
	mGSF(1,0) = mNodesCoordinates_(0,2)*mNodesCoordinates_(2,1)-mNodesCoordinates_(0,2)*mNodesCoordinates_(3,1)-mNodesCoordinates_(2,1)*mNodesCoordinates_(3,2)-mNodesCoordinates_(0,1)*mNodesCoordinates_(2,2)+mNodesCoordinates_(0,1)*mNodesCoordinates_(3,2)+mNodesCoordinates_(3,1)*mNodesCoordinates_(2,2);
	mGSF(1,1) =-mNodesCoordinates_(0,2)*mNodesCoordinates_(2,0)+mNodesCoordinates_(0,2)*mNodesCoordinates_(3,0)+mNodesCoordinates_(3,2)*mNodesCoordinates_(2,0)+mNodesCoordinates_(0,0)*mNodesCoordinates_(2,2)-mNodesCoordinates_(0,0)*mNodesCoordinates_(3,2)-mNodesCoordinates_(3,0)*mNodesCoordinates_(2,2);
	mGSF(1,2) = mNodesCoordinates_(0,1)*mNodesCoordinates_(2,0)-mNodesCoordinates_(0,1)*mNodesCoordinates_(3,0)-mNodesCoordinates_(3,1)*mNodesCoordinates_(2,0)-mNodesCoordinates_(0,0)*mNodesCoordinates_(2,1)+mNodesCoordinates_(0,0)*mNodesCoordinates_(3,1)+mNodesCoordinates_(3,0)*mNodesCoordinates_(2,1);
	mGSF(2,0) =-mNodesCoordinates_(0,2)*mNodesCoordinates_(1,1)+mNodesCoordinates_(0,2)*mNodesCoordinates_(3,1)+mNodesCoordinates_(1,1)*mNodesCoordinates_(3,2)+mNodesCoordinates_(0,1)*mNodesCoordinates_(1,2)-mNodesCoordinates_(0,1)*mNodesCoordinates_(3,2)-mNodesCoordinates_(3,1)*mNodesCoordinates_(1,2);
	mGSF(2,1) =-mNodesCoordinates_(0,0)*mNodesCoordinates_(1,2)+mNodesCoordinates_(0,2)*mNodesCoordinates_(1,0)+mNodesCoordinates_(0,0)*mNodesCoordinates_(3,2)-mNodesCoordinates_(0,2)*mNodesCoordinates_(3,0)-mNodesCoordinates_(3,2)*mNodesCoordinates_(1,0)+mNodesCoordinates_(3,0)*mNodesCoordinates_(1,2);
	mGSF(2,2) =-mNodesCoordinates_(0,1)*mNodesCoordinates_(1,0)+mNodesCoordinates_(0,1)*mNodesCoordinates_(3,0)+mNodesCoordinates_(3,1)*mNodesCoordinates_(1,0)+mNodesCoordinates_(0,0)*mNodesCoordinates_(1,1)-mNodesCoordinates_(0,0)*mNodesCoordinates_(3,1)-mNodesCoordinates_(3,0)*mNodesCoordinates_(1,1);
	mGSF(3,0) =-mNodesCoordinates_(0,1)*mNodesCoordinates_(1,2)+mNodesCoordinates_(0,2)*mNodesCoordinates_(1,1)-mNodesCoordinates_(0,2)*mNodesCoordinates_(2,1)+mNodesCoordinates_(0,1)*mNodesCoordinates_(2,2)-mNodesCoordinates_(1,1)*mNodesCoordinates_(2,2)+mNodesCoordinates_(2,1)*mNodesCoordinates_(1,2);
	mGSF(3,1) = mNodesCoordinates_(0,0)*mNodesCoordinates_(1,2)-mNodesCoordinates_(0,2)*mNodesCoordinates_(1,0)-mNodesCoordinates_(0,0)*mNodesCoordinates_(2,2)+mNodesCoordinates_(0,2)*mNodesCoordinates_(2,0)+mNodesCoordinates_(1,0)*mNodesCoordinates_(2,2)-mNodesCoordinates_(2,0)*mNodesCoordinates_(1,2);
	mGSF(3,2) =-mNodesCoordinates_(0,0)*mNodesCoordinates_(1,1)+mNodesCoordinates_(0,1)*mNodesCoordinates_(1,0)-mNodesCoordinates_(0,1)*mNodesCoordinates_(2,0)+mNodesCoordinates_(0,0)*mNodesCoordinates_(2,1)-mNodesCoordinates_(1,0)*mNodesCoordinates_(2,1)+mNodesCoordinates_(2,0)*mNodesCoordinates_(1,1);

	// epsilon = 1/2[(grad u)+(grad u)^t]
	mBe << mGSF(0,0), 0., 0., mGSF(1,0), 0., 0., mGSF(2,0), 0., 0., mGSF(3,0), 0., 0.,
		0., mGSF(0,1), 0., 0., mGSF(1,1), 0., 0., mGSF(2,1), 0., 0., mGSF(3,1), 0.,
		0., 0., mGSF(0,2), 0., 0., mGSF(1,2), 0., 0., mGSF(2,2), 0., 0., mGSF(3,2),
		mGSF(0,1), mGSF(0,0), 0., mGSF(1,1), mGSF(1,0), 0., mGSF(2,1), mGSF(2,0), 0., mGSF(3,1), mGSF(3,0), 0.,
		mGSF(0,2), 0., mGSF(0,0), mGSF(1,2), 0., mGSF(1,0), mGSF(2,2), 0., mGSF(2,0), mGSF(3,2), 0., mGSF(3,0),
		0., mGSF(0,2), mGSF(0,1), 0., mGSF(1,2), mGSF(1,1), 0., mGSF(2,2), mGSF(2,1), 0., mGSF(3,2), mGSF(3,1);
}

void TetrahedronElement::buildElasticityMatrix()
{
	double lameL = mat_.getLameL();
	double lameM = mat_.getLameM();

	mElasticity_ << lameL + 2*lameM , lameL, lameL, 0, 0, 0,
		            lameL , lameL + 2*lameM, lameL, 0, 0, 0,
					lameL , lameL , lameL+ 2*lameM, 0, 0, 0,
					0, 0, 0, lameM, 0, 0,
					0, 0, 0, 0, lameM, 0,
					0, 0, 0, 0, 0, lameM; 
}

void TetrahedronElement::computeKe()
{
	MatrixXd mBe(6,12);

	double detJ = computeJacobian();

	computeShapeFunctionGradient(mBe);

	double sumVGaussWeights = vGaussWeights_.sum();
	sumVGaussWeights /= detJ;
	mKe_ = mBe.transpose() * (mElasticity_ * sumVGaussWeights) * mBe;
}

// Definition of TetrahedronElement derived from VIRTUAL FiniteElement
// End


///////////////////////////////////////////////////////////////////////////
// Definition of the VIRTUAL FEM class
// Begin

FEM::FEM(){
	nbrDefMat_ = 1;
	mass_ = 0;
	com_.resize(3); com_.setZero();
	I_.resize(9); I_.setZero();
}

FEM::~FEM(){
}

// Methode d'assemblage
void FEM::assembleK()
{
	int nbrNodePerElt = fe_[0]->getNbrNodePerElt();
#if 1
	std::cout << "nbrNodePerElt:" << nbrNodePerElt << std::endl;
	std::cout << "nbrNodeT_:" << nbrNodeT_ << std::endl;
	//std::cout << "dof_:" << dof_ << std::endl;
#endif

	MatrixXd coords(nbrNodePerElt,3);
	VectorXi dofe(3*nbrNodePerElt);

	nbrNodeF_ = 0;
	// Determine the size of mK by counting non-negative terms in dof_
	// Should be reconsidered to a better way/implementation 
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		if (dof_(i,1) > 0){
			++nbrNodeF_;
		}
	}
    
	// Initialize mK to appropriate size + zeros;
	if(nbrNodeF_ > 0)
	{
		// Size of the full Stiffness matrix
		mK_.resize(nbrNodeF_*3, nbrNodeF_*3);
		mK_.setZero();
		// This matrix is not necessary when all free nodes are under force entry
		// it is necessary only when we decide that some Nodes can be controlled in displacement
		mKD_.resize(nbrNodeF_*3,nbrNodeT_*3-nbrNodeF_*3);
		if(mKD_.size() > 0)	mKD_.setZero();

	}
	else
	{
		assert(!"Should never get here FEM::assembleK() with zero free nodes!");
		return;
	}

	// Start building Stiffness matrix from each FE Ke's
	for(int e = 0; e < nbrElts_ ; ++e)
	{		
		// get coordinates of each FE's node
		for(int i = 0; i < nbrNodePerElt ; ++i)
		{
			for (int j = 0 ; j < 3; j++){
				coords(i,j) = nodeCoord_(conTable_(e,i)-1,j);				
			}
		}
		// ccompute this FE's Ke
		fe_[conMat_[e]]->setNodesCoordinates(coords);
		fe_[conMat_[e]]->computeKe();
		
		// Build the connectivity elements
		for(int i = 0; i < nbrNodePerElt ; ++i)
		{
			for (int j = 0 ; j < 3; j++){
				dofe(3*i+j) = dof_(conTable_(e,i)-1,j);
			}
		}

		// Assembly process per se
		for (int i = 0 ; i < dofe.size(); ++i)
		{
			if(dofe(i) > 0){
				for(int j = 0; j < dofe.size(); ++j){
					if(dofe(j) > 0){
					  double v = fe_[conMat_[e]]->getKe(i,j);
					  if (v) mK_.coeffRef(dofe(i)-1,dofe(j)-1) += v; 
					}
					else if(mKD_.size() > 0){
						mKD_(dofe(i)-1,-dofe(j)-1) += fe_[conMat_[e]]->getKe(i,j);
					}
				}
			}else{
				// Nothing to do apriori, need to check this
			}
		}
	}
}

void FEM::sortK()
{
#if 1
  std::cout << "fNodeSize:" << fNodeSize_ << std::endl;
  std::cout << "iNodeSize:" << iNodeSize_ << std::endl;
  std::cout << "dNodeSize:" << dNodeSize_ << std::endl;
#endif
	VectorXi fDofs(3*fNodeSize_);
	if(fDofs.size() > 0) fDofs.setZero();

	VectorXi iDofs;
        if (iNodeSize_){
            iDofs.resize(3*iNodeSize_);
            iDofs.setZero();
        }

	VectorXi fNodes;
        if (fNodeSize_) fNodes.resize(fNodeSize_);

	VectorXi iNodes;
        if (iNodeSize_) iNodes.resize(iNodeSize_);

	VectorXi dNodes;
        if (dNodeSize_) dNodes.resize(dNodeSize_);

	indexNode_.resize(nbrNodeT_);
	indexNode_.setZero();

	int fN = 0;
	int iN = 0;
	int dN = 0;
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		if (tagNodes_(i) == 0)
		{
			iNodes(iN) = i + 1;
			++iN;
		}
		if (tagNodes_(i) == 2)
		{
			fNodes(fN) = i + 1;
			++fN;
		}
		if (tagNodes_(i) == 1)
		{
			dNodes(dN) = i + 1;
			++dN;
		}
	}

	// This loop prepares for the correspondance between Nodes (force and displacement) and the Node
	// indexNode_ containts index the Nodes on which force is applied and those index + fNodeSize_ of
	// nodes on which displacement is applied
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		int j;
		for(j = 0; j < fNodeSize_; ++j)
		{
			if (fNodes(j) == i+1)
			{
				indexNode_(i) = j;
				break;
			}
		}
		if (j == fNodeSize_)
		{
			for(j = 0; j < dNodeSize_; ++j)
			{
				if (dNodes(j) == i+1)
				{
					indexNode_(i) = j + fNodeSize_;
					break;
				}
			}
			if (j == dNodeSize_)
			{
				indexNode_(i) = -1;
			}
		}
	}
	
	for (int i = 0 ; i < fNodeSize_ ; ++i){
		for (int j = 0 ;  j < 3; ++j){
			fDofs(3*i+j) = dof_(fNodes(i)-1,j);
		}
	}

	for (int i=0 ; i < iNodeSize_ ; ++i){
		for (int j = 0 ; j < 3 ; ++j)
		{
			iDofs(3*i+j) = dof_(iNodes(i)-1,j);
		}
	}

	//Computation of the organized Matrices
	VectorXi sortmap(3*(fNodeSize_+iNodeSize_));

	for (int i = 0 ; i < fNodeSize_ ; ++i){
		for (int j = 0 ;  j < 3; ++j){
			fDofs(3*i+j) = dof_(fNodes(i)-1,j);
			sortmap(dof_(fNodes(i)-1,j)-1) = 3*i+j;
		}
	}

	for (int i=0 ; i < iNodeSize_ ; ++i){
		for (int j = 0 ; j < 3 ; ++j)
		{
			iDofs(3*i+j) = dof_(iNodes(i)-1,j);
			sortmap(dof_(iNodes(i)-1,j)-1) = 3*i+j+3*fNodeSize_;
		}
	}

	int fn = fNodeSize_*3, in = iNodeSize_*3;
	cK_.resize(fn, fn);
	cK_.setZero();
	Kis_.resize(in, fn);
	Ksi_.resize(fn, in);
	MatrixXd Kii(in, in);
	Kii.setZero();
	int r, c;
	double v;
        for (int k=0; k<mK_.outerSize(); ++k){
	  for (SparseMatrix<double>::InnerIterator it(mK_,k); it; ++it){
	    r = it.row();
	    c = it.col();
	    v = it.value();
	    if (r < fn && c < fn){
	      cK_(r,c) = v;
	    }else if (r < fn && c < fn+in){
	      Ksi_.coeffRef(r,c-fn) = v;
	    }else if (r < fn+in && c < fn){
	      Kis_.coeffRef(r-fn,c) = v;
	    }else{
	      Kii(r-fn,c-fn) = v;
	    }
	  }        
	}
	Kii.computeInverse(&KiiInv_);
	// for debug purpose to be removed
	cout << "sK-> computed" << endl;
}

void FEM::sortT()
{
	// sorting triangles according to same tags groups
	int currentTag;
	int currentTagCpt = 0;

	int triSize = triangles_.rows();
	int tagSize = tagsColors_.size();

    countTagTriangle_.resize(tagSize);

	int triCpt = 0;

	for(int t = 0; t < tagSize; ++t)
	{
		currentTag = tagsColors_(t);
		currentTagCpt = 0;

		for(int i = triCpt; i < triSize; ++i)
		{
			if (triangles_(i, 0) == currentTag)
			{
				if (i != triCpt)
				{
					for(int k = 0; k < 4; ++k)
					{
						int tmp = triangles_(triCpt, k);
						triangles_(triCpt, k) = triangles_(i, k);
						triangles_(i, k) = tmp;
					}
				}
				++currentTagCpt;
				++triCpt;		
			}
		}
		countTagTriangle_(t) = currentTagCpt;
	}
}

void FEM::computeInverseForceCondensedK()
{
	if(cK_.size() > 0){
		// Compute the condensed stiffness matrix
	        cK_ -= Ksi_ * KiiInv_ * Kis_;
		cK_.computeInverse(&cK_);
		// for debug purpose, to be removed
		cout << "cK-> computed" << endl;
	}
	else cout << "cK-> not computed. No applied forces." << endl;
}

void FEM::computeDisplacementCondensedK()
{
	VectorXi fDofs(3*fNodeSize_);
	if(fDofs.size() > 0) fDofs.setZero();

	VectorXi iDofs(3*iNodeSize_);
	iDofs.setZero();

	MatrixXd mKDtmp;
        if (dNodeSize_) {
            mKDtmp.resize(3*(fNodeSize_+iNodeSize_),3*dNodeSize_);
            mKDtmp.setZero();
        }

	VectorXi fNodes(fNodeSize_);

	VectorXi iNodes(iNodeSize_);

	int fN = 0;
	int iN = 0;
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		if (tagNodes_(i) == 0)
		{
			iNodes(iN) = i + 1;
			++iN;
		}
		if (tagNodes_(i) == 2)
		{
			fNodes(fN) = i + 1;
			++fN;
		}
	}

	for (int i = 0 ; i < fNodeSize_ ; ++i){
		for (int j = 0 ; j < 3 ; ++j){
			fDofs(3*i+j) = dof_(fNodes(i)-1,j);
		}
	}

	for (int i = 0 ; i < iNodeSize_ ; ++i){
		for (int j = 0 ; j < 3 ; ++j){
			iDofs(3*i+j) = dof_(iNodes(i)-1,j);
		}
	}

	//Computation of the organized Matrices
	for (int i = 0; i < 3*fNodeSize_; ++i){
		for (int j = 0; j < 3*dNodeSize_; ++j){
			mKDtmp(i,j) = mKD_(fDofs(i)-1,j);
		}
	}
	for (int i = 0 ; i < 3*iNodeSize_ ; ++i){
		for (int j = 0 ; j < 3*dNodeSize_; ++j){
			mKDtmp(i+ 3*fNodeSize_, j) = mKD_(iDofs(i)-1,j);
		}
	}

	// Later should eliminate mKD since of no use after computing dK_
	mKD_ = mKDtmp;
	dK_.resize(3*fNodeSize_,3*dNodeSize_);

	if(dK_.size() > 0){
		dK_.setZero();

                Ksd_ = mKDtmp.block(0,0,3*fNodeSize_,3*dNodeSize_);
                Kid_ = mKDtmp.block(3*fNodeSize_,0, 3*iNodeSize_,3*dNodeSize_);
		dK_ = -Ksd_+Ksi_*KiiInv_*Kid_;

                //KK_ = Ksd_.transpose() - Kid_.transpose()*Kii_.inverse()*Kis_;
                //std::cout << "KK:" << KK_ << std::endl;getchar();

		// for debug purpose, to be removed
		cout << "dK-> computed" << endl;
	}
	else cout << "dK-> not computed. No applied forces or displacement." << endl;

	cout << "------------------------------------------------------" << endl << endl;
}	

// For debug purpose, should be removed
void FEM::saveWhatEver(const string & fileName) const
{
	ofstream fId;

	fId.open(fileName.c_str(), ios::out | ios::trunc);
	if (fId.is_open())
	{
		fId << mK_ << endl << endl << endl;
		fId.close();
	}
	else{
		cerr << "Problem in creating " << fileName << endl;
	}	
}

// Some useful gets
const MatrixXi & FEM::getTriangles() const{
	return triangles_;
}

const VectorXi & FEM::getIndexNode() const{
   return indexNode_;
}

const MatrixXd & FEM::getNodes() const{
   return nodeCoord_;
}

const MatrixXd & FEM::getcK() const{
	return cK_;
}
#if 1
void FEM::setcK(const MatrixXd &m){
    cK_ = m;
}
#endif

const MatrixXd & FEM::getdK() const{
	return dK_;
}

const MatrixXf & FEM::getColors() const{
	return colors_;
}
const VectorXi & FEM::getTagColor() const{
	return tagsColors_;
}
const VectorXi & FEM::getCountTagTriangle() const{
	return countTagTriangle_;
}


// Definition of VIRTUAL FEM class
// End

///////////////////////////////////////////////////////////////////////////

// Definition of the GMSH class derived from FEM
// Begin

// This scans the inp file for basic information on the object (no mesh yet)
GMSH::GMSH(const string & fileName)
{
	string line;
	string directory;
	ifstream fInp;

	bool pb = false;
	
	fInp.open(fileName.c_str());

	if (fInp.is_open())
	{
		cout << "FileName " << fileName << " is opened" << endl;

		while (! fInp.eof() )
		{
			getline (fInp, line);
			// removing carriage return if necessary
			if(line[line.size()-1] == '\r')	line = line.substr(0, line.size()-1);

			size_t found = line.find(".msh");
			if (found != string::npos)
			{
				directory = fileName.substr( 0, fileName.find_last_of( '/' ) +1 ).c_str();

				fMsh_.open((directory + line).c_str());
				if (fMsh_.is_open())
				{
					//cout << "FileName " << line << " is opened" << endl;
					pb = false;
				}
				else{
					cerr << "FileName " << line << " not found" << endl;
					pb = true;
				}
				break;
			}
		}
		// problem in finding the .msh file
		if (fInp.eof() || pb)
		{
			cerr << "Problem in finding the .msh file GMSH::GMSH! Copy this file and try again" << endl;
			assert(!"Problem in finding the .msh file GMSH::GMSH!");
		}

		// First we need the element type ELEMENT-TYPE
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("ELEMENT-TYPE");
			if (found != string::npos){
				break;
			}
		}		
		ElementType et;
        if(!fInp.eof())
		{
			fInp >> et;
			if (et < 0 || et >= sizeof(ElementType) ){
				cout << "Number of Material is not correct" << endl; // to be removed
				et = TETRAHEDRON4; // we rather should exit at this point
			}
		}
		else{ // Default
			cout << "Type of Finite Elt not found default is TETRAHEDRON4" << endl; // to be removed
			et = TETRAHEDRON4; // we rather should exit at this point
		}

		// Find MATERIAL keyword
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("MATERIAL");
			//cout << line << endl; // to be removed
			if (found != string::npos){
				break;
			}
		}		
        if(!fInp.eof())
		{
			int nbrMat = 0;
			fInp >> nbrMat;
			if (nbrMat <= 0){
				cout << "Number of Material is not correct" << nbrMat << endl; // to be removed
			}
			else{
				cout << "Number of Material is " << nbrMat << endl; // to be removed
				nbrDefMat_ = nbrMat;
			}
		}
		else{ // Default
			cout << "Number of Material not found Default is 1" << endl; // to be removed
			nbrDefMat_ = 1;
		}
		
		// Creation of as much Elements as materials 
		fe_.resize(nbrDefMat_);
		// Now reading the materials and filling
		for(int i=0; i<nbrDefMat_; ++i)
		{
			double young, poisson, rho;
			fInp >> young; 
			fInp >> poisson;
			fInp >> rho;
			cout << "Material " << i << " ; Young = " << young << " ; Poisson = " << poisson << " ; Rho = " << rho << endl ;
			switch (et)
			{
			case TETRAHEDRON4:
				fe_[i] = new TetrahedronElement (Material(young, poisson, rho));
				break;
			case TETRAHEDRON10:
				cerr << "Not yet implemented" << endl;
				break;
			case CUBIC8:
				cerr << "Not yet implemented" << endl;
				break;
			case PRISMATIC6:
				cerr << "Not yet implemented" << endl;
				break;
			case SHELL:
				cerr << "Not yet implemented" << endl;
				break;
			default:
				assert(!"sould really not be here at all GMSH::GMSH(const string &)!");
			}
		}

		// Reading GEOMETRY VOLUMES 
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found;
			found = line.find("GEOMETRY-VOLUME");
			if (found != string::npos){
				break;
			}
		}
		int nbrGeoVol;
        if(!fInp.eof())
		{
			fInp >> nbrGeoVol;
			if (nbrGeoVol <= 0){
				cout << "Number of GEOMETRY-VOLUME is not correct" << endl;
				nbrGeoVol = 1;
			}
			else{
				cout << endl << "Number of GEOMETRY-VOLUME is " << nbrGeoVol << endl << endl;
			}
		}
		else{ // Default
			cout << "GEOMETRY-VOLUME not found must be at least one" << endl; // to be removed
			nbrGeoVol = 1;
		}
		geoVol_.resize(nbrGeoVol,2);
		for(int i = 0; i < nbrGeoVol; ++i){
			fInp >> geoVol_(i,0) >> geoVol_(i,1) ;
		}
		cout << "Associations between materials and geometric volumes" << endl << geoVol_ << endl << endl;

		// Find PHYSICAL-VOLUME keyword
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found;
			found = line.find("PHYSICAL-VOLUME");
			if (found != string::npos){
				break;
			}
		}

        if(!fInp.eof())
		{
			fInp >> phyVolId_;
		}
		else{ // Default
			cout << "PHYSICAL-VOLUME not found must be at least one" << endl; // to be removed
			phyVolId_ = -1;
		}
		cout << "Index of PHYSICAL-VOLUME = " << phyVolId_ << endl << endl;

		// Find DIRICHLET-BC keyword
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("DIRICHLET-BC");
			if (found != string::npos){
				break;
			}
		}
		int nbrDBC = 0;
        if(!fInp.eof())
		{
			fInp >> nbrDBC;
			cout << "Number of Dirichlet faces = " << nbrDBC << endl;
			if (nbrDBC < 0){
				cout << "Number of DIRICHLET-BC is not correct. Set to 0." << endl; // to be removed
				nbrDBC = 0;
			}
		}
		else{ // Default
			cout << "DIRICHLET-BC not found. It must be at least zero." << endl; // to be removed
			nbrDBC = 0;
		}
		dirichlet_.resize(nbrDBC);
		if(nbrDBC > 0){
			for(int i = 0; i < nbrDBC; ++i){
				fInp >> dirichlet_[i];
			}
			cout << "List of Dirichlet faces indexes = " << endl << dirichlet_ << endl;
		}

		// Find EFFORT-BC keyword
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("EFFORT-BC");
			if (found != string::npos){
				break;
			}
		}
		int nbrEff = 0;
        if(!fInp.eof())
		{
			fInp >> nbrEff;
			cout << "Number of Effort faces = " << nbrEff << endl;
			if (nbrEff < 0){
				cout << "Number of EFFORT-BC is not correct. Set to 0." << endl; // to be removed
				nbrEff = 0;
			}
		}
		else{ // Default
			cout << "EFFORT-BC not found. It must be at least zero." << endl; // to be removed
			nbrEff = 0;
		}
		effort_.resize(nbrEff);
		if(nbrEff > 0){
			for(int i = 0; i < nbrEff; ++i){
				fInp >> effort_[i];
			}
			cout << "List of Effort faces indexes = " << endl << effort_ << endl;
		}

		if(nbrDBC == 0 && nbrEff == 0){
			cerr << "Error: need at least one surface (applied forces or Dirichlet)." << endl;
			return;
		}

		// setting the colors
		// Find COLORS keyword
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("COLORS");
			if (found != string::npos){
				break;
			}
		}
		if(!fInp.eof())
		{
			colors_.resize(nbrEff+nbrDBC, 4);
			tagsColors_.resize(nbrEff+nbrDBC);
			for(int i = 0; i < nbrEff+nbrDBC; ++i){
				fInp >> tagsColors_(i) >> colors_(i, 0) >> colors_(i, 1) >> colors_(i, 2) >> colors_(i, 3); 
			}
		}
		else{ // Default
			cout << "COLORS not found. Completed by default color." << endl;
			colors_.resize(nbrEff+nbrDBC, 4);
			tagsColors_.resize(nbrEff+nbrDBC);
			for(int i = 0; i < nbrDBC; ++i)
			{	// default color red
				tagsColors_(i) = dirichlet_[i];
				colors_(i, 0) = 0.5f; 
				colors_(i, 1) = 0.1f;
				colors_(i, 2) = 0.0f;
				colors_(i, 3) = 0.5f;
			}
			for(int i = nbrDBC; i < nbrDBC+nbrEff; ++i)
			{	// default color red
				tagsColors_(i) = effort_[i];
				colors_(i, 0) = 0.4f; 
				colors_(i, 1) = 0.0f;
				colors_(i, 2) = 0.6f;
				colors_(i, 3) = 0.5f;
			}
		}
		while (! fInp.eof())
		{
			getline (fInp, line);
			size_t found = line.find("MASS-PROPERTY");
			if (found != string::npos){
				break;
			}
		}
		if(!fInp.eof())
		{
		    fInp >> mass_;
		    for (int i=0; i<3; i++) fInp >> com_[i];
		    for (int i=0; i<9; i++) fInp >> I_[i];
		}

		cout << "End of the reading of the .inp file." << endl << endl ;

		// Now we have all in hand to scan the .msh file
		scanFile();
	}
	else{
		cerr << "FileName " << fileName << "not found" << endl;
	}
	fInp.close();
	
}

GMSH::~GMSH()
{
	fMsh_.close();
}

bool GMSH::isDirichlet(int index) const
{
	int dir = dirichlet_.size();
	for(int i = 0; i < dir; ++i)
	{
		if(dirichlet_[i] == index)
			return true;
	}
	return false;
}

bool GMSH::isEffort(int index) const
{
	int dir = effort_.size();
	for(int i = 0; i < dir; ++i)
	{
		if(effort_[i] == index)
			return true;
	}
	return false;
}

int GMSH::whatMaterial(int volID) const
{
	int size = geoVol_.rows();
	bool MatFound = false;
	for(int i = 0; i < size; ++i)
	{
		if(geoVol_(i,0) ==  volID)
		{
			MatFound = true;
			return geoVol_(i,1);
		}
	}
	if(MatFound == false)
	{
		cerr << "GMSH::whatMaterial-> Material not found in geoVol_ returning -1" << endl;
		return -1;
	}
	
	return -1; // only not to have a warning in compilation
}

// This is the scan msh file
void GMSH::scanFile()
{
	string line;
	// check that .msh file is really open (not necessary)
	if (fMsh_.is_open()){
		cout << "Scanning the .msh file..." << endl;
	}
	else{
		cerr << "GMSH::scanFile-> the .msh file ID should be OK: fatal error!" << endl;
		return;
	}
	// Read number of Nodes
	while (! fMsh_.eof())
	{
		getline (fMsh_, line);
		size_t found = line.find("$Nodes");
		if (found != string::npos){
			break;
		}
	}
	if(!fMsh_.eof())
	{
		fMsh_ >> nbrNodeT_;
		cout << "Number of Nodes = " << nbrNodeT_ << endl; // to be removed
		if (nbrNodeT_ <= 0){
			cerr << "GMSH::scanFile-> Number of Nodes is not correct" << endl; 
			return;
		}
	}
	else{ // Default
		cout << "GMSH::scanFile-> Number of Nodes is mandatory in .msh file (key: $Nodes)" << endl; 
		return;
	}
	// Get the Nodes
	nodeCoord_.resize(nbrNodeT_, 3);
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		int nodeID;
		fMsh_ >> nodeID >> nodeCoord_(i,0) >> nodeCoord_(i,1) >> nodeCoord_(i,2);
		if (nodeID != i+1){
			cerr << "GMSH::scanFile-> node mismatch while Scanning, should stop, check the .msh File" << endl;
			return;
		}
	}
	//cout << "List of Nodes coordinates " << endl << nodeCoord_ << endl ;
	// reading Elements which gathers physical, geometrical volumes and Dirichlet + free triangles
	// Read number of Nodes
	while (! fMsh_.eof())
	{
		getline (fMsh_, line);
		size_t found = line.find("$Elements");
		if (found != string::npos){
			break;
		}
	}
	int nbrElts;
	if(!fMsh_.eof())
	{
		fMsh_ >> nbrElts;
		if (nbrElts <= 0){
			cerr << "GMSH::scanFile-> Number of Elements is not correct" << endl; 
			return;
		}
	}
	else{ // Default
		cout << "GMSH::scanFile-> Number of Elememts is mandatory in .msh file (key: $Elements)" << endl; 
		return;
	}
	cout << "Total number of elements = " << nbrElts << endl;
	// temporary Triangle list oversized!
	MatrixXi tmpTri(nbrElts, 5); // 5 is ID EltGeo NbrTags Tag1 n1 n2 n3
	int dTriSize = 0;
	int fTriSize = 0;
	int ii = 0;
	int tmp;
	while(! fMsh_.eof())
	{
		fMsh_ >> tmp;

		if(tmp != ii+1){
			cerr << "GMSH::scanFile-> Triangle Elements numbering mismatch: " 
				<< tmp << " != " << ii+1 <<endl;
			return;
		}

		fMsh_ >> tmp; 
		if (tmp == 2) // 2 is the ID number for a triangle  
		{
			fMsh_ >> tmp; // number of tags
			int nbrTag = tmp;
			for(int j = 0; j < nbrTag; ++j)
			{ // reading tags
				fMsh_ >> tmp;
				if (j == 0) // we only need the first tag
				{
					if(isDirichlet(tmp)) // checks if Dirichlet Node
					{
						tmpTri(ii,0) = 0; // 0 is Dirichlet
						++dTriSize;
					}
					else
					{
						if(isEffort(tmp))
						{
							tmpTri(ii,0) = 1; // 1 is Deformable surface triangle
							++fTriSize;
						}
						else{
							tmpTri(ii,0) = 2; // 2 is for anything else GMSH might have additional surfaces...
						}
					}
					tmpTri(ii,1) = tmp;
				}
			}
			// save triangles nodes
			fMsh_ >> tmpTri(ii,2) >> tmpTri(ii,3) >> tmpTri(ii,4);
			++ii;
		}
		else{
			//cout << "Tetrahedron reached for Elts " << ii << endl;
			break;
		}
	}
	// We copy the content of tmpTri to triangles_ because 
	// now we know the sizes
	// We also fill tagNodes_ 0 = INTERN NODES; 1 = DISPLACEMENT/DIRICHLET; 2 = FORCE; 3 = RIGID 
	tagNodes_.resize(nbrNodeT_);
    tagNodes_.setZero();

	if (ii < nbrElts && ! fMsh_.eof())
	{
		triangles_.resize(dTriSize + fTriSize, 4); // Triangles list (surfID, node1, node2, node3)
		
		int d = 0; 
		int f = 0; 
		
		for(int j = 0; j < ii; ++j)
		{
			if (tmpTri(j,0) == 0) // Dirichlet
			{						
				triangles_(d + fTriSize,0) = tmpTri(j,1);
				triangles_(d + fTriSize,1) = tmpTri(j,2);
				triangles_(d + fTriSize,2) = tmpTri(j,3);
				triangles_(d + fTriSize,3) = tmpTri(j,4);

				// We can tag Dirichlet nodes because we would like them to be fixed
				tagNodes_(tmpTri(j,2)-1) = 1;
				tagNodes_(tmpTri(j,3)-1) = 1;
				tagNodes_(tmpTri(j,4)-1) = 1;

				++d;
			}
			else
			{
				if(tmpTri(j,0) == 1) // Free/Effort/Surface deformable
				{
					triangles_(f,0) = tmpTri(j,1);
					triangles_(f,1) = tmpTri(j,2);
					triangles_(f,2) = tmpTri(j,3);
					triangles_(f,3) = tmpTri(j,4);		

					++f;

					// We do not tag the force nodes at this moment
				}
			}
		}
		for(int j = 0; j < ii; ++j)
		{
			if (tmpTri(j,0) == 1)
			{
				if(tagNodes_(tmpTri(j,2)-1) != 1) tagNodes_(tmpTri(j,2)-1) = 2; 
				if(tagNodes_(tmpTri(j,3)-1) != 1) tagNodes_(tmpTri(j,3)-1) = 2;
				if(tagNodes_(tmpTri(j,4)-1) != 1) tagNodes_(tmpTri(j,4)-1) = 2;
			}
		}
	}
	else
	{
		cerr << "GMSH::scanFile-> Problem in reading surface data in .msh" << endl;
		return;
	}

	// Set sizings
	fNodeSize_ = 0;
	iNodeSize_ = 0;
	dNodeSize_ = 0;
	rNodeSize_ = 0;
	for(int i = 0; i < nbrNodeT_; ++i)
	{
		if (tagNodes_(i) == 0) ++iNodeSize_; // internal node
		if (tagNodes_(i) == 1) ++dNodeSize_; // Dirichlet node
		if (tagNodes_(i) == 2) ++fNodeSize_; // effort node
		if (tagNodes_(i) == 3) ++rNodeSize_; // rigid node // to be implemented later
	}

	// Erase tmpTri, should change this
	tmpTri.resize(1,1);

	// Reading the remaining Elements (certainly Tetrahedrons)
	int nbrTriangles = ii;
	cout << "Number of surface Triangles = " << nbrTriangles << endl ;
	nbrElts -= ii; // right size of volumic elements
	nbrElts_ = nbrElts; 
	conTable_.resize(nbrElts, 4); // Elements list
	conMat_.resize(nbrElts);      // Material per element list
	ii = 0;
	cout << "Number of volumic elements = " << nbrElts << endl;
	cout << "Start processing Volumic Elements..." << endl;
	while(! fMsh_.eof() && tmp == 4) // Not end of .msh file and volumic element = 4
	{
		fMsh_ >> tmp; // number of tags
		int nbrTag = tmp;
		for(int j = 0; j < nbrTag; ++j)
		{ // passing/reading tags, if needed do something here
			fMsh_ >> tmp;
			if (j == 0 && tmp != phyVolId_) // First tag is not the expected physical volume
			{
				cerr << "GMSH::scanFile-> more than one Physical volume not allowed, error in .msh file" << endl;
				return;
			}
			else
			{
				if (j == 1)
				{
					int mat = whatMaterial(tmp);
					if (mat < 0 || mat > nbrDefMat_)
					{
						cerr << "GMSH::scanFile-> Material mismatch, error in .msh file" << endl;
						return;
					}
					else{
						conMat_[ii] = mat;
					}
				}
			}
		}
		// save volumic nodes
		fMsh_ >> conTable_(ii,0) >> conTable_(ii,1) >> conTable_(ii,2) >> conTable_(ii,3);
		++ii;
		if (ii == nbrElts)
			break;
		fMsh_ >> tmp; 
		if (tmp != ii+nbrTriangles+1){cerr << "volume Elements numbering mismatch:" << tmp << " != " << ii+nbrTriangles+1 << endl; return;}    // Cet affichage d'erreur pose probleme en fin de fichier
		fMsh_ >> tmp; 
	}
	if (ii != nbrElts)
	{
		cerr << "GMSH::scanFile-> problem in reading the volumic elements in .msh  : " << ii << " != " << nbrElts << endl;
		return;
	}

	// DOF matrix building (not from .msh file)
	dof_.resize(nbrNodeT_, 3);	// DOF Matrix
	int cursP = 0;				// Positif cursor for non-Dirichlet nodes
    int cursN = 0;				// Negatif cursor for Dirichlet nodes
	
	for (int node = 0; node < nbrNodeT_; ++node)
	{
		if (tagNodes_(node) == 1)
		{
			for (int i = 0; i < 3; ++i)
			{
				--cursN;
				dof_(node, i) = cursN;
			}
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				++cursP;
				dof_(node, i) = cursP;
			}
		}
	}
}

// Definition of  GMSH class derived from FEM
// End
}
