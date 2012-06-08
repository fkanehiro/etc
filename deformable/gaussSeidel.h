#ifndef _GAUSS_SEIDEL_
#define _GAUSS_SEIDEL_

#include <afmaths/types.h>

#include "afdyn/DefaultDynamicsSimulator.h"

#include <vector>

namespace afdyn
{	
	//helper function of Gauss Seidel
	int eigen(const afbase::matrix3d & M, double & x0, double & x1);
	
	//newtonCoulomb3
	void solve6Full (afbase::matrix6d M, afbase::vector6d & x, const afbase::vector6d & b);
	void newtonCoulomb3(
			afbase::vector3d & f, const afbase::matrix3d & L, 
			afbase::vector3d & v, const afbase::vector3d & vFree, double mu);	

	void gaussSeidel(
		const int & numInteraction,
		std::vector <detail::InteractionData *> & interactionVector,
		const std::vector < std::vector < afbase::matrix3d > > & lambdaMatrix
		);
		

	// First optimization
	void solve6FullSpecific (
		const afbase::matrix6d & M, 
		afbase::vector6d & x_check, 
		const afbase::vector6d & b
	);
	
		
	// Second optimization
	void solve6FullSpecific_v2 (
		double inv_M00_2_2,
		const afbase::matrix3d & M01,
		const afbase::matrix3d & M10,
		const afbase::matrix3d & M11,
		afbase::vector3d & x_up, afbase::vector3d & x_dn, 
		const afbase::vector6d & b
	);
	void newtonCoulomb3_v2(
			afbase::vector3d & f, const afbase::matrix3d & L, 
			afbase::vector3d & v, const afbase::vector3d & vFree, double mu);
	
	void gaussSeidel_v2(
		const int & numInteraction,
		std::vector <detail::InteractionData *> & interactionVector,
		const std::vector < std::vector < afbase::matrix3d > > & lambdaMatrix
		);

}

/*!	\fn int eigen(tvmet::Matrix<double,2,2> M, double& x0, double& x1)
*	\brief Finds eigen values of a 2x2 matrix
*	\param M (matrix)
*	\return x0, x1 (eigen values)
*/

/*! 
*	\fn void afdyn::solve6Full (matrix6d M, vector6d& x, const vector6d& b);
*	\brief solve the equation x | p = M * x
*	\param M (size 6x6) is a 6x6 inversible matrix
*	\param x (size 6) is the result
*	\param b (size 6)
*	\return : none
*/

/*! 
*	\fn void afdyn::solve6Full (matrix6d M, vector6d& x, const vector6d& b);
*	\brief solve the equation x | p = M * x
*	\param M (size 6x6) is a 6x6 inversible matrix and has the following shape:
*          (1, 0, 0,  X, X, X)
*          (0, 1, 0,  X, X, X)
*          (0, 0, X,  0, 0, 0)
*     	   (X, X, 0,  X, X, X)
*    	   (X, X, 0,  X, X, X)
*          (0, 0, 1,  X, X, X)
*	\param x (size 6) is the result
*	\param b (size 6)
*	\return : none
*/


/*!	\fn void newtonCoulomb3(vector3d & f, const matrix3d &  L, vector3d &  v, const vector3d &  vFree, double mu)
*	\brief Solves contact force with friction with Newton method at one contact point
*	\param L (inertia), v (velocity of contact points), vFree (free velocity of contact points), mu (coefficient of friction)
*	\param f (contact force), v (velocity)
*	\return 
*/

/*!
*	\fn		void afdyn::DefaultDynamicsSimulator::gaussSeidel( const int & numContact,	std::vector <detail::InteractionData *> & collisionVector,	const std::vector < std::vector < matrix3d > > & lambdaMatrix );
*	\brief	Solves contact forces with friction using a Gauss-Seidel like solver coupled with Newton-Coulomb method on whole system
*/

/////////////////////////



/*! 
*	\fn void afdyn::solve6FullSpecific_v2 (
		double inv_M00_2_2,
		const afbase::matrix3d & M01,
		const afbase::matrix3d & M10,
		const afbase::matrix3d & M11,
		afbase::vector3d & x_up, afbase::vector3d & x_dn, 
		const afbase::vector6d & b
	);
*	\brief solve the equation x | p = M * x
*	The Matrix M has the following form: 
*          (1, 0, 0,  			       )
*          (0, 1, 0,  			  M01  )
*          (0, 0, M00(2,2),            )
*     	   (                           )
*    	   (  M10                 M11  )
*     	   (                           )
* 	only part this matrix are used:
*	\param inv_M00_2_2 = 1/M00(2,2)
*	\param M01 the 3.3 upper right matrix
*	\param M10 the 3.3 low left matrix
*	\param M11 the 3.3 low right matrix
*	\param x_up (size 3) is the result x = (x_up, x_dn)
*	\param x_dn (size 3) 
*	\param b (size 6)
*	\return : none
*/

/*!	\fn void newtonCoulomb3_v2(vector3d & f, const matrix3d &  L, vector3d &  v, const vector3d &  vFree, double mu)
*	\brief Solves contact force with friction with Newton method at one contact point
*	\param L (inertia), v (velocity of contact points), vFree (free velocity of contact points), mu (coefficient of friction)
*	\param f (contact force), v (velocity)
*	\return 
*/


/*!
*	\fn		void afdyn::gaussSeidel_v2( const int & numContact,	std::vector <detail::InteractionData *> & collisionVector,	const std::vector < std::vector < matrix3d > > & lambdaMatrix );
*	\brief	Solves contact forces with friction using a Gauss-Seidel like solver coupled with Newton-Coulomb method on whole system
*/

#endif
