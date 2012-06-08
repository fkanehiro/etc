#include "gaussSeidel.h"
#include "afdyn/PointInteraction.h"
#include "afcoll/PointCollision.h"

namespace afdyn
{
	// --
	//newtonCoulomb3_v2
	// In this version, the matrix M is not completly built
	void newtonCoulomb3_v2(vector3d & f, const matrix3d & L, vector3d & v, const vector3d & vFree, double mu)
	{
		int k = 0;
		double rhoT = 0.0;

		// Buffer variables
		double x0 (0);
		double x1 (0);
		double LBuf_00;
		double LBuf_01;
		double LBuf_11;


		vector3d buf3 (0,0,0);

		//Only part of the matrix dphi is built.
		// For the upper left part, only the term in the bottom right is used
		afbase::matrix3d dphi_10(0); 
		afbase::matrix3d dphi_11(0); 

		afbase::vector6d phi   ; phi.fill(0);

		afbase::vector3d buf_up;
		afbase::vector3d buf_dn;

		if (eigen(L, x0, x1))
			rhoT =x0 / (x1 * x1);
		else
		{
			std::cerr << "Problem for rhoT" << std::endl;
			return;
		}

		// Built Phi and dPhi (PhD thesis of Christian Duriez, p.72)

		//--------------------------------------------------
		// Fill part of D_phi_0
		// The following operation are grouped
		//°the dPhi is Initialize to 0 then :
		//  dPhi(1:3,1:3)=Id(3x3)
		//  dPhi(1:3,4:6)=-L
		//°dPhi(5,2)=1/L(2,2)
		//°switch lines 2 & 5 for matrix inversion


		//The upper right of the matrix dPhi is the identity, except the term in(2,2)
		// as a simplification, this part is not defined.
		//dphi_00[0] = 1.0;	dphi_00[4] = 1.0;	dphi_00[8] = 1.0 / L[8];
		afbase::matrix3d dphi_01(
			-L[0],	-L[1], -L[2],
			-L[3],	-L[4], -L[5],
			0,0,0
		);

		// Compute the last line of the matrix dphi
		dphi_10[8] = 1.0;		
		dphi_11[6] = -L[6];		dphi_11[7] = -L[7];		dphi_11[8] = -L[8];

		// End fill part of D_phi_0
		//--------------------------------------------------

		//--------------------------------------------------
		// Main loop

		do
		{	
			//--------------------------------------------------
			// Fill _phi_0

			// _phi_0(1:3)=vel-vel_free-L*f
			buf3 = L * f;
			buf3 += vFree;
			buf3 *= -1;
			buf3 += v;
			
			phi(0) = buf3(0);
			phi(1) = buf3(1);
			phi(5) = buf3(2);

			// _phi_0(2)=vel_n/L(1,1)
			phi(2) = v(2) / L[8];

			// norm=norm(f_0(2:3)-rho_t * vel_0(2:3))
			//buf3 = f - v * rhoT;
			buf3[0] = v[0] * (- rhoT) + f[0];
			buf3[1] = v[1] * (- rhoT) + f[1];
			// buf3[2] = v[2] * (- rhoT) + f[2]; // This computation is not necessary
			
			double norm = sqrt(buf3[0] * buf3[0] + buf3[1] * buf3[1]);

			double mu_norm = mu / norm;
			double mu_f2_norm = mu_norm * f(2);

			// _phi_0(5:6)=f_0(2:3)-mu*f_0(1)*(f_0(2:3)-rho_t*vel_0(2:3))/norm
			phi(3) = f(0) - mu_f2_norm * buf3[0]; 
			phi(4) = f(1) - mu_f2_norm * buf3[1];

			// End fill _phi_0
			//--------------------------------------------------

			//--------------------------------------------------
			// Fill D_phi_0

			// dPhi(5:6,4)=-(mu/norm(f_0(2:3)-rho_t*vel(2:3)))*(f_0(2:3)-rho_t*vel(2:3))
			dphi_11[2] = -mu_norm * buf3[0];
			dphi_11[5] = -mu_norm * buf3[1];
			
			// Compute derivate of V/norm(V)
			double norm3 = norm * norm * norm; 

			// dL=[v2^2 -v1*v2; -v2*v1 v1^2]/(norm(V)^3)
			LBuf_00 =  buf3[1] * buf3[1] / norm3;
			LBuf_01 = -buf3[0] * buf3[1] / norm3;
			LBuf_11 =  buf3[0] * buf3[0] / norm3;

			double  mu_f2 = mu * f(2);
			double  rho_mu_f2 = rhoT * mu_f2;

			{
				dphi_10[0] = rho_mu_f2 * LBuf_00;
				dphi_10[1] = dphi_10[3] = rho_mu_f2 * LBuf_01;

				dphi_10[4] = rho_mu_f2 * LBuf_11;
				dphi_11[0] = -mu_f2    * LBuf_00 + 1.0;
				dphi_11[1] = dphi_11[3] = -mu_f2    * LBuf_01;

				dphi_11[4] = -mu_f2    * LBuf_11 + 1.0;
			}
			
			// End fill D_phi_0
			//--------------------------------------------------

			//--------------------------------------------------
			// Invert system

			afdyn::solve6FullSpecific_v2(
				1.0 / L[8], dphi_01, dphi_10, dphi_11,
				buf_up,buf_dn, phi);
			
			// End invert system
			//--------------------------------------------------

			// Update forces and accelerations
			v -= buf_up;
			f -= buf_dn;
			
			++k;
			if (k > 100)
				break;
		}
		while ( phi.norm() > 1e-5);

		// End main loop
		//--------------------------------------------------
	}
	
	// solve6FullSpecific_v2
	void solve6FullSpecific_v2 (
		double M_22,
		const afbase::matrix3d & M01,
		const afbase::matrix3d & M10,
		const afbase::matrix3d & M11,
		afbase::vector3d & x_up, afbase::vector3d & x_dn,
		const afbase::vector6d & b
	)
	{
		//In this version, the matrices U_01 and L_10 are not exactly computed
		// There are simplified to the matrices M01 and M10 (resp.) since the 
		// used parts correspond.

		// eg. U_01: only the indexes 0..5 are used. There, U_01 = M01
		// U_01 (M(0,3), M(0,4), M(0,5), M(1,3), M(1,4), M(1,5), 0, 0, 0);

		// eg. L_10: only the indexes 0,1, 3,4 are used. There, L_10 = M10
		// L_10 (M(3,0), M(3,1), 0, M(4,0), M(4,1), 0, 0, 0, 0);
		const afbase::matrix3d & U_01 (M01);
		const afbase::matrix3d & L_10 (M10);

		double inv_L_10_8 = 1./M_22;
		
		// Previously U_11 (M(3,3), M(3,4), M(3,5), 0, M(4,4), M(4,5), 0,0,M(5,5))
		// It can be approximated to M11, since the indexes 3,6,7 are not used.
		afbase::matrix3d U_11 (M11);
		
		afbase::matrix3d M_11 (M11);
		
		//pivot Gauss
		M_11(0,0) -= L_10[0] * M01(0,0);
		M_11(0,1) -= L_10[0] * M01(0,1);
		M_11(0,2) -= L_10[0] * M01(0,2);

		M_11(1,0) -= L_10[3] * M01(0,0);
		M_11(1,1) -= L_10[3] * M01(0,1);
		M_11(1,2) -= L_10[3] * M01(0,2);

		M_11(0,0) -= L_10[1] * M01(1,0);
		M_11(0,1) -= L_10[1] * M01(1,1);
		M_11(0,2) -= L_10[1] * M01(1,2);

		M_11(1,0) -= L_10[4] * M01(1,0);
		M_11(1,1) -= L_10[4] * M01(1,1);
		M_11(1,2) -= L_10[4] * M01(1,2);

		M_11(2,0) -= inv_L_10_8 * M01(2,0);
		M_11(2,1) -= inv_L_10_8 * M01(2,1);
		M_11(2,2) -= inv_L_10_8 * M01(2,2);

		const double L_11__3 = M_11(1,0) / M_11(0,0);
		M_11(1,1) -= L_11__3 * M_11(0,1);
		M_11(1,2) -= L_11__3 * M_11(0,2);

		const double L_11__6 = M_11(2,0) / M_11(0,0);
		M_11(2,1) -= L_11__6 * M_11(0,1);
		M_11(2,2) -= L_11__6 * M_11(0,2);

		const double L_11__7 = M_11(2,1) / M_11(1,1);
		M_11(2,2) -= M_11(1,2);

		U_11[0] -= L_10[0] * U_01[0] + L_10[1] * U_01[3];
		U_11[1] -= L_10[0] * U_01[1] + L_10[1] * U_01[4];
		U_11[2] -= L_10[0] * U_01[2] + L_10[1] * U_01[5];

		U_11[4] -= L_10[3] * U_01[1] + L_10[4] * U_01[4] + L_11__3 * U_11[1];
		U_11[5] -= L_10[3] * U_01[2] + L_10[4] * U_01[5] + L_11__3 * U_11[2];

		U_11[8] -= L_11__6 * U_11[2] + L_11__7 * U_11[5];

		// Resolution of mat*x=b
		//x = b;	// Update right hand side
		for (unsigned i=0; i<3; ++i)
		{
			x_up[i] = b[i];
			x_dn[i] = b[i+3];
		}

		x_dn[0] -= L_10[0] * x_up[0] + L_10[1] * x_up[1]; 
		x_dn[1] -= L_10[3] * x_up[0] + L_10[4] * x_up[1] + L_11__3 * x_dn[0]; 
		x_dn[2] -= inv_L_10_8 * x_up[2] + L_11__6 * x_dn[0] + L_11__7 * x_dn[1];  

		// Back substitution
		x_dn[2] /= U_11[8];

		x_dn[1] -= U_11[5] * x_dn[2];
		x_dn[1] /= U_11[4];

		x_dn[0] -= U_11[2] * x_dn[2] + U_11[1] * x_dn[1];
		x_dn[0] /= U_11[0];

		x_up[2] /= M_22;
		x_up[1] -= U_01[5] * x_dn[2] + U_01[4] * x_dn[1] + U_01[3] * x_dn[0];
		x_up[0] -= U_01[2] * x_dn[2] + U_01[1] * x_dn[1] + U_01[0] * x_dn[0];
	}
	
		//gaussSeidel
	void gaussSeidel_v2(
		const int & numInteraction,
		std::vector <detail::InteractionData *> & interactionVector,
		const std::vector < std::vector < matrix3d > > & lambdaMatrix
		)
	{
		// Precision and convergence variables
		double epsilon1 = 1e-5;
		double epsilon2 = 1e-5;
		double norm = 0;
		double delta = 0;

		// Dynamic variables
		vector3d velt (0,0,0);

		// Buffer variables
		vector3d buf(0,0,0);
		vector3d vBuf(0,0,0);
		vector3d fBuf(0,0,0);

		int k=0;

		// Miscellaneous variables
		// Extract 3x3 block matrix from Lambda^(-1) and free velocity
		for (int i=0; i<numInteraction; ++i)
		{
			detail::InteractionData* interactionData_i = interactionVector[i];
			PointInteraction* interaction_i = interactionData_i->pointInteraction_;

			//initialization of the parameters
			interactionData_i->frictionCoeff_ = interaction_i->getFrictionCoef();
			interactionData_i->force1_ = interactionData_i->force_;

			//Compute the inverse of each matrix in the diagonal
			interactionData_i->inverselambdaMatrix_ = lambdaMatrix[i][i].inverse( );
		}

		//--------------------------------------------------
		// Main loop

		do
		{
			for (int i=0; i<numInteraction; ++i)
			{
				detail::InteractionData* interactionData_i = interactionVector[i];
				PointInteraction * interaction_i = interactionData_i->pointInteraction_;
				afcoll::PointCollision* collision_i = interaction_i->getPointCollision();
				const std::vector <matrix3d> & lambdaMatrix_i = lambdaMatrix[i];

				velt = collision_i->getRelativeVelocity();
				vector3d & fc_i = interactionData_i->force_;

				// Suppress the test i==j, by splitting the for loop into two
				for (int j=0; j<i; ++j)
					if ( interactionVector[j]->activeCollision_ == true)
						velt += lambdaMatrix_i[j] * interactionVector[j]->force_;

				for (int j=i+1; j<numInteraction; ++j)
					if ( interactionVector[j]->activeCollision_ == true)
						velt += lambdaMatrix_i[j] * interactionVector[j]->force_;

				if (velt[2] < -epsilon1) //if there is interpenetration
				{
					fBuf = fc_i;

					vBuf = velt;

					// ftest=-lambda^(-1)*vel
					buf = interactionData_i->inverselambdaMatrix_ * velt;

					fc_i = -buf; // Pure resting

					// Sliding case
					norm = sqrt(fc_i[0] * fc_i[0] + fc_i[1] * fc_i[1]);

					if (norm > interactionData_i->frictionCoeff_ * fabs(fc_i[2]))
					{
						// Resting forces are computed so that the normal force are in the wrong way
						// We put it in the right way so that we can manage it in sliding case
						if (fc_i(2) < 0.0)
							fc_i *= -1;

						// If this contact was active, there are chances that we already applied Newton on it, so
						// we take the value of previously computed force
						if (interactionVector[i]->activeCollision_ == true)
							fc_i = fBuf;
						else
							// Computed force was a resting one
							velt.clear();

						// Newton applied on this contact
						newtonCoulomb3_v2(fc_i, lambdaMatrix_i[i], velt, vBuf, interactionVector[i]->frictionCoeff_);
					}
					interactionVector[i]->activeCollision_ = true;
				}
				// There is no contact
				else 
				{
					interactionVector[i]->activeCollision_ = false; 
					fc_i.clear();
				}
			}

			// Convergence
			delta = 0.0;
			for (int i=0; i<numInteraction; ++i)
			{
				const vector3d & fc_i = interactionVector[i]->force_;
				vector3d & fc1_i = interactionVector[i]->force1_;

				if (fc_i.norm() < 1e-8)
				{
					vector3d fc_fc1 =  fc_i - fc1_i;
					delta += fc_fc1.norm();
				}
				else
				{
					vector3d fc_fc1 =  fc_i - fc1_i;
					delta += fc_fc1.norm() / fc_i.norm();
				}
				fc1_i = fc_i;
			}

			++k;
			if (k > 100)
				break;
		}
		while (delta > epsilon2);

		// End main loop
		//--------------------------------------------------

		// Group forces and active contact status
		for (int i=0; i<numInteraction; ++i)
		{
			detail::InteractionData* id = interactionVector[i];
			PointInteraction* pi = id->pointInteraction_;

			//pass the force into world coordinates
			afcoll::PointCollision* pc = pi->getPointCollision();

			//Compute the matrix of frame change
			const vector3d& tangent1 = pc->getTangent1();
			const vector3d& tangent2 = pc->getTangent2();
			const vector3d& normal = pc->getNormal();

			matrix3d transposedFrame(
				tangent1[0], tangent2[0], normal[0],
				tangent1[1], tangent2[1], normal[1],
				tangent1[2], tangent2[2], normal[2]
			);

			//Compute the force in the good frame
			vector3d force = transposedFrame * interactionVector[i]->force_;

			//Compute the torque in the good frame
			const vector3d & position = pc->getCollisionPoint();
			vector3d torque =  position.cross_product(force);

			//set them
			pi->setInteractionForce( force );
			pi->setInteractionTorque( torque );
		}
	}
}
