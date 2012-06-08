#include "gaussSeidel.h"
#include "afdyn/PointInteraction.h"
#include "afcoll/PointCollision.h"

namespace afdyn
{
	//helper function of Gauss Seidel
	int eigen(const matrix3d & M, double & x0, double & x1)
	{
		// Coefficients of caracteristic polynom
		//A = 1.0;
		double B = -M[0] - M[4];
		double C =  M[0] * M[4] - M[3] * M[1];

		// Resolution of caracteristic polynom
		double delta = B * B - 4.0 * C;
		if (delta >= 0)
		{
			double sqrt_delta = sqrt(delta);
			x0 = (-B - sqrt_delta) * 0.5;
			x1 = (-B + sqrt_delta) * 0.5;
		}
		// patch - if b and c are too small, it can append numerical issues, making delta negative
		else if(delta >= -1e-20)
		{
			double sqrt_delta = sqrt(-delta);
			x0 = (-B - sqrt_delta) * 0.5;
			x1 = (-B + sqrt_delta) * 0.5;
		}
		else
		{
			std::cerr << "No eigen solution:";
			std::cerr << "\tB=" << B <<  "\tC=" << C;
			std::cerr << "\tdelta= " << delta  <<  std::endl;
			return 0;
		}

		return 1;
	}


	// --
	//newtonCoulomb3
	void newtonCoulomb3(vector3d & f, const matrix3d & L, vector3d & v, const vector3d & vFree, double mu)
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
		afbase::matrix6d dphi  ; dphi.fill(0);
		afbase::vector6d phi   ; phi.fill(0);
		afbase::vector6d buf6  ; buf6.fill(0);

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
		dphi[0]  = 1.0;		dphi[3]  = -L[0];		dphi[4]  = -L[1];		dphi[5]  = -L[2];
		dphi[7]  = 1.0;		dphi[9]  = -L[3];		dphi[10] = -L[4];		dphi[11] = -L[5];
		dphi[14] = 1.0 / L[8];
		dphi[32] = 1.0;		dphi[33] = -L[6];		dphi[34] = -L[7];		dphi[35] = -L[8];

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
			for(int i=0; i<3; i++)
				phi(i) = buf3(i);

			// _phi_0(4)=vel_n/L(1,1)
			phi(5) = v(2) / L[8];

			// norm=norm(f_0(2:3)-rho_t * vel_0(2:3))
			//buf3 = f - v * rhoT;
			buf3 = v;
			buf3 *= - rhoT;
			buf3 += f;

			double & buf3_0 = buf3[0];
			double & buf3_1 = buf3[1];
			double norm = sqrt(buf3_0 * buf3_0 + buf3_1 * buf3_1);

			double mu_norm = mu / norm;
			double mu_f2_norm = mu_norm * f(2);

			// _phi_0(5:6)=f_0(2:3)-mu*f_0(1)*(f_0(2:3)-rho_t*vel_0(2:3))/norm
			phi(3) = f(0) - mu_f2_norm * buf3_0; 
			phi(4) = f(1) - mu_f2_norm * buf3_1;

			double phi_2 = phi(2);
			phi(2)	  = phi(5);
			phi(5)	  = phi_2;
			// End fill _phi_0
			//--------------------------------------------------

			//--------------------------------------------------
			// Fill D_phi_0

			// dPhi(5:6,4)=-(mu/norm(f_0(2:3)-rho_t*vel(2:3)))*(f_0(2:3)-rho_t*vel(2:3))
			dphi[23] = -mu_norm * buf3_0;
			dphi[29] = -mu_norm * buf3_1;

			// Compute derivate of V/norm(V)
			double norm3 = norm * norm * norm; 

			// dL=[v2^2 -v1*v2; -v2*v1 v1^2]/(norm(V)^3)
			LBuf_00 =  buf3_1 * buf3_1 / norm3;
			LBuf_01 = -buf3_0 * buf3_1 / norm3;
			LBuf_11 =  buf3_0 * buf3_0 / norm3;

			// dPhi_dQt=rho_t*mu*f_0(1)*dL(f_0(2:3)-rho_t*vel(2:3))
			// dPhi_dFt=Id(2x2)-mu*f_0(1)*dL(f_0(2:3)-rho_t*vel(2:3))      
			// dPhi(5:6,2:3)=dPhi_dQt
			// dPhi(5:6,5:6)=dPhi_dFt

			double  mu_f2 = mu * f(2);
			double  rho_mu_f2 = rhoT * mu_f2;

			{
				dphi[18] = rho_mu_f2 * LBuf_00;
				dphi[19] = dphi[24] = rho_mu_f2 * LBuf_01;

				dphi[25] = rho_mu_f2 * LBuf_11;
				dphi[21] = -mu_f2    * LBuf_00;
				dphi[21] += 1.0;
				dphi[22] = dphi[27] = -mu_f2    * LBuf_01;

				dphi[28] = -mu_f2    * LBuf_11;
				dphi[28] += 1.0;
			}


			// End fill D_phi_0
			//--------------------------------------------------

			//--------------------------------------------------
			// Invert system

			afdyn::solve6FullSpecific(dphi,buf6, phi);

			// End invert system
			//--------------------------------------------------

			// Update forces and accelerations
			for (int i=0; i<3; ++i)
			{
				v(i) -= buf6(i  );
				f(i) -= buf6(i+3);
			}

			++k;
			if (k > 100)
				break;
		}
		while ( phi.norm() > 1e-5);

		// End main loop
		//--------------------------------------------------
	}

	void solve6Full (afbase::matrix6d M, afbase::vector6d & x, const afbase::vector6d & b)
	{
		afbase::matrix3d U_00 (M(0,0), M(0,1), M(0,2), 0.0   , M(1,1), M(1,2), 0.0   , 0.0   , M(2,2));
		afbase::matrix3d U_01 (M(0,3), M(0,4), M(0,5), M(1,3), M(1,4), M(1,5), M(2,3), M(2,4), M(2,5));
		afbase::matrix3d U_11 (M(3,3), M(3,4), M(3,5), 0.0   , M(4,4), M(4,5), 0.0   , 0.0   , M(5,5));
		afbase::matrix3d L_00 (0.0);
		afbase::matrix3d L_10 (0.0);
		afbase::matrix3d L_11 (0.0);

		//pivot Gauss
		L_00[0] = 1;
		L_00[3]= M(1,0) / M(0,0);
		M(1,1) -= L_00[3] * M(0,1);
		M(1,2) -= L_00[3] * M(0,2);
		M(1,3) -= L_00[3] * M(0,3);
		M(1,4) -= L_00[3] * M(0,4);
		M(1,5) -= L_00[3] * M(0,5);
		L_00[6]= M(2,0) / M(0,0);
		M(2,1) -= L_00[6] * M(0,1);
		M(2,2) -= L_00[6] * M(0,2);
		M(2,3) -= L_00[6] * M(0,3);
		M(2,4) -= L_00[6] * M(0,4);
		M(2,5) -= L_00[6] * M(0,5);
		L_10[0]= M(3,0) / M(0,0);
		M(3,1) -= L_10[0] * M(0,1);
		M(3,2) -= L_10[0] * M(0,2);
		M(3,3) -= L_10[0] * M(0,3);
		M(3,4) -= L_10[0] * M(0,4);
		M(3,5) -= L_10[0] * M(0,5);
		L_10[3]= M(4,0) / M(0,0);
		M(4,1) -= L_10[3] * M(0,1);
		M(4,2) -= L_10[3] * M(0,2);
		M(4,3) -= L_10[3] * M(0,3);
		M(4,4) -= L_10[3] * M(0,4);
		M(4,5) -= L_10[3] * M(0,5);
		L_10[6]= M(5,0) / M(0,0);
		M(5,1) -= L_10[6] * M(0,1);
		M(5,2) -= L_10[6] * M(0,2);
		M(5,3) -= L_10[6] * M(0,3);
		M(5,4) -= L_10[6] * M(0,4);
		M(5,5) -= L_10[6] * M(0,5);
		L_00[4] = 1;
		L_00[7]= M(2,1) / M(1,1);
		M(2,2) -= L_00[7] * M(1,2);
		M(2,3) -= L_00[7] * M(1,3);
		M(2,4) -= L_00[7] * M(1,4);
		M(2,5) -= L_00[7] * M(1,5);
		L_10[1]= M(3,1) / M(1,1);
		M(3,2) -= L_10[1] * M(1,2);
		M(3,3) -= L_10[1] * M(1,3);
		M(3,4) -= L_10[1] * M(1,4);
		M(3,5) -= L_10[1] * M(1,5);
		L_10[4]= M(4,1) / M(1,1);
		M(4,2) -= L_10[4] * M(1,2);
		M(4,3) -= L_10[4] * M(1,3);
		M(4,4) -= L_10[4] * M(1,4);
		M(4,5) -= L_10[4] * M(1,5);
		L_10[7] = M(5,1) / M(1,1);
		M(5,2) -= L_10[7] * M(1,2);
		M(5,3) -= L_10[7] * M(1,3);
		M(5,4) -= L_10[7] * M(1,4);
		M(5,5) -= L_10[7] * M(1,5);
		L_00[8] = 1;
		L_10[2]= M(3,2) / M(2,2);
		M(3,3) -= L_10[2] * M(2,3);
		M(3,4) -= L_10[2] * M(2,4);
		M(3,5) -= L_10[2] * M(2,5);
		L_10[5] = M(4,2) / M(2,2);
		M(4,3) -= L_10[5] * M(2,3);
		M(4,4) -= L_10[5] * M(2,4);
		M(4,5) -= L_10[5] * M(2,5);
		L_10[8] = M(5,2) / M(2,2);
		M(5,3) -= L_10[8] * M(2,3);
		M(5,4) -= L_10[8] * M(2,4);
		M(5,5) -= L_10[8] * M(2,5);
		L_11[0] = 1;
		L_11[3] = M(4,3) / M(3,3);
		M(4,4) -= L_11[3] * M(3,4);
		M(4,5) -= L_11[3] * M(3,5);
		L_11[6] = M(5,3) / M(3,3);
		M(5,4) -= L_11[6] * M(3,4);
		M(5,5) -= L_11[6] * M(3,5);
		L_11[4] = 1;
		L_11[7] = M(5,4) / M(4,4);
		M(5,5) -= L_11[4] * M(4,5);

		U_00[4] -= L_00[3] * U_00[1];
		U_00[5] -= L_00[3] * U_00[2];
		U_01[3] -= L_00[3] * U_01[0];
		U_01[4] -= L_00[3] * U_01[1];
		U_01[5] -= L_00[3] * U_01[2];

		U_00[8] -= L_00[6] * U_00[2] + L_00[7] * U_00[5];
		U_01[6] -= L_00[6] * U_01[0] + L_00[7] * U_01[3];
		U_01[7] -= L_00[6] * U_01[1] + L_00[7] * U_01[4];
		U_01[8] -= L_00[6] * U_01[2] + L_00[7] * U_01[5];

		U_11[0] -= L_10[0] * U_01[0] + L_10[1] * U_01[3] + L_10[2] * U_01[6];
		U_11[1] -= L_10[0] * U_01[1] + L_10[1] * U_01[4] + L_10[2] * U_01[7];
		U_11[2] -= L_10[0] * U_01[2] + L_10[1] * U_01[5] + L_10[2] * U_01[8];

		U_11[4] -= L_10[3] * U_01[1] + L_10[4] * U_01[4] + L_10[5] * U_01[7] + L_11[3] * U_11[1];
		U_11[5] -= L_10[3] * U_01[2] + L_10[4] * U_01[5] + L_10[5] * U_01[8] + L_11[3] * U_11[2];

		U_11[8] -= L_10[6] * U_01[2] + L_10[7] * U_01[5] + L_10[8] * U_01[8] + L_11[6] * U_11[2] + L_11[7] * U_11[5];



		// Resolution of mat*x=b
		x = b;	// Update right hand side
		x[1] -= L_00[3] * x[0]; 
		x[2] -= L_00[6] * x[0] + L_00[7] * x[1]; 
		x[3] -= L_10[0] * x[0] + L_10[1] * x[1] + L_10[2] * x[2]; 
		x[4] -= L_10[3] * x[0] + L_10[4] * x[1] + L_10[5] * x[2] + L_11[3] * x[3]; 
		x[5] -= L_10[6] * x[0] + L_10[7] * x[1] + L_10[8] * x[2] + L_11[6] * x[3] + L_11[7] * x[4];  

		// Back substitution
		x[5] /= U_11[8];

		x[4] -= U_11[5] * x(5);
		x[4] /= U_11[4];

		x[3] -= U_11[2] * x(5) + U_11[1] * x(4);
		x[3] /= U_11[0];

		x[2] -= U_01[8] * x(5) + U_01[7] * x(4) + U_01[6] * x(3);
		x[2] /= U_00[8];

		x[1] -= U_01[5] * x(5) + U_01[4] * x(4) + U_01[3] * x(3) + U_00[5] * x(2);
		x[1] /= U_00[4];

		x[0] -= U_01[2] * x(5) + U_01[1] * x(4) + U_01[0] * x(3) + U_00[2] * x(2) + U_00[1] * x(1);
		x[0] /= U_00[0];
	}

	void solve6FullSpecific (
		const afbase::matrix6d & M,
		afbase::vector6d & x,
		const afbase::vector6d & b
	)
	{
		//  check that M has the form
		//          1            0            0            X            X            X
		//          0            1            0            X            X            X
		//          0            0            X            0            0            0
		//     		X            X            0            X            X            X
		//    		X            X            0            X            X            X
		//          0            0            1            X            X            X
		assert ( 
		(M(0,0) == 1) && (M(0,1) == 0) && (M(0,2) == 0) &&
		(M(1,0) == 0) && (M(1,1) == 1) && (M(1,2) == 0) &&
		(M(2,0) == 0) && (M(2,1) == 0) && (M(2,3) == 0) && (M(2,4) == 0) && (M(2,5) == 0) && 
		(M(3,2) == 0) && 
		(M(4,2) == 0) && 
		(M(5,0) == 0) && (M(5,1) == 0) && (M(5,2) == 1) 
		);

		const afbase::matrix3d U_01 (
			M(0,3), M(0,4), M(0,5),	
			M(1,3), M(1,4), M(1,5),	
			0	  , 	 0, 	 0
		);

		afbase::matrix3d U_11 (
			M(3,3), M(3,4), M(3,5), 
			0.0   , M(4,4), M(4,5), 
			0.0   , 0.0   , M(5,5)
		);

		const afbase::matrix3d L_10 (
			M(3,0), M(3,1), 0,    
			M(4,0), M(4,1), 0,  
			0     ,      0, 1./M(2,2)
		);
		
		afbase::matrix3d M_11 (
			M(3,3), M(3,4), M(3,5),    
			M(4,3), M(4,4), M(4,5),  
			M(5,3), M(5,4), M(5,5)
		);

		//pivot Gauss
		M_11(0,0) -= L_10[0] * M(0,3);
		M_11(0,1) -= L_10[0] * M(0,4);
		M_11(0,2) -= L_10[0] * M(0,5);

		M_11(1,0) -= L_10[3] * M(0,3);
		M_11(1,1) -= L_10[3] * M(0,4);
		M_11(1,2) -= L_10[3] * M(0,5);

		M_11(0,0) -= L_10[1] * M(1,3);
		M_11(0,1) -= L_10[1] * M(1,4);
		M_11(0,2) -= L_10[1] * M(1,5);

		M_11(1,0) -= L_10[4] * M(1,3);
		M_11(1,1) -= L_10[4] * M(1,4);
		M_11(1,2) -= L_10[4] * M(1,5);

		M_11(2,0) -= L_10[8] * M(2,3);
		M_11(2,1) -= L_10[8] * M(2,4);
		M_11(2,2) -= L_10[8] * M(2,5);

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
		x = b;	// Update right hand side

		x[3] -= L_10[0] * x[0] + L_10[1] * x[1]; 
		x[4] -= L_10[3] * x[0] + L_10[4] * x[1] + L_11__3 * x[3]; 
		x[5] -= L_10[8] * x[2] + L_11__6 * x[3] + L_11__7 * x[4];  

		// Back substitution
		x[5] /= U_11[8];

		x[4] -= U_11[5] * x(5);
		x[4] /= U_11[4];

		x[3] -= U_11[2] * x(5) + U_11[1] * x(4);
		x[3] /= U_11[0];

		x[2] /= M(2,2);
		x[1] -= U_01[5] * x(5) + U_01[4] * x(4) + U_01[3] * x(3);
		x[0] -= U_01[2] * x(5) + U_01[1] * x(4) + U_01[0] * x(3);
	}
	
		//gaussSeidel
	void gaussSeidel(
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

				for (int j=0; j<numInteraction; ++j)
					if ( interactionVector[j]->activeCollision_ == true && i != j)
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
						newtonCoulomb3(fc_i, lambdaMatrix_i[i], velt, vBuf, interactionVector[i]->frictionCoeff_);
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
