/**
 * File              : ../UR5Kinematics/ur5_kinematics_mex.h
 * Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
 * Date              : 07.10.2018
 * Last Modified Date: 10.12.2018
 * Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
 */

#include <ur_kinematics/ur_kin.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cstring>

using namespace std;
using namespace Eigen;

typedef Map<Matrix<double, 3, Dynamic> > Centers;
typedef Map<Matrix<double, 4, 4> > HomTransMat;

typedef Map<Matrix<double, 6, 6> > JacobianMat;

typedef Map<Matrix<double, 8, 6> > IKSolutionMat;

const size_t num_entries = 16;

inline Matrix4d create_transformation_matrix(const double tx, 
                                             const double ty,
                                             const double tz,
                                             const double ax,
                                             const double ay,
                                             const double az)
{
const Affine3d rx = Affine3d(AngleAxisd(ax, Vector3d(1, 0, 0)));
const Affine3d ry = Affine3d(AngleAxisd(ay, Vector3d(0, 1, 0)));
const Affine3d rz = Affine3d(AngleAxisd(az, Vector3d(0, 0, 1)));
const Affine3d R = rz * ry * rx;
const Affine3d T(Translation3d(Vector3d(tx,ty,tz)));
Matrix4d H = (T * R).matrix();
return H;
}

class UR5Kinematics
{
    protected:

        Vector3d v1;
        Vector3d v2;
        Vector3d v3;
        Vector3d v4;
        Vector3d v5;
        Vector3d v6;

        const double start_val1 = 0;
        const double start_val2 = 0;
        const double start_val3 = 0;
        const double start_val4 = -0.3;
        const double start_val5 = -0.3;
        const double start_val6 = -0.40;

        const double start_val7 = 0.2;
        const double start_val8 = 0.0;

        const double end_val1 = 1;
        const double end_val2 = 1.0;
        const double end_val3 = 0.8;
        const double end_val4 = 1;
        const double end_val5 = 1;
        const double end_val6 = 0.8;

        const double end_val7 = 0.9;
        const double end_val8 = 1.0;

        Matrix4d PCM1; 
        Matrix4d PCM2; 
        Matrix4d PCM3; 
        Matrix4d PCM4; 
        Matrix4d PCM5; 
        Matrix4d PCM6; 

        Matrix4d PCM7; 
        Matrix4d PCM8; 

        inline void get_ball_centers_per_link(const size_t N, const Vector3d direction, const double start_val, const double end_val, const Matrix4d transform, Centers& output) const
        {
            ArrayXd interp = ArrayXd::LinSpaced(N, start_val, end_val);
            Matrix<double, 4, Dynamic> l(4, N);
            for (size_t i=0; i<N; i++)
            {
                Vector4d tmp;
                Matrix<double, 1, 1> one = Matrix<double, 1, 1>::Ones();
                tmp << direction * interp[i], one;
                l.col(i) = tmp;
            }

            Matrix<double, 4, Dynamic> out(4, N);
            out = transform * l;
            output = out.topRows(3);
        }

    public:
        const double offset_1 = 0.135;
        const double offset_2 = 0.425;
        const double offset_3 = 0.39225;
        const double offset_5 = 0.09465;
        const double offset_6 = 0.0823;
        const double offset_7 = 0.015;

		const double offset_tool = offset_6;

		const double ball_radius_large = 0.07;
		const double ball_radius_medium = 0.06;
		const double ball_radius_medium_small = 0.045;
		const double ball_radius_small = 0.04;
		const double ball_radius_vvsmall = 0.018;
		const double ball_radius_vsmall = 0.025;



        // const size_t N1 = 3;
        // const size_t N2 = 5;
        // const size_t N3 = 4;
        // const size_t N4 = 4;
        // const size_t N5 = 3;
        // const size_t N6 = 3;
        static const size_t N1 = 3;
        static const size_t N2 = 5;
        static const size_t N3 = 4;
        static const size_t N4 = 4;
        static const size_t N5 = 3;
        static const size_t N6 = 3;


		// Tool stuff
		static const size_t N7 = 7;
		static const size_t N8 = 4;
		const double tool_long_length = 0.170;
		const double tool_short_length = 0.070;

		static const size_t num_balls = N1+N2+N3+N4+N5+N6+N7+N8;
		double ball_sizes[num_balls];
		double ball_link_memberships[num_balls];
		// Matrix<double, 1, Dynamic> ball_sizes;
		// Matrix<double, 1, num_balls> ball_sizes;
	

        inline void get_ball_centers(const HomTransMat& M1, const HomTransMat& M2, const HomTransMat& M3, const HomTransMat& M4, const HomTransMat& M5, const HomTransMat& M6, Centers& B1, Centers& B2, Centers& B3, Centers& B4, Centers& B5, Centers& B6, Centers& B7, Centers& B8) const
        {
            // Use forward kinematics to figure out ball centers of link 1
            const Matrix4d T1 = M1 * PCM1;
            const Matrix4d T2 = M2 * PCM2;
            const Matrix4d T3 = M3 * PCM3;
            const Matrix4d T4 = M4 * PCM4;
            const Matrix4d T5 = M5 * PCM5;
            const Matrix4d T6 = M6 * PCM6;

            get_ball_centers_per_link(N1, v1, start_val1, end_val1, T1, B1);
            get_ball_centers_per_link(N2, v2, start_val2, end_val2, T2, B2);
            get_ball_centers_per_link(N3, v3, start_val3, end_val3, T3, B3);
            get_ball_centers_per_link(N4, v4, start_val4, end_val4, T4, B4);
            get_ball_centers_per_link(N5, v5, start_val5, end_val5, T5, B5);
            get_ball_centers_per_link(N6, v6, start_val6, end_val6, T6, B6);

			// Also get tool ball centers
			const Matrix4d T7 = T6 * PCM7; 	
			const Matrix4d T8 = T6 * PCM8; 	
			const Vector3d long_dir(0, tool_long_length, 0);
			const Vector3d short_dir(-tool_short_length, 0, 0);
            get_ball_centers_per_link(N7, long_dir, start_val7, end_val7, T7, B7);
            get_ball_centers_per_link(N8, short_dir, start_val8, end_val8, T8, B8);
        }

        inline void get_forward_kinematics(const double* joint_angles, HomTransMat& T1, HomTransMat& T2, HomTransMat& T3, HomTransMat& T4, HomTransMat& T5, HomTransMat& T6, HomTransMat& Tend) const
        {
            ur_kinematics::forward_all(joint_angles, T1.data(), T2.data(), T3.data(), T4.data(), T5.data(), T6.data());
            // Use eigen to transpose the matrices
            T1.transposeInPlace();
            T2.transposeInPlace();
            T3.transposeInPlace();
            T4.transposeInPlace();
            T5.transposeInPlace();
            T6.transposeInPlace();
			Tend = T6;
        }

		inline size_t get_inverse_kinematics(const double* T_ee_ptr, const double q6_des, double* ik_sol_ptr) const
		{
			return ur_kinematics::inverse(T_ee_ptr, ik_sol_ptr, q6_des);
		}

        inline void get_geometric_jacobian(const HomTransMat& T0,
                                           const HomTransMat& T1,
                                           const HomTransMat& T2,
                                           const HomTransMat& T3,
                                           const HomTransMat& T4,
                                           const HomTransMat& T5,
                                           const HomTransMat& T6,
                                           JacobianMat& J)
        {
            const Vector3d p0 = T0.block<3, 1>(0, 3);
            const Vector3d p1 = T1.block<3, 1>(0, 3);
            const Vector3d p2 = T2.block<3, 1>(0, 3);
            const Vector3d p3 = T3.block<3, 1>(0, 3);
            const Vector3d p4 = T4.block<3, 1>(0, 3);
            const Vector3d p5 = T5.block<3, 1>(0, 3);
            const Vector3d p =  T6.block<3, 1>(0, 3);

            const Vector3d z0 = T0.block<3, 1>(0, 2);
            const Vector3d z1 = T1.block<3, 1>(0, 2);
            const Vector3d z2 = T2.block<3, 1>(0, 2);
            const Vector3d z3 = T3.block<3, 1>(0, 2);
            const Vector3d z4 = T4.block<3, 1>(0, 2);
            const Vector3d z5 = T5.block<3, 1>(0, 2);

            //Position Jacobian
            J.block<3, 1>(0, 0) = z0.cross(p-p0);
            J.block<3, 1>(0, 1) = z1.cross(p-p1);
            J.block<3, 1>(0, 2) = z2.cross(p-p2);
            J.block<3, 1>(0, 3) = z3.cross(p-p3);
            J.block<3, 1>(0, 4) = z4.cross(p-p4);
            J.block<3, 1>(0, 5) = z5.cross(p-p5);
            
            //Orientation Jacobian
            J.block<3, 1>(3, 0) = z0;
            J.block<3, 1>(3, 1) = z1;
            J.block<3, 1>(3, 2) = z2;
            J.block<3, 1>(3, 3) = z3;
            J.block<3, 1>(3, 4) = z4;
            J.block<3, 1>(3, 5) = z5;
        }

        void compute_rho(const double* q1, const double* q2, double* max_dist)
        {
            double t1_q1[16];
            double t2_q1[16];
            double t3_q1[16];
            double t4_q1[16];
            double t5_q1[16];
            double t6_q1[16];
            double tend_q1[16];

            double t1_q2[16];
            double t2_q2[16];
            double t3_q2[16];
            double t4_q2[16];
            double t5_q2[16];
            double t6_q2[16];
            double tend_q2[16];

            HomTransMat M1_q1(t1_q1, 4, 4);
            HomTransMat M2_q1(t2_q1, 4, 4);
            HomTransMat M3_q1(t3_q1, 4, 4);
            HomTransMat M4_q1(t4_q1, 4, 4);
            HomTransMat M5_q1(t5_q1, 4, 4);
            HomTransMat M6_q1(t6_q1, 4, 4);
            HomTransMat Mend_q1(tend_q1, 4, 4);

            HomTransMat M1_q2(t1_q2, 4, 4);
            HomTransMat M2_q2(t2_q2, 4, 4);
            HomTransMat M3_q2(t3_q2, 4, 4);
            HomTransMat M4_q2(t4_q2, 4, 4);
            HomTransMat M5_q2(t5_q2, 4, 4);
            HomTransMat M6_q2(t6_q2, 4, 4);
            HomTransMat Mend_q2(tend_q2, 4, 4);

            const size_t b1_sz = 3*N1;
            const size_t b2_sz = 3*N2;
            const size_t b3_sz = 3*N3;
            const size_t b4_sz = 3*N4;
            const size_t b5_sz = 3*N5;
            const size_t b6_sz = 3*N6;
            const size_t b7_sz = 3*N7;
            const size_t b8_sz = 3*N8;

            double b1_q1[b1_sz];
            double b2_q1[b2_sz];
            double b3_q1[b3_sz];
            double b4_q1[b4_sz];
            double b5_q1[b5_sz];
            double b6_q1[b6_sz];
            double b7_q1[b7_sz];
            double b8_q1[b8_sz];

            Centers B1_q1(b1_q1, 3, N1);
            Centers B2_q1(b2_q1, 3, N2);
            Centers B3_q1(b3_q1, 3, N3);
            Centers B4_q1(b4_q1, 3, N4);
            Centers B5_q1(b5_q1, 3, N5);
            Centers B6_q1(b6_q1, 3, N6);
            Centers B7_q1(b7_q1, 3, N7);
            Centers B8_q1(b8_q1, 3, N8);

            double b1_q2[b1_sz];
            double b2_q2[b2_sz];
            double b3_q2[b3_sz];
            double b4_q2[b4_sz];
            double b5_q2[b5_sz];
            double b6_q2[b6_sz];
            double b7_q2[b7_sz];
            double b8_q2[b8_sz];

            Centers B1_q2(b1_q2, 3, N1);
            Centers B2_q2(b2_q2, 3, N2);
            Centers B3_q2(b3_q2, 3, N3);
            Centers B4_q2(b4_q2, 3, N4);
            Centers B5_q2(b5_q2, 3, N5);
            Centers B6_q2(b6_q2, 3, N6);
            Centers B7_q2(b7_q2, 3, N7);
            Centers B8_q2(b8_q2, 3, N8);

            get_forward_kinematics(q1, M1_q1, M2_q1, M3_q1, M4_q1, M5_q1, M6_q1, Mend_q1);
            get_forward_kinematics(q2, M1_q2, M2_q2, M3_q2, M4_q2, M5_q2, M6_q2, Mend_q2);

            get_ball_centers(M1_q1, M2_q1, M3_q1, M4_q1, M5_q1, M6_q1, B1_q1, B2_q1, B3_q1, B4_q1, B5_q1, B6_q1, B7_q1, B8_q1); 
            get_ball_centers(M1_q2, M2_q2, M3_q2, M4_q2, M5_q2, M6_q2, B1_q2, B2_q2, B3_q2, B4_q2, B5_q2, B6_q2, B7_q2, B8_q2); 

			const size_t num_mats = 8;    
            const Centers* Bmats_q1[num_mats] = {&B1_q1, &B2_q1, &B3_q1, &B4_q1, &B5_q1, &B6_q1, &B7_q1, &B8_q1};
            const Centers* Bmats_q2[num_mats] = {&B1_q2, &B2_q2, &B3_q2, &B4_q2, &B5_q2, &B6_q2, &B7_q2, &B8_q2};

            *max_dist = 0;
            for (size_t m = 0; m < num_mats; m++)
            {
                Centers bq1 = *(Bmats_q1[m]);
                Centers bq2 = *(Bmats_q2[m]);
                Vector3d pos_diff = bq1.rightCols(1) - bq2.rightCols(1);// get the right most column
                const double curr_dist = pos_diff.norm(); 
                // std::cout << "m : " << m << ", curr_dist: " << curr_dist << std::endl;
                if (*max_dist < curr_dist)
                {
                    *max_dist = curr_dist;
                }
            }
        }

        void get_enclosing_cylinder_radius_list(const Centers& ball_centers1,
												const Centers& ball_centers2,
												const Centers& ball_centers3,
												const Centers& ball_centers4,
												const Centers& ball_centers5,
												const Centers& ball_centers6,
												const Centers& ball_centers7,
												const Centers& ball_centers8,
												const HomTransMat& T1,
												const HomTransMat& T2,
												const HomTransMat& T3,
												const HomTransMat& T4,
												const HomTransMat& T5,
												const HomTransMat& T6,
												double* radius_list) const
        {
            const size_t num_ball_center_mats = 8;
            Matrix<double, 3, num_ball_center_mats> ball_center_link_start;
            Matrix<double, 3, num_ball_center_mats> ball_center_link_end;
            ball_center_link_start << ball_centers1.leftCols(1), 
                                      ball_centers2.leftCols(1), 
                                      ball_centers3.leftCols(1), 
                                      ball_centers4.leftCols(1), 
                                      ball_centers5.leftCols(1), 
                                      ball_centers6.leftCols(1), 
                                      ball_centers7.leftCols(1); 
                                      ball_centers8.leftCols(1); 

            ball_center_link_end << ball_centers1.rightCols(1),
                                    ball_centers2.rightCols(1),
                                    ball_centers3.rightCols(1),
                                    ball_centers4.rightCols(1),           
                                    ball_centers5.rightCols(1),
                                    ball_centers6.rightCols(1),
                                    ball_centers7.rightCols(1),
                                    ball_centers8.rightCols(1);
			const size_t num_joints = 6;
            const HomTransMat* Tmats[num_joints] = {&T1, &T2, &T3, &T4, &T5, &T6};
            Matrix4d Tmat = Matrix4d::Identity();
            for (size_t mk = 0; mk < num_joints; mk++)
            {
                const Vector3d bc_start = ball_center_link_start.col(mk);
                Vector4d tmp;
                tmp << 0, 0, 1, 0;
                const Vector4d dir_vec_tmp = Tmat.col(2); 
                Vector3d dir_vec = dir_vec_tmp.topRows(3);
                dir_vec.normalize();

                Tmat = *(Tmats[mk]);

                radius_list[mk] = 0;
				//First we find max distances using balls along the joints
                for (size_t wk = mk; wk < num_ball_center_mats; wk++)
                {
					//Note: wk goes to all balls listed in ball center links.
					// i.e. NOTE THAT num_ball_center_mats >= num_joints
					//Now proceed to tool balls
                    const Vector3d bc_curr = ball_center_link_end.col(wk);
                    const Vector3d tmp_vec = bc_curr - bc_start;
                    Vector3d tmp_vec_final = tmp_vec - (tmp_vec.dot(dir_vec))*dir_vec;
                    double curr_dist_from_axis = tmp_vec_final.norm() + 0.08;
                    if (radius_list[mk] < curr_dist_from_axis)
                    {
                        radius_list[mk] = curr_dist_from_axis;
                    }
                }
				// for (size_t tk = 0; tk < ball_centers7.cols(); tk++)
				// {
				// 	const Vector3d bc_curr = ball_centers7.col
				// }
				
            }
        }
		inline void get_ball_sizes(double* ball_size_array)
		{
			std::memcpy(ball_size_array, ball_sizes, sizeof(double) * num_balls);
		}
		inline void get_ball_link_memberships(double* ball_link_memberships_array)
		{
			std::memcpy(ball_link_memberships_array, ball_link_memberships, sizeof(double) * num_balls);
		}

        UR5Kinematics()
        {
            v1 = Vector3d(0, offset_1, 0);
            v2 = Vector3d(0, 0, offset_2);
            v3 = Vector3d(0, 0, offset_3);
            v4 = Vector3d(0, offset_5, 0);
            v5 = Vector3d(0, 0, offset_5);
            v6 = Vector3d(0, offset_6, 0);
            PCM1 = create_transformation_matrix(0,0,0,-M_PI/2,0,0) * 
                                  create_transformation_matrix(0,0,0,0,0,M_PI);

            PCM2 = create_transformation_matrix(0,0,0, M_PI/2,0,0) * 
                                  create_transformation_matrix(0,0,0,0,-M_PI/2,0) *
                                  create_transformation_matrix(0,offset_1,-offset_2,0,0,0);

            PCM3 = create_transformation_matrix(0,0,0, M_PI/2,0,0) * 
                                  create_transformation_matrix(0,0,0,0,-M_PI/2,0) *
                                  create_transformation_matrix(0,offset_7,-offset_3,0,0,0);

            PCM4 = create_transformation_matrix(0,-offset_5,0,0,0,0);

            PCM5 = create_transformation_matrix(0,0,0, M_PI/2,0,0) * 
                                  create_transformation_matrix(0,0,-offset_5,0,0,0);

            PCM6 = create_transformation_matrix(0,0,0, M_PI/2,0,0) * 
                                  create_transformation_matrix(0,-offset_6,0,0,0,0);

			// These are for the tools
            PCM7 = create_transformation_matrix(0,offset_tool,0,0,0,0);
            PCM8 = create_transformation_matrix(0,offset_tool+tool_long_length,0,0,0,0);
			
			// Set the bounding radius of each ball
			Map<Matrix<double, 1, num_balls> > ball_sizes_eigen(ball_sizes, 1, num_balls);
			
			ball_sizes_eigen << ball_radius_large		 * Matrix<double, 1, N1>::Ones(),
							 	ball_radius_medium		 * Matrix<double, 1, N2>::Ones(),
							 	ball_radius_medium		 * Matrix<double, 1, N3>::Ones(),
							 	ball_radius_small		 * Matrix<double, 1, N4>::Ones(),
							 	ball_radius_medium_small * Matrix<double, 1, N5>::Ones(),
							 	ball_radius_small        * Matrix<double, 1, N6>::Ones(),
							 	ball_radius_vvsmall      * Matrix<double, 1, N7>::Ones(),
							 	ball_radius_vsmall       * Matrix<double, 1, N8>::Ones();
			// Set the link membership of each ball
			Map<Matrix<double, 1, num_balls> > ball_link_memberships__eigen(ball_link_memberships, 1, num_balls);
			ball_link_memberships__eigen << 1 * Matrix<double, 1, N1>::Ones(),
							 	           2 * Matrix<double, 1, N2>::Ones(),
							 	           3 * Matrix<double, 1, N3>::Ones(),
							 	           4 * Matrix<double, 1, N4>::Ones(),
							 	           5 * Matrix<double, 1, N5>::Ones(),
							 	           6 * Matrix<double, 1, N6>::Ones(),
							 	           7 * Matrix<double, 1, N7>::Ones(),
							 	           8 * Matrix<double, 1, N8>::Ones();

            std::cout << "\nUR5Kinematics initialized\n" << std::endl;
        };
        ~UR5Kinematics()
        {
            std::cout << "\nUR5Kinematics destroyed\n" << std::endl;
        }
};
