/**
 * File              : ../UR5Kinematics/ur5_kinematics_mex.cpp
 * Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
 * Date              : 06.10.2018
 * Last Modified Date: 10.12.2018
 * Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
 */
#include "mex.h"
#include "matrix.h"
#include "class_handle.hpp"
#include "ur5_kinematics_mex.h"

#include <iostream>

#define GET_SIZE(x) (sizeof(x)/sizeof(x[0]))

const size_t REQUEST_FORWARD_KINEMATICS_CMD_ID = 0;
const size_t REQUEST_INVERSE_KINEMATICS_CMD_ID = 1;
const size_t REQUEST_COLLISION_BALLS_CMD_ID = 2;
const size_t REQUEST_ENCLOSING_CYLINDERS_CMD_ID = 3;
const size_t REQUEST_COMPUTE_RHO_CMD_ID = 4;
// const size_t DUMMY_ID = 5;

/* IF YOU ADD MORE ENSURE THAT CORRESPONDING FIELD NAMES ARE WRITTEN TOO */
/* IF YOU ADD MORE ENSURE THAT CORRESPONDING FIELD NAMES ARE WRITTEN TOO */
/* IF YOU ADD MORE ENSURE THAT CORRESPONDING FIELD NAMES ARE WRITTEN TOO */
const size_t IDX_TRANSFORM_MATRICES = 0;
const size_t IDX_BALL_CENTERS = 1;
const size_t IDX_BALL_RADII = 2;
const size_t IDX_ENCLOSING_CYLINDER_RADII = 3;
const size_t IDX_BALL_LINK_MEMBERSHIPS = 4;
const char* result_type_1_field_names[] = {"transform_matrices",
                                           "balls_centers",
                                           "ball_radii",
                                           "enclosing_cylinder_radii",
                                           "ball_link_memberships"}; 
const size_t result_type_1_num_fields = GET_SIZE(result_type_1_field_names);

const size_t IDX_T1 = 0;
const size_t IDX_T2 = 1;
const size_t IDX_T3 = 2;
const size_t IDX_T4 = 3;
const size_t IDX_T5 = 4;
const size_t IDX_T6 = 5;
const size_t IDX_TEND = 6;
const char* transforms_field_names[] = {"T1", "T2", "T3", "T4", "T5", "T6", "Tend"};
const size_t transforms_num_fields = GET_SIZE(transforms_field_names);

const char* ball_centers_field_names[] = {"B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8"};
const size_t ball_centers_num_fields = GET_SIZE(ball_centers_field_names);

UR5Kinematics ur_kin;

inline void allocate_memory_for_matrix(double*& ptr, mxArray*& arr, const size_t rows, const size_t cols)
{
    // Allocate MATLAB memory for matrix 
    arr = mxCreateDoubleMatrix(rows, cols, mxREAL);
    // Extract pointer to data
    ptr = mxGetDoubles(arr);
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
    if (nrhs < 1)
    {
        mexErrMsgTxt("First input should be a command id of type uint64.");
    }
    // Get the command id
    const size_t cmd_id = (size_t) mxGetScalar(prhs[0]);

    if ( REQUEST_FORWARD_KINEMATICS_CMD_ID == cmd_id || REQUEST_COLLISION_BALLS_CMD_ID == cmd_id || REQUEST_ENCLOSING_CYLINDERS_CMD_ID == cmd_id)
    {
        // Get joint angles
        const mxDouble *const joint_angles = mxGetDoubles(prhs[1]);

        plhs[0] = mxCreateStructMatrix(1, 1, result_type_1_num_fields, result_type_1_field_names);

        double* H1 = NULL; 
        double* H2 = NULL;
        double* H3 = NULL;
        double* H4 = NULL;
        double* H5 = NULL;
        double* H6 = NULL;
        double* Hend = NULL;

        mxArray* T1 = NULL; 
        mxArray* T2 = NULL;
        mxArray* T3 = NULL;
        mxArray* T4 = NULL;
        mxArray* T5 = NULL;
        mxArray* T6 = NULL;
        mxArray* Tend = NULL;

        allocate_memory_for_matrix(H1, T1, 4, 4);
        allocate_memory_for_matrix(H2, T2, 4, 4);
        allocate_memory_for_matrix(H3, T3, 4, 4);
        allocate_memory_for_matrix(H4, T4, 4, 4);
        allocate_memory_for_matrix(H5, T5, 4, 4);
        allocate_memory_for_matrix(H6, T6, 4, 4);
        allocate_memory_for_matrix(Hend, Tend, 4, 4);

        HomTransMat M1(H1, 4, 4);
        HomTransMat M2(H2, 4, 4);
        HomTransMat M3(H3, 4, 4);
        HomTransMat M4(H4, 4, 4);
        HomTransMat M5(H5, 4, 4);
        HomTransMat M6(H6, 4, 4);
        HomTransMat Mend(Hend, 4, 4);

        // Compute forward kinematics and write transformation matrix directly
        // into MATLAB memory
        ur_kin.get_forward_kinematics(joint_angles, M1, M2, M3, M4, M5, M6, Mend);

        // Assign values
        mxArray* transforms = mxCreateStructMatrix(1, 1, transforms_num_fields, transforms_field_names);
        mxSetFieldByNumber(transforms, 0, IDX_T1, T1);
        mxSetFieldByNumber(transforms, 0, IDX_T2, T2);
        mxSetFieldByNumber(transforms, 0, IDX_T3, T3);
        mxSetFieldByNumber(transforms, 0, IDX_T4, T4);
        mxSetFieldByNumber(transforms, 0, IDX_T5, T5);
        mxSetFieldByNumber(transforms, 0, IDX_T6, T6);
        mxSetFieldByNumber(transforms, 0, IDX_TEND, Tend);
        
        mxSetFieldByNumber(plhs[0], 0, IDX_TRANSFORM_MATRICES, transforms);
        if (REQUEST_FORWARD_KINEMATICS_CMD_ID == cmd_id) return;

        // We go ahead if collision balls were requested
        mxArray* B1 = NULL; 
        mxArray* B2 = NULL;
        mxArray* B3 = NULL;
        mxArray* B4 = NULL;
        mxArray* B5 = NULL;
        mxArray* B6 = NULL;
        mxArray* B7 = NULL;
        mxArray* B8 = NULL;
        
        double* B1_ptr = NULL;
        double* B2_ptr = NULL;
        double* B3_ptr = NULL;
        double* B4_ptr = NULL;
        double* B5_ptr = NULL;
        double* B6_ptr = NULL;
        double* B7_ptr = NULL;
        double* B8_ptr = NULL;

        // Allocate MATLAB memory for ball centers
        allocate_memory_for_matrix(B1_ptr, B1, 3, ur_kin.N1);
        allocate_memory_for_matrix(B2_ptr, B2, 3, ur_kin.N2);
        allocate_memory_for_matrix(B3_ptr, B3, 3, ur_kin.N3);
        allocate_memory_for_matrix(B4_ptr, B4, 3, ur_kin.N4);
        allocate_memory_for_matrix(B5_ptr, B5, 3, ur_kin.N5);
        allocate_memory_for_matrix(B6_ptr, B6, 3, ur_kin.N6);
        allocate_memory_for_matrix(B7_ptr, B7, 3, ur_kin.N7);
        allocate_memory_for_matrix(B8_ptr, B8, 3, ur_kin.N8);

        // Allocate MATLAB memory for ball radii 
        mxArray* B_radii = NULL;
        double* B_radii_ptr = NULL;
        allocate_memory_for_matrix(B_radii_ptr, B_radii, 1, ur_kin.num_balls);

        // Allocate MATLAB memory for ball link membership 
        mxArray* B_link_memberships = NULL;
        double* B_link_memberships_ptr = NULL;
        allocate_memory_for_matrix(B_link_memberships_ptr, B_link_memberships, 1, ur_kin.num_balls);
        

        // Assign pointer to memory for Eigen
        // B = ball_centers
        Centers B1_c(B1_ptr, 3, ur_kin.N1);
        Centers B2_c(B2_ptr, 3, ur_kin.N2);
        Centers B3_c(B3_ptr, 3, ur_kin.N3);
        Centers B4_c(B4_ptr, 3, ur_kin.N4);
        Centers B5_c(B5_ptr, 3, ur_kin.N5);
        Centers B6_c(B6_ptr, 3, ur_kin.N6);
		
		// Tool ball centers
        Centers B7_c(B7_ptr, 3, ur_kin.N7);
        Centers B8_c(B8_ptr, 3, ur_kin.N8);

        ur_kin.get_ball_centers(M1, M2, M3, M4, M5, M6, B1_c, B2_c, B3_c, B4_c, B5_c, B6_c, B7_c, B8_c); 
		ur_kin.get_ball_sizes(B_radii_ptr);
		ur_kin.get_ball_link_memberships(B_link_memberships_ptr);

        // Assign values
        mxArray* ball_centers = mxCreateStructMatrix(1, 1, ball_centers_num_fields, ball_centers_field_names);
        mxSetFieldByNumber(ball_centers, 0, 0, B1);
        mxSetFieldByNumber(ball_centers, 0, 1, B2);
        mxSetFieldByNumber(ball_centers, 0, 2, B3);
        mxSetFieldByNumber(ball_centers, 0, 3, B4);
        mxSetFieldByNumber(ball_centers, 0, 4, B5);
        mxSetFieldByNumber(ball_centers, 0, 5, B6);
        mxSetFieldByNumber(ball_centers, 0, 6, B7);
        mxSetFieldByNumber(ball_centers, 0, 7, B8);

        mxSetFieldByNumber(plhs[0], 0, IDX_BALL_CENTERS, ball_centers);
        mxSetFieldByNumber(plhs[0], 0, IDX_BALL_RADII, B_radii);
        mxSetFieldByNumber(plhs[0], 0, IDX_BALL_LINK_MEMBERSHIPS, B_link_memberships);

        if (REQUEST_COLLISION_BALLS_CMD_ID == cmd_id) return;

        // We go ahead and process the next command

        mxArray* enclosing_cylinder_radii = NULL;
        double* enclosing_cylinder_radii_ptr = NULL;
        // Allocate MATLAB memory for enclosing cylinder radii 
        allocate_memory_for_matrix(enclosing_cylinder_radii_ptr, enclosing_cylinder_radii, 6, 1);
        ur_kin.get_enclosing_cylinder_radius_list(B1_c, B2_c, B3_c, B4_c, B5_c, B6_c, B7_c, B8_c, M1, M2, M3, M4, M5, M6, enclosing_cylinder_radii_ptr);
        mxSetFieldByNumber(plhs[0], 0, IDX_ENCLOSING_CYLINDER_RADII, enclosing_cylinder_radii);

        if (REQUEST_ENCLOSING_CYLINDERS_CMD_ID == cmd_id) return;
    }

	if ( REQUEST_INVERSE_KINEMATICS_CMD_ID == cmd_id )
	{
		// Get T_ee (end-effector)
        mxDouble* T_ee_ptr = mxGetDoubles(prhs[1]);
        mxDouble* q6_des = mxGetDoubles(prhs[2]);
    // std::cout << "\n START SRC \n" << std::endl;
    // for (size_t mk = 0; mk < 16; mk++)
    // {
    //     std::cout << T_ee_ptr[mk] << std::endl;
    // }
    // std::cout << "\n END SRC \n" << std::endl;
		double* num_sols = NULL;

		double* ik_sol = NULL;
		mxArray* ik_sol_out = NULL;
        allocate_memory_for_matrix(ik_sol, ik_sol_out, 6, 8);
        plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
		num_sols = mxGetDoubles(plhs[1]);

		*num_sols = ur_kin.get_inverse_kinematics(T_ee_ptr, *q6_des, ik_sol);
        plhs[0] = ik_sol_out;
		 
		return;
	}

    if ( REQUEST_COMPUTE_RHO_CMD_ID == cmd_id )
    {
        // Get joint angles
        const mxDouble *const q1 = mxGetDoubles(prhs[1]);
        const mxDouble *const q2 = mxGetDoubles(prhs[2]);

        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);

        double* max_dist = mxGetDoubles(plhs[0]);
        ur_kin.compute_rho(q1, q2, max_dist);
		return;
    }
    // if ( DUMMY_ID == cmd_id )
    // {
    //     std::cout << "\nDUMMY\n" << std::endl;
    //     const char* fnames[] = {"A", "B"};
    //     plhs[0] = mxCreateStructMatrix(1, 1, 2, fnames);
    //     std::cout << "\nCREATED\n" << std::endl;
    //     mxArray* entry = mxCreateDoubleMatrix(1, 1, mxREAL);
    //     double* entry_ptr = mxGetDoubles(entry);
    //     entry_ptr[0] = 5.5;
    //     std::cout << "\nSET 1\n" << std::endl;
    //
    //     mxArray* entry2 = mxCreateDoubleMatrix(1, 1, mxREAL);
    //     double* entry2_ptr = mxGetDoubles(entry2);
    //     entry2_ptr[0] = 10.5;
    //     std::cout << "\nSET 2\n" << std::endl;
    //
    //     mxSetFieldByNumber(plhs[0], 0, 0, entry);
    //     mxSetFieldByNumber(plhs[0], 0, 1, entry2);
    //     std::cout << "\nDONE\n" << std::endl;
    // }
    
}
