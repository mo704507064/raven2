/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU  General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */


/** Methods for r2_jacobian class
 *
 *
 *  \date May 17, 2016
 *  \author Andy Lewis
 *
 *  \ingroup control
 *  \ingroup Kinematics
 */


#include "DS0.h"
#include <stdio.h>
#include <iostream>
#include "r2_jacobian_defs.h"

extern int NUM_MECH;

/** r2_jacobian constructor with pre-set velocities and forces
 *
 *	\param vel[6]    the velocities for constructing the r2_jacobian object
 *	\param f[6]      the forces for constructing the r2_jacobian object
 *
 */
r2_jacobian::r2_jacobian(Eigen::VectorXf vel, Eigen::VectorXf f) {
	set_vel(vel);
	set_force(f);
	return;
}

/** set the velocities of an r2_jacobian
 *
 *	\param Eigen::VectorXf vel   the new velocities for the jacobian
 *
 *	\return void
 */
void r2_jacobian::set_vel(Eigen::VectorXf vel){
	velocity = vel;
	return;
}

/** get the velocities of an r2_jacobian
 *
 *	\param vel[6]   array to put velocities values into
 *
 *	\return void
 */
void r2_jacobian::get_vel(float vel[6]){

	return;
}

/** set the forces of an r2_jacobian
 *
 *	\param Eigen::VectorXf f   the new force vector for the jacobian
 *
 *	\return void
 */
void r2_jacobian::set_force(Eigen::VectorXf f){
	force = f;
    return;
}

/** get the forces of an r2_jacobian
 *
 *	\param f[6]   array to put force values into
 *
 *	\return void
 */
void r2_jacobian::get_force(float f[]){

    return;
}

/** This function is the bones of the r2_jacobian class
 *
 * \desc 	This function is the bones of the r2_jacobian class -
 * 			it orchestrates the calculation of the Jacobian, and
 * 			end effector velocities and torques.
 *
 * \param  j_pos[6]      the joint states of the RAVEN
 * \param  j_vel[6]      the joint velocities of the RAVEN
 * \param  j_torque[6]   the joint torques of the RAVEN
 *
 * \return int   success = 1
 */
int r2_jacobian::update_r2_jacobian(float j_pos[6], float j_vel[6], float j_torque[6], int arm_type){
	int success = 0;

	//recalculate matrix based on j pos
	success = calc_jacobian(j_pos, arm_type); //success = 1 if jacobian is calculated successfully

	//calculate jacobian velocities
	success &= calc_velocities(j_vel); //success if velocities also calculated

	//calculate jacobian forces
	success &= calc_forces(j_torque); //success if velocities also calculated

	static int check = 0;
	if (check %3000 == 0){
		printf("updated Jacobian! \n");
		std::cout<<j_matrix<<std::endl;
		check = 0;
	}
	check++;

	return success;

}



/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_vel  joint velocities of robot
 *
 * \return int   success = 1
 *
 */
int r2_jacobian::calc_velocities(float j_vel[6]){
	int success = 0;

	return success;
}


/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_torques  joint torques of robot
 *
 * \return int   success = 1
 */
int r2_jacobian::calc_forces(float j_torques[6]){
	int success = 0;

	return success;
}

/** called from rt_raven in order to start the calculation process with a robot device
 *
 * \desc Acts on the jacobian objects in each mechanism of the given device. Doesn't use
 * 		 runlevel yet.
 *
 * \param d0         device to calculate jacobians for
 * \param runlevel	 runlevel of device, not used yet
 *
 * \return int success = 1
 *
 */
int r2_device_jacobian(struct robot_device *d0, int runlevel){
	int success = 1;
	float j_pos[6];
	float j_vel[6];
	float j_torque[6];
	int arm_type;


	for (int m=0; m<NUM_MECH; m++){
		//populate arrays for updating jacobian
		for (int i = 0; i < 6; i++){
			j_pos[i] = d0->mech[m].joint[i].jpos;
			j_vel[i] = d0->mech[m].joint[i].jvel;
			/** \todo add joint torque calculation
			 * tau_d is after gearbox  */
			j_torque[i] = i;
		}
		arm_type = d0->mech[m].type;
		//calculate jacobian values for this mech
		success &= d0->mech[m].r2_jac.update_r2_jacobian(j_pos, j_vel, j_torque, arm_type);

	}


	return success;
}


/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_pos  joint positions of robot
 *
 * \return int   success = 1
 *
 */
int r2_jacobian::calc_jacobian(float j_pos[6], int arm_type){
	int success = 0;
//row 1	(0-indexed)
	j_matrix(0,0) = L*Ca12;
	j_matrix(0,1) = 1;
	j_matrix(0,2) = 1;
	j_matrix(0,3) = 1;
	j_matrix(0,4) = 1;
	j_matrix(0,5) = 0;	//done
//row 2
	j_matrix(1,0) = 1;
	j_matrix(1,1) = 1;
	j_matrix(1,2) = 1;
	j_matrix(1,3) = 1;
	j_matrix(1,4) = 1;
	j_matrix(1,5) = 0; //done
//row 3
	j_matrix(2,0) = 1;
	j_matrix(2,1) = 1;
	j_matrix(2,2) = 1;
	j_matrix(2,3) = 1;
	j_matrix(2,4) = 0; //done
	j_matrix(2,5) = 1;
//row 4
	j_matrix(3,0) = 1;
	j_matrix(3,1) = 1;
	j_matrix(3,2) = 1;
	j_matrix(3,3) = 0; //done
	j_matrix(3,4) = 1;
	j_matrix(3,5) = 1;
//row 5
	j_matrix(4,0) = 1;
	j_matrix(4,1) = 1;
	j_matrix(4,2) = 1;
	j_matrix(4,3) = 0; //done
	j_matrix(4,4) = 1;
	j_matrix(4,5) = 1;
//row 6
	j_matrix(5,0) = 1;
	j_matrix(5,1) = 1;
	j_matrix(5,2) = 1;
	j_matrix(5,3) = 0; //done
	j_matrix(5,4) = 1;
	j_matrix(5,5) = 0; //done

	return success;
}

