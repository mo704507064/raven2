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
 *  Created on: May 17, 2016
 *  Author: Andy Lewis
 *
 *  /ingroup control
 */

#include "r2_jacobian.h"
#include "defines.h"

r2_jacobian::r2_jacobian(float vel[6], float f[6]) {
	set_vel(vel);
	set_force(f);
	return;
}

void r2_jacobian::set_vel(float vel[6]){
	for(int i = 1; i < 6; i++){
			velocity[i] = vel[i];
	}
	return;
}

void r2_jacobian::set_force(float f[6]){
	for(int i = 1; i < 6; i++){
		force[i] = f[i];
	}
    return;
}



