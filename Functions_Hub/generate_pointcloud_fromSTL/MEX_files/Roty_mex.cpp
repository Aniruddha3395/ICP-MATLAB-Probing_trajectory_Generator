#include "mex.h"
#include "matrix.h"
#include <iostream>
#include "string.h"
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <stdio.h>
#include <vector>

Eigen::Matrix3d rot_y(double);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    double *ptr_t;
    ptr_t = mxGetDoubles(MatlabIn[0]);
    double t = *ptr_t;

    // Method 
    Eigen::Matrix3d T = rot_y(t);

    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(3,3,mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 (mxGetPr(MatlabOut[0]),3,3);
    M0 = T.array(); 
}

Eigen::Matrix3d rot_y(double t)
{
	Eigen::Matrix3d ry;
	ry << cos(t),	0, sin(t),
			   0,	1,		0,
		 -sin(t),	0, cos(t);
	return ry;  
}