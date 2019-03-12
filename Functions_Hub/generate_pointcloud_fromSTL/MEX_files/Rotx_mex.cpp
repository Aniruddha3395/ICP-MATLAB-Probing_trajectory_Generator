#include "mex.h"
#include "matrix.h"
#include <iostream>
#include "string.h"
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <stdio.h>
#include <vector>

Eigen::Matrix3d rot_x(double);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    double *ptr_t;
    ptr_t = mxGetDoubles(MatlabIn[0]);
    double t = *ptr_t;

    // Method 
    Eigen::Matrix3d T = rot_x(t);

    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(3,3,mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 (mxGetPr(MatlabOut[0]),3,3);
    M0 = T.array(); 
}

Eigen::Matrix3d rot_x(double t)
{
	Eigen::Matrix3d rx;
	rx << 	1,		0,		0,
			0, cos(t),-sin(t),
			0, sin(t), cos(t);
	return rx;  
}