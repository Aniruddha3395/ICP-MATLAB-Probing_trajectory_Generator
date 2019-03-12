#include "mex.h"
#include "matrix.h"
#include <iostream>
#include "string.h"
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <stdio.h>
#include <vector>

Eigen::MatrixXd generate_grid_points(double, double, double, double, double, double);
Eigen::VectorXd linsp(double, double, double);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    double *ptr_pathgap_x;
    double *ptr_pathgap_y;
    double *ptr_xmin;
    double *ptr_ymin;
    double *ptr_xmax;
    double *ptr_ymax;
    ptr_pathgap_x = mxGetDoubles(MatlabIn[0]);
    ptr_pathgap_y = mxGetDoubles(MatlabIn[1]);
    ptr_xmin = mxGetDoubles(MatlabIn[2]);
    ptr_ymin = mxGetDoubles(MatlabIn[3]);
    ptr_xmax = mxGetDoubles(MatlabIn[4]);
    ptr_ymax = mxGetDoubles(MatlabIn[5]);
    double pathgap_x = *ptr_pathgap_x;
    double pathgap_y = *ptr_pathgap_y;
    double xmin = *ptr_xmin;
    double ymin = *ptr_ymin;
    double xmax = *ptr_xmax;
    double ymax = *ptr_ymax;
    
    // Method 
    Eigen::MatrixXd pts = generate_grid_points(pathgap_x, pathgap_y, xmin, ymin, xmax, ymax);

    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(pts.rows(), pts.cols(), mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 ( mxGetPr(MatlabOut[0]), pts.rows(), pts.cols() );
    M0 = pts.array(); 
}

Eigen::MatrixXd generate_grid_points(double pathgap_x, double pathgap_y, double xmin, double ymin, double xmax, double ymax)
{
    // Function to generate the uniform mesh grid of points along the x-y plane
    // INPUT = gap between the adjacent points and maximum value in x and y direction
    // OUTPUT = All points consisting the uniform grid

    Eigen::VectorXd j = linsp(floor(ymin),ceil(ymax),pathgap_y);
    Eigen::VectorXd i_val = linsp(floor(xmin),ceil(xmax),pathgap_x);
    Eigen::MatrixXd pts = Eigen::MatrixXd::Constant(j.rows()*i_val.rows(),2,0);
    long int st_pt = 0;
    for (long int i=0;i<i_val.rows();++i)
    {
        pts.block(st_pt,0,j.rows(),1) = i_val(i)*Eigen::MatrixXd::Constant(j.rows(),1,1);
        pts.block(st_pt,1,j.rows(),1) = j.block(0,0,j.rows(),j.cols());
        st_pt = st_pt + j.rows();
    }
    return pts; 
}

Eigen::VectorXd linsp(double strt, double end, double stp)
{
    int sz;
    if (strt<=end && stp>0 || strt>=end && stp<0)
    {
        sz = int((end-strt)/stp)+1;
    }
    else
    {
        if (strt>=end)
        {
            std::cerr << "start value is greater than the end value for incement!" << std::endl;
            std::terminate();   
        }
        else
        {
            std::cerr << "start value is less than the end value for decrement!" << std::endl;
            std::terminate();   
        }
    }
    return Eigen::VectorXd::LinSpaced(sz,strt,strt+stp*(sz-1));
}