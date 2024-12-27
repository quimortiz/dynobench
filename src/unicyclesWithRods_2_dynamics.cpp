#include "unicyclesWithRods_2_dynamics.hpp"
#include <cmath>

namespace dynobench {
void calcV_unicyclesWithRods_2(double* ff, double l1, const double *x, const double *u) {
    // ** Extract state variables **
    double px1 = x[0], py1 = x[1];
    double alpha1 = x[2], alpha2 = x[3];
    double theta1 = x[4];

    // ** Extract control inputs **
    double v1 = u[0], omega1 = u[1];
    double v2 = u[2], omega2 = u[3];

    // ** Compute positions of the second robot **
    double px2 = px1 + l1 * cos(theta1);
    double py2 = py1 + l1 * sin(theta1);

    // ** Construct B Matrix **
    Eigen::MatrixXd B(6, 4);
    B.setZero();
    // Robot 1
    B(0, 0) = cos(alpha1);
    B(1, 0) = sin(alpha1);
    B(2, 1) = 1;
    // Robot 2
    B(3, 2) = cos(alpha2);
    B(4, 2) = sin(alpha2);
    B(5, 3) = 1;

    // ** Construct A Matrix **
    Eigen::MatrixXd A(1, 6);
    A.setZero();
    A(0, 0) = 2 * (px1 - px2);
    A(0, 1) = 2 * (py1 - py2);
    A(0, 3) = -2 * (px1 - px2);
    A(0, 4) = -2 * (py1 - py2);

    // ** Compute Pseudoinverse of A **
    Eigen::MatrixXd A_pinv = A.transpose() * (A * A.transpose()).inverse();

    // ** Compute G Matrix **
    Eigen::MatrixXd G = B - A_pinv * (A * B);

    // ** Compute Constrained Dynamics **
    Eigen::VectorXd u_vec(4);
    u_vec << v1, omega1, v2, omega2;
    Eigen::VectorXd state_dot = G * u_vec;

    // ** Compute theta_dot1 (rod between robot 1 and robot 2) **
    double dpx1 = state_dot[0];
    double dpy1 = state_dot[1];
    double dpx2 = state_dot[3];
    double dpy2 = state_dot[4];
    double dx1 = px2 - px1, dy1 = py2 - py1;
    double theta_dot1 = (dx1 * (dpy2 - dpy1) - dy1 * (dpx2 - dpx1)) / (l1 * l1);

    // ** Assign results to ff (new state representation) **
    ff[0] = state_dot[0];  // dx1
    ff[1] = state_dot[1];  // dy1
    ff[2] = state_dot[2];  // dalpha1
    ff[3] = state_dot[5];  // dalpha2
    ff[4] = theta_dot1;  // dtheta1
}
}
