#include "unicyclesWithRods_4_dynamics.hpp"
#include <cmath>

namespace dynobench {
void calcV_unicyclesWithRods_4(double* ff, double l1, double l2, double l3,
                               const double *x, const double *u) {
    // ** Extract state variables **
    double px1 = x[0], py1 = x[1];
    double alpha1 = x[2], alpha2 = x[3], alpha3 = x[4], alpha4 = x[5];
    double theta1 = x[6], theta2 = x[7], theta3 = x[8];

    // ** Extract control inputs **
    double v1 = u[0], omega1 = u[1];
    double v2 = u[2], omega2 = u[3];
    double v3 = u[4], omega3 = u[5];
    double v4 = u[6], omega4 = u[7];

    // ** Compute positions of other robots **
    double px2 = px1 + l1 * cos(theta1);
    double py2 = py1 + l1 * sin(theta1);
    double px3 = px2 + l2 * cos(theta2);
    double py3 = py2 + l2 * sin(theta2);
    double px4 = px3 + l3 * cos(theta3);
    double py4 = py3 + l3 * sin(theta3);

    // ** Construct B Matrix **
    Eigen::MatrixXd B(12, 8);
    B.setZero();
    // Robot 1
    B(0, 0) = cos(alpha1);
    B(1, 0) = sin(alpha1);
    B(2, 1) = 1;
    // Robot 2
    B(3, 2) = cos(alpha2);
    B(4, 2) = sin(alpha2);
    B(5, 3) = 1;
    // Robot 3
    B(6, 4) = cos(alpha3);
    B(7, 4) = sin(alpha3);
    B(8, 5) = 1;
    // Robot 4
    B(9, 6) = cos(alpha4);
    B(10, 6) = sin(alpha4);
    B(11, 7) = 1;

    // ** Construct A Matrix **
    Eigen::MatrixXd A(3, 12);
    A.setZero();
    A(0, 0) = 2 * (px1 - px2);
    A(0, 1) = 2 * (py1 - py2);
    A(0, 3) = -2 * (px1 - px2);
    A(0, 4) = -2 * (py1 - py2);

    A(1, 3) = 2 * (px2 - px3);
    A(1, 4) = 2 * (py2 - py3);
    A(1, 6) = -2 * (px2 - px3);
    A(1, 7) = -2 * (py2 - py3);

    A(2, 6) = 2 * (px3 - px4);
    A(2, 7) = 2 * (py3 - py4);
    A(2, 9) = -2 * (px3 - px4);
    A(2, 10) = -2 * (py3 - py4);

    // ** Compute Pseudoinverse of A **
    Eigen::MatrixXd A_pinv = A.transpose() * (A * A.transpose()).inverse();

    // ** Compute G Matrix **
    Eigen::MatrixXd G = B - A_pinv * (A * B);

    // ** Compute Constrained Dynamics **
    Eigen::VectorXd u_vec(8);
    u_vec << v1, omega1, v2, omega2, v3, omega3, v4, omega4;
    Eigen::VectorXd state_dot = G * u_vec;

    // ** Compute theta_dot values **
    double dpx1 = state_dot[0];
    double dpy1 = state_dot[1];
    double dpx2 = state_dot[3];
    double dpy2 = state_dot[4];
    double dpx3 = state_dot[6];
    double dpy3 = state_dot[7];
    double dpx4 = state_dot[9];
    double dpy4 = state_dot[10];

    double dx1 = px2 - px1, dy1 = py2 - py1;
    double theta_dot1 = (dx1 * (dpy2 - dpy1) - dy1 * (dpx2 - dpx1)) / (l1 * l1);

    double dx2 = px3 - px2, dy2 = py3 - py2;
    double theta_dot2 = (dx2 * (dpy3 - dpy2) - dy2 * (dpx3 - dpx2)) / (l2 * l2);

    double dx3 = px4 - px3, dy3 = py4 - py3;
    double theta_dot3 = (dx3 * (dpy4 - dpy3) - dy3 * (dpx4 - dpx3)) / (l3 * l3);

    // ** Assign results to ff (new state representation) **
    ff[0] = state_dot[0];  // dx1
    ff[1] = state_dot[1];  // dy1
    ff[2] = state_dot[2];  // dalpha1
    ff[3] = state_dot[5];  // dalpha2
    ff[4] = state_dot[8];  // dalpha3
    ff[5] = state_dot[11]; // dalpha4
    ff[6] = theta_dot1;    // dtheta1
    ff[7] = theta_dot2;    // dtheta2
    ff[8] = theta_dot3;    // dtheta3
}
}
