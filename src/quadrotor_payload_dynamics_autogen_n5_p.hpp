#pragma once

// Auto generated file
// Created at: 2023-09-05--18-05-18

#include "dynobench/quadrotor_payload_n.hpp"


namespace dynobench {

void calcV_n5_p(Eigen::Ref<Eigen::VectorXd> ff, const Quad3dpayload_n_params &params, const double *x, const double *u);

void calcStep_n5_p(Eigen::Ref<Eigen::VectorXd> xnext, const Quad3dpayload_n_params &params, const double *x, const double *u, double dt);

void calcJ_n5_p(Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u, const Quad3dpayload_n_params &params, const double *x, const double *u);

void calcF_n5_p(Eigen::Ref<Eigen::MatrixXd> Fx,Eigen::Ref<Eigen::MatrixXd> Fu, const Quad3dpayload_n_params &params,const double *x, const double *u,double dt);

}
