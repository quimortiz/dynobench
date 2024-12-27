#pragma once

// Auto generated file
// Created at: 2024-06-28--19-00-16

namespace dynobench {

void calcV_DintegratorCables(double *ff, double m0, double m1, double m2,
                             double l1, double l2, const double *x,
                             const double *u);

void calcStep_DintegratorCables(double *xnext, double m0, double m1, double m2,
                                double l1, double l2, const double *x,
                                const double *u, double dt);

void calcJ_DintegratorCables(double *Jx, double *Ju, double m0, double m1,
                             double m2, double l1, double l2, const double *x,
                             const double *u);

void calcF_DintegratorCables(double *Fx, double *Fu, double m0, double m1,
                             double m2, double l1, double l2, const double *x,
                             const double *u, double dt);

} // namespace dynobench
