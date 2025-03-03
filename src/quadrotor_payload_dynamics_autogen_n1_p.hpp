#pragma once

// Generated on 2025-03-03--13-40-41

namespace dynobench {
void calcV_n1_p(double* ff, double mp, double arm_length, double t2t, const double *m, const double *J_vx, const double *J_vy, const double *J_vz, const double *l, const double *x, const double *u);
void calcStep_n1_p(double* xnext, double mp, double arm_length, double t2t, const double *m, const double *J_vx, const double *J_vy, const double *J_vz, const double *l, const double *x, const double *u, double dt);
} // namespace dynobench
