#include "DintegratorCables_dynamics.hpp"
#include <cmath>
// Auto generated file
// Created at: 2024-08-15--11-17-45

namespace dynobench {
void calcV_DintegratorCables(double *ff, double m0, double m1, double m2,
                             double l1, double l2, const double *x,
                             const double *u) {

  ff[0] = x[4];
  ff[1] = x[5];
  ff[2] = x[6];
  ff[3] = x[7];
  ff[4] =
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) /
          sqrt(m0 + m1 + m2);
  ff[5] =
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) /
          sqrt(m0 + m1 + m2);
  ff[6] =
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0)) -
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]);
  ff[7] =
      -(-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]);
}
void calcStep_DintegratorCables(double *xnext, double m0, double m1, double m2,
                                double l1, double l2, const double *x,
                                const double *u, double dt) {

  xnext[0] = dt * x[4] + x[0];
  xnext[1] = dt * x[5] + x[1];
  xnext[2] = dt * x[6] + x[2];
  xnext[3] = dt * x[7] + x[3];
  xnext[4] =
      dt *
          ((-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
               (1.0 * l1 * m1 *
                    (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) /
                         (pow(-1.0 * pow(l1, 2) * pow(m1, 2) *
                                      pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                  1.0 * pow(l1, 2) * pow(m1, 2) *
                                      pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                  pow(l1, 2) * m1,
                              3.0 / 2.0) *
                          (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                               (m0 + m1 + m2) +
                           pow(l2, 2) * m2 -
                           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                       sin(x[3]) / (m0 + m1 + m2) -
                                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                       cos(x[3]) / (m0 + m1 + m2),
                               2) /
                               (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                    pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                1.0 * pow(l1, 2) * pow(m1, 2) *
                                    pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                pow(l1, 2) * m1))) +
                     pow(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                 (m0 + m1 + m2) -
                             1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                 (m0 + m1 + m2) +
                             pow(l1, 2) * m1,
                         -1.0 / 2.0)) *
                    sin(x[2]) /
                    (sqrt(m0 + m1 + m2) *
                     sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                              (m0 + m1 + m2) -
                          1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                              (m0 + m1 + m2) +
                          pow(l1, 2) * m1)) -
                1.0 * l2 * m2 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[3]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)))) /
               sqrt(m0 + m1 + m2) +
           (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1))) +
                1.0 * l2 * m2 * sin(x[3]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)))) /
               sqrt(m0 + m1 + m2) +
           (1.0 * l1 * m1 *
                (-1.0 * l1 * m1 * cos(x[2]) /
                     ((m0 + m1 + m2) *
                      sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)) -
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) /
                      ((m0 + m1 + m2) *
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) /
                     (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1) *
                      (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                           (m0 + m1 + m2) +
                       pow(l2, 2) * m2 -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) /
                           (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                (m0 + m1 + m2) -
                            1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                (m0 + m1 + m2) +
                            pow(l1, 2) * m1)))) *
                sin(x[2]) /
                (sqrt(m0 + m1 + m2) *
                 sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) +
            1.0 * l2 * m2 *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) /
                     ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                sin(x[3]) /
                (sqrt(m0 + m1 + m2) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1)))) *
               (1.0 * l1 * m1 * pow(x[6], 2) * sin(x[2]) +
                1.0 * l2 * m2 * pow(x[7], 2) * sin(x[3]) + u[1] + u[3]) /
               sqrt(m0 + m1 + m2) +
           (1.0 * l1 * m1 *
                (1.0 * l1 * m1 * sin(x[2]) /
                     ((m0 + m1 + m2) *
                      sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)) -
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) /
                      ((m0 + m1 + m2) *
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) /
                     (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1) *
                      (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                           (m0 + m1 + m2) +
                       pow(l2, 2) * m2 -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) /
                           (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                (m0 + m1 + m2) -
                            1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                (m0 + m1 + m2) +
                            pow(l1, 2) * m1)))) *
                sin(x[2]) /
                (sqrt(m0 + m1 + m2) *
                 sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) /
                     ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                sin(x[3]) /
                (sqrt(m0 + m1 + m2) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1))) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) *
               (1.0 * l1 * m1 * pow(x[6], 2) * cos(x[2]) +
                1.0 * l2 * m2 * pow(x[7], 2) * cos(x[3]) + u[0] + u[2]) /
               sqrt(m0 + m1 + m2)) +
      x[4];
  xnext[5] =
      dt *
          ((-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
               (-1.0 * l1 * m1 *
                    (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) /
                         (pow(-1.0 * pow(l1, 2) * pow(m1, 2) *
                                      pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                  1.0 * pow(l1, 2) * pow(m1, 2) *
                                      pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                  pow(l1, 2) * m1,
                              3.0 / 2.0) *
                          (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                               (m0 + m1 + m2) +
                           pow(l2, 2) * m2 -
                           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                       sin(x[3]) / (m0 + m1 + m2) -
                                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                       cos(x[3]) / (m0 + m1 + m2),
                               2) /
                               (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                    pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                1.0 * pow(l1, 2) * pow(m1, 2) *
                                    pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                pow(l1, 2) * m1))) +
                     pow(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                 (m0 + m1 + m2) -
                             1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                 (m0 + m1 + m2) +
                             pow(l1, 2) * m1,
                         -1.0 / 2.0)) *
                    cos(x[2]) /
                    (sqrt(m0 + m1 + m2) *
                     sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                              (m0 + m1 + m2) -
                          1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                              (m0 + m1 + m2) +
                          pow(l1, 2) * m1)) +
                1.0 * l2 * m2 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[3]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)))) /
               sqrt(m0 + m1 + m2) +
           (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1))) -
                1.0 * l2 * m2 * cos(x[3]) /
                    (sqrt(m0 + m1 + m2) *
                     (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                          (m0 + m1 + m2) +
                      pow(l2, 2) * m2 -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) /
                          (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)))) /
               sqrt(m0 + m1 + m2) +
           (-1.0 * l1 * m1 *
                (1.0 * l1 * m1 * sin(x[2]) /
                     ((m0 + m1 + m2) *
                      sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)) -
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) /
                      ((m0 + m1 + m2) *
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) /
                     (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1) *
                      (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                           (m0 + m1 + m2) +
                       pow(l2, 2) * m2 -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) /
                           (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                (m0 + m1 + m2) -
                            1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                (m0 + m1 + m2) +
                            pow(l1, 2) * m1)))) *
                cos(x[2]) /
                (sqrt(m0 + m1 + m2) *
                 sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) /
                     ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                cos(x[3]) /
                (sqrt(m0 + m1 + m2) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1)))) *
               (1.0 * l1 * m1 * pow(x[6], 2) * cos(x[2]) +
                1.0 * l2 * m2 * pow(x[7], 2) * cos(x[3]) + u[0] + u[2]) /
               sqrt(m0 + m1 + m2) +
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * m1 * cos(x[2]) /
                     ((m0 + m1 + m2) *
                      sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1)) -
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) /
                      ((m0 + m1 + m2) *
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) /
                     (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                               (m0 + m1 + m2) -
                           1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                               (m0 + m1 + m2) +
                           pow(l1, 2) * m1) *
                      (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                           (m0 + m1 + m2) +
                       pow(l2, 2) * m2 -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) /
                           (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                (m0 + m1 + m2) -
                            1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                (m0 + m1 + m2) +
                            pow(l1, 2) * m1)))) *
                cos(x[2]) /
                (sqrt(m0 + m1 + m2) *
                 sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) -
            1.0 * l2 * m2 *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) /
                     ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                cos(x[3]) /
                (sqrt(m0 + m1 + m2) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1))) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) *
               (1.0 * l1 * m1 * pow(x[6], 2) * sin(x[2]) +
                1.0 * l2 * m2 * pow(x[7], 2) * sin(x[3]) + u[1] + u[3]) /
               sqrt(m0 + m1 + m2)) +
      x[5];
  xnext[6] =
      dt * ((-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) /
                     (pow(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                  (m0 + m1 + m2) -
                              1.0 * pow(l1, 2) * pow(m1, 2) *
                                  pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                              pow(l1, 2) * m1,
                          3.0 / 2.0) *
                      (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                           (m0 + m1 + m2) +
                       pow(l2, 2) * m2 -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) /
                           (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                                (m0 + m1 + m2) -
                            1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                                (m0 + m1 + m2) +
                            pow(l1, 2) * m1))) +
                 pow(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                             (m0 + m1 + m2) -
                         1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                             (m0 + m1 + m2) +
                         pow(l1, 2) * m1,
                     -1.0 / 2.0)) /
                sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                         (m0 + m1 + m2) -
                     1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                         (m0 + m1 + m2) +
                     pow(l1, 2) * m1) -
            (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) /
                ((-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                      (m0 + m1 + m2) +
                  pow(l1, 2) * m1) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1))) +
            (1.0 * l1 * m1 * sin(x[2]) /
                 ((m0 + m1 + m2) * sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) -
             (-1.0 * l1 * m1 *
                  (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2)) *
                  sin(x[2]) /
                  ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                         pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                     1.0 * pow(l1, 2) * pow(m1, 2) *
                                         pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                     pow(l1, 2) * m1)) +
              1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) /
                 (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1) *
                  (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                       (m0 + m1 + m2) -
                   1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                       (m0 + m1 + m2) +
                   pow(l2, 2) * m2 -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) /
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)))) *
                (1.0 * l1 * m1 * pow(x[6], 2) * cos(x[2]) +
                 1.0 * l2 * m2 * pow(x[7], 2) * cos(x[3]) + u[0] + u[2]) /
                sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                         (m0 + m1 + m2) -
                     1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                         (m0 + m1 + m2) +
                     pow(l1, 2) * m1) +
            (-1.0 * l1 * m1 * cos(x[2]) /
                 ((m0 + m1 + m2) * sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                        1.0 * pow(l1, 2) * pow(m1, 2) *
                                            pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                        pow(l1, 2) * m1)) -
             (1.0 * l1 * m1 *
                  (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2)) *
                  cos(x[2]) /
                  ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                         pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                     1.0 * pow(l1, 2) * pow(m1, 2) *
                                         pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                     pow(l1, 2) * m1)) -
              1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) /
                 (sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1) *
                  (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                       (m0 + m1 + m2) -
                   1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                       (m0 + m1 + m2) +
                   pow(l2, 2) * m2 -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) /
                       (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                            (m0 + m1 + m2) -
                        1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                            (m0 + m1 + m2) +
                        pow(l1, 2) * m1)))) *
                (1.0 * l1 * m1 * pow(x[6], 2) * sin(x[2]) +
                 1.0 * l2 * m2 * pow(x[7], 2) * sin(x[3]) + u[1] + u[3]) /
                sqrt(-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                         (m0 + m1 + m2) -
                     1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                         (m0 + m1 + m2) +
                     pow(l1, 2) * m1)) +
      x[6];
  xnext[7] =
      dt * (-(-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) /
                ((-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                      (m0 + m1 + m2) +
                  pow(l1, 2) * m1) *
                 (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                      (m0 + m1 + m2) -
                  1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                      (m0 + m1 + m2) +
                  pow(l2, 2) * m2 -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) /
                      (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                           (m0 + m1 + m2) -
                       1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                           (m0 + m1 + m2) +
                       pow(l1, 2) * m1))) +
            (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) /
                (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                     (m0 + m1 + m2) -
                 1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                     (m0 + m1 + m2) +
                 pow(l2, 2) * m2 -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) /
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) +
            (-1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 sin(x[2]) /
                 ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                        pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                    1.0 * pow(l1, 2) * pow(m1, 2) *
                                        pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                    pow(l1, 2) * m1)) +
             1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 * pow(x[6], 2) * cos(x[2]) +
                 1.0 * l2 * m2 * pow(x[7], 2) * cos(x[3]) + u[0] + u[2]) /
                (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                     (m0 + m1 + m2) -
                 1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                     (m0 + m1 + m2) +
                 pow(l2, 2) * m2 -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) /
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1)) +
            (1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 cos(x[2]) /
                 ((m0 + m1 + m2) * (-1.0 * pow(l1, 2) * pow(m1, 2) *
                                        pow(sin(x[2]), 2) / (m0 + m1 + m2) -
                                    1.0 * pow(l1, 2) * pow(m1, 2) *
                                        pow(cos(x[2]), 2) / (m0 + m1 + m2) +
                                    pow(l1, 2) * m1)) -
             1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 * pow(x[6], 2) * sin(x[2]) +
                 1.0 * l2 * m2 * pow(x[7], 2) * sin(x[3]) + u[1] + u[3]) /
                (-1.0 * pow(l2, 2) * pow(m2, 2) * pow(sin(x[3]), 2) /
                     (m0 + m1 + m2) -
                 1.0 * pow(l2, 2) * pow(m2, 2) * pow(cos(x[3]), 2) /
                     (m0 + m1 + m2) +
                 pow(l2, 2) * m2 -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) /
                     (-1.0 * pow(l1, 2) * pow(m1, 2) * pow(sin(x[2]), 2) /
                          (m0 + m1 + m2) -
                      1.0 * pow(l1, 2) * pow(m1, 2) * pow(cos(x[2]), 2) /
                          (m0 + m1 + m2) +
                      pow(l1, 2) * m1))) +
      x[7];
}
void calcJ_DintegratorCables(double *Jx, double *Ju, double m0, double m1,
                             double m2, double l1, double l2, const double *x,
                             const double *u) {
  Jx[32] = 1;
  Jx[41] = 1;
  Jx[50] = 1;
  Jx[59] = 1;
  Jx[20] =
      -1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    3) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -5.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) +
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * m1 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * m1 *
                         (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2)) -
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * m1 *
                         (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2)) -
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[28] =
      -1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    3) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -5.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) +
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * m1 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) +
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) +
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[52] =
      2.0 * l1 * m1 * x[6] * sin(x[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      2.0 * l1 * m1 * x[6] * cos(x[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Jx[60] =
      2.0 * l2 * m2 * x[7] * sin(x[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      2.0 * l2 * m2 * x[7] * cos(x[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Jx[21] =
      -1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    3) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -5.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) +
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (1.0 * l1 * m1 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l1 * m1 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * m1 *
                         (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2)) -
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * m1 *
                         (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2)) -
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[29] =
      -1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    3) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -5.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) +
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (1.0 * l1 * m1 *
               pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2),
                   2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -2) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) +
           1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) +
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                    (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    pow(m2 * (l2 * l2) -
                            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                        sin(x[3]) / (m0 + m1 + m2) -
                                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                        cos(x[3]) / (m0 + m1 + m2),
                                2) *
                                1.0 /
                                (m1 * (l1 * l1) -
                                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2) -
                                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                     (m0 + m1 + m2)) -
                            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                (m0 + m1 + m2),
                        -2) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2) +
                          1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[53] =
      2.0 * l1 * m1 * x[6] * sin(x[2]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      2.0 * l1 * m1 * x[6] * cos(x[2]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[61] =
      2.0 * l2 * m2 * x[7] * sin(x[3]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (-1.0 * l1 * m1 * cos(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         cos(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) -
                     1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2) +
      2.0 * l2 * m2 * x[7] * cos(x[3]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (1.0 * l1 * m1 * sin(x[2]) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) /
                    (m0 + m1 + m2) -
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -1.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) *
                    (-1.0 * l1 * m1 *
                         (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2)) *
                         sin(x[2]) * 1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) /
                         (m0 + m1 + m2) +
                     1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Jx[22] =
      -1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) +
      1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               3) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -5.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) +
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2))) +
      (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0)) -
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2),
              2) *
          (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -2) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) -
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) -
           (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
          (1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l1 * m1 *
                    (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2)) -
           (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Jx[30] =
      -1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) +
      1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               3) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -5.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) +
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2))) -
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2),
              2) *
          (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -2) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) -
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) -
      (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
               (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               pow(m2 * (l2 * l2) -
                       pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                   (m0 + m1 + m2) -
                               1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                   (m0 + m1 + m2),
                           2) *
                           1.0 /
                           (m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2)) -
                       1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                           (m0 + m1 + m2),
                   -2) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2) +
                     1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]);
  Jx[54] =
      2.0 * l1 * m1 * x[6] * sin(x[2]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
      2.0 * l1 * m1 * x[6] * cos(x[2]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Jx[62] =
      2.0 * l2 * m2 * x[7] * sin(x[3]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
      2.0 * l2 * m2 * x[7] * cos(x[3]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Jx[23] =
      -1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) +
      1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2),
              2) *
          (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -2) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) -
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) -
      (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) +
      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l1 * m1 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l1 * m1 *
               (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]);
  Jx[31] =
      -1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) +
      1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2),
              2) *
          (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -2) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) -
      (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) +
      (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
           2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
          1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          pow(m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
              -2) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2) +
                1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
           1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]);
  Jx[55] =
      2.0 * l1 * m1 * x[6] * sin(x[2]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) +
      2.0 * l1 * m1 * x[6] * cos(x[2]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2));
  Jx[63] =
      2.0 * l2 * m2 * x[7] * sin(x[3]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) +
      2.0 * l2 * m2 * x[7] * cos(x[3]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2));
  Ju[4] =
      -l1 * sin(x[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Ju[12] =
      l1 * cos(x[2]) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Ju[20] =
      -l2 * sin(x[3]) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Ju[28] =
      l2 * cos(x[3]) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (1.0 * l1 * m1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Ju[5] =
      -l1 * sin(x[2]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Ju[13] =
      l1 * cos(x[2]) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    pow(m1 * (l1 * l1) -
                            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2) -
                            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                (m0 + m1 + m2),
                        -3.0 / 2.0) *
                    1.0 /
                    (m2 * (l2 * l2) -
                     pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                 (m0 + m1 + m2) -
                             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                 (m0 + m1 + m2),
                         2) *
                         1.0 /
                         (m1 * (l1 * l1) -
                          1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2) -
                          1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                              (m0 + m1 + m2)) -
                     1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                         (m0 + m1 + m2)) +
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0)) /
               sqrt(m0 + m1 + m2) +
           1.0 * l2 * m2 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[3]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Ju[21] =
      -l2 * sin(x[3]) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2);
  Ju[29] =
      l2 * cos(x[3]) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) * 1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
               sqrt(m0 + m1 + m2)) /
          sqrt(m0 + m1 + m2) +
      (-1.0 * l1 * m1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
           sqrt(m0 + m1 + m2) -
       1.0 * l2 * m2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       pow(m0 + m1 + m2, -1.0 / 2.0)) /
          sqrt(m0 + m1 + m2);
  Ju[6] =
      -l1 * sin(x[2]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Ju[14] =
      l1 * cos(x[2]) *
          pow(m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
              -1.0 / 2.0) *
          (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -3.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)));
  Ju[22] =
      l2 *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          sin(x[3]) * 1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (1.0 * l1 * m1 * sin(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (-1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    sin(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) +
                1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Ju[30] =
      -l2 *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          cos(x[3]) * 1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      pow(m1 * (l1 * l1) -
              1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
              1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
          -1.0 / 2.0) *
          (-1.0 * l1 * m1 * cos(x[2]) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) /
               (m0 + m1 + m2) -
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
               pow(m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2),
                   -1.0 / 2.0) *
               1.0 /
               (m2 * (l2 * l2) -
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                    1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) -
                1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
               (1.0 * l1 * m1 *
                    (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2)) *
                    cos(x[2]) * 1.0 /
                    (m1 * (l1 * l1) -
                     1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2) -
                     1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                         (m0 + m1 + m2)) /
                    (m0 + m1 + m2) -
                1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)));
  Ju[7] =
      l1 *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          sin(x[2]) * 1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2));
  Ju[15] =
      -l1 *
          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
          cos(x[2]) * 1.0 /
          (m1 * (l1 * l1) -
           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
          1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2));
  Ju[23] =
      -l2 * sin(x[3]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (-1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               sin(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) +
           1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2));
  Ju[31] =
      l2 * cos(x[3]) * 1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
      1.0 /
          (m2 * (l2 * l2) -
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
               1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
           1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
           1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
          (1.0 * l1 * m1 *
               (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                    (m0 + m1 + m2) -
                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                    (m0 + m1 + m2)) *
               cos(x[2]) * 1.0 /
               (m1 * (l1 * l1) -
                1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
               (m0 + m1 + m2) -
           1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2));
}
void calcF_DintegratorCables(double *Fx, double *Fu, double m0, double m1,
                             double m2, double l1, double l2, const double *x,
                             const double *u, double dt) {
  Fx[0] = 1;
  Fx[32] = dt;
  Fx[9] = 1;
  Fx[41] = dt;
  Fx[18] = 1;
  Fx[50] = dt;
  Fx[27] = 1;
  Fx[59] = dt;
  Fx[20] =
      dt *
      (-1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     3) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -5.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) +
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (-1.0 * l1 * m1 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * m1 *
                          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2)) -
                 (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * m1 *
                          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2)) -
                 (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[28] =
      dt *
      (-1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     3) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -5.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) +
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (-1.0 * l1 * m1 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) +
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) +
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) +
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) +
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[36] = 1;
  Fx[52] =
      dt *
      (2.0 * l1 * m1 * x[6] * sin(x[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       2.0 * l1 * m1 * x[6] * cos(x[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fx[60] =
      dt *
      (2.0 * l2 * m2 * x[7] * sin(x[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       2.0 * l2 * m2 * x[7] * cos(x[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fx[21] =
      dt *
      (-1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     3) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -5.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) +
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (1.0 * l1 * m1 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l1 * m1 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * m1 *
                          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2)) -
                 (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * m1 *
                          (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2)) -
                 (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[29] =
      dt *
      (-1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     3) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -5.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) +
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (1.0 * l1 * m1 *
                pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                            (m0 + m1 + m2) -
                        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                            (m0 + m1 + m2),
                    2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -2) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) +
            1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) +
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) +
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
        1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                     (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     pow(m2 * (l2 * l2) -
                             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) *
                                         sin(x[3]) / (m0 + m1 + m2) -
                                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                         cos(x[3]) / (m0 + m1 + m2),
                                 2) *
                                 1.0 /
                                 (m1 * (l1 * l1) -
                                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2) -
                                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                      (m0 + m1 + m2)) -
                             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                                 (m0 + m1 + m2),
                         -2) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2) +
                           1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2) +
                  1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[45] = 1;
  Fx[53] =
      dt *
      (2.0 * l1 * m1 * x[6] * sin(x[2]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       2.0 * l1 * m1 * x[6] * cos(x[2]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[61] =
      dt *
      (2.0 * l2 * m2 * x[7] * sin(x[3]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2) +
       2.0 * l2 * m2 * x[7] * cos(x[3]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fx[22] =
      dt *
      (-1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) +
       1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                3) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -5.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) +
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                     (m0 + m1 + m2))) +
       (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0)) -
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
           (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -2) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) -
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) -
            (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) *
           (1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * m1 *
                     (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2)) -
            (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))));
  Fx[30] =
      dt *
      (-1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                3) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -5.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) +
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                     (m0 + m1 + m2))) -
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
           (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -2) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) -
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) -
       (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
             1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (-pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                pow(m2 * (l2 * l2) -
                        pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                    (m0 + m1 + m2) -
                                1.0 * l1 * l2 * m1 * m2 * cos(x[2]) *
                                    cos(x[3]) / (m0 + m1 + m2),
                            2) *
                            1.0 /
                            (m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2)) -
                        1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                            (m0 + m1 + m2),
                    -2) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2) +
                      1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
             1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]));
  Fx[54] =
      dt * (2.0 * l1 * m1 * x[6] * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (-1.0 * l1 * m1 * cos(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          cos(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) -
                      1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
            2.0 * l1 * m1 * x[6] * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (1.0 * l1 * m1 * sin(x[2]) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) /
                     (m0 + m1 + m2) -
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -1.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) *
                     (-1.0 * l1 * m1 *
                          (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2)) *
                          sin(x[2]) * 1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) /
                          (m0 + m1 + m2) +
                      1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)))) +
      1;
  Fx[62] =
      dt *
      (2.0 * l2 * m2 * x[7] * sin(x[3]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) +
       2.0 * l2 * m2 * x[7] * cos(x[3]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))));
  Fx[23] =
      dt *
      (-1.0 * l1 * m1 * sin(x[2]) * x[6] * x[6] * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) +
       1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
           (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -2) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) -
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) -
       (-l1 * u[0] * cos(x[2]) - l1 * u[1] * sin(x[2])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) +
       (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
       (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) -
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l1 * m1 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l1 * m1 *
                (1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]));
  Fx[31] =
      dt *
      (-1.0 * l2 * m2 * sin(x[3]) * x[7] * x[7] * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) +
       1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) -
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                       (m0 + m1 + m2) -
                   1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                       (m0 + m1 + m2),
               2) *
           (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -2) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) -
       (-l1 * u[0] * sin(x[2]) + l1 * u[1] * cos(x[2])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       (-l2 * u[2] * sin(x[3]) + l2 * u[3] * cos(x[3])) *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) +
       (-l2 * u[2] * cos(x[3]) - l2 * u[3] * sin(x[3])) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
       (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
        1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           (-2.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) / (m0 + m1 + m2) +
            2.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) / (m0 + m1 + m2)) *
           1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           pow(m2 * (l2 * l2) -
                   pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                               (m0 + m1 + m2) -
                           1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                               (m0 + m1 + m2),
                       2) *
                       1.0 /
                       (m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2)) -
                   1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2),
               -2) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * cos(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * cos(x[3]) * (x[7] * x[7]) + u[0] + u[2]) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2) +
                 1.0 * l1 * l2 * m1 * m2 * sin(x[3]) * cos(x[2]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 * sin(x[2]) * (x[6] * x[6]) +
            1.0 * l2 * m2 * sin(x[3]) * (x[7] * x[7]) + u[1] + u[3]));
  Fx[55] =
      dt *
      (2.0 * l1 * m1 * x[6] * sin(x[2]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) +
       2.0 * l1 * m1 * x[6] * cos(x[2]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Fx[63] =
      dt * (2.0 * l2 * m2 * x[7] * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) +
            2.0 * l2 * m2 * x[7] * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) +
      1;
  Fu[4] =
      dt *
      (-l1 * sin(x[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (1.0 * l1 * m1 * sin(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) +
        1.0 * l2 * m2 * sin(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (-1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 sin(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) +
             1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2) +
        pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fu[12] =
      dt *
      (l1 * cos(x[2]) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (-1.0 * l1 * m1 * cos(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) +
        1.0 * l2 * m2 * sin(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 cos(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) -
             1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fu[20] =
      dt *
      (-l2 * sin(x[3]) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (1.0 * l1 * m1 * sin(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) +
        1.0 * l2 * m2 * sin(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (-1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 sin(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) +
             1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2) +
        pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fu[28] =
      dt *
      (l2 * cos(x[3]) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (1.0 * l1 * m1 * sin(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (-1.0 * l1 * m1 * cos(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) +
        1.0 * l2 * m2 * sin(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 cos(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) -
             1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fu[5] =
      dt *
      (-l1 * sin(x[2]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-1.0 * l1 * m1 * cos(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (1.0 * l1 * m1 * sin(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) -
        1.0 * l2 * m2 * cos(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (-1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 sin(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) +
             1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fu[13] =
      dt *
      (l1 * cos(x[2]) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     pow(m1 * (l1 * l1) -
                             1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2) -
                             1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                                 (m0 + m1 + m2),
                         -3.0 / 2.0) *
                     1.0 /
                     (m2 * (l2 * l2) -
                      pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                                  (m0 + m1 + m2) -
                              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                                  (m0 + m1 + m2),
                          2) *
                          1.0 /
                          (m1 * (l1 * l1) -
                           1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2) -
                           1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                               (m0 + m1 + m2)) -
                      1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                          (m0 + m1 + m2)) +
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0)) /
                sqrt(m0 + m1 + m2) +
            1.0 * l2 * m2 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[3]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-1.0 * l1 * m1 * cos(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (-1.0 * l1 * m1 * cos(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) -
        1.0 * l2 * m2 * cos(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 cos(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) -
             1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2) +
        pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fu[21] =
      dt *
      (-l2 * sin(x[3]) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-1.0 * l1 * m1 * cos(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (1.0 * l1 * m1 * sin(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (-1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      sin(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) +
                  1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) -
        1.0 * l2 * m2 * cos(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (-1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 sin(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) +
             1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2));
  Fu[29] =
      dt *
      (l2 * cos(x[3]) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) * 1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) /
                sqrt(m0 + m1 + m2)) /
           sqrt(m0 + m1 + m2) +
       (-1.0 * l1 * m1 * cos(x[2]) *
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0) *
            (-1.0 * l1 * m1 * cos(x[2]) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) /
                 (m0 + m1 + m2) -
             (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                  (m0 + m1 + m2) -
              1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                  (m0 + m1 + m2)) *
                 pow(m1 * (l1 * l1) -
                         1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2) -
                         1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                             (m0 + m1 + m2),
                     -1.0 / 2.0) *
                 1.0 /
                 (m2 * (l2 * l2) -
                  pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                              (m0 + m1 + m2) -
                          1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                              (m0 + m1 + m2),
                      2) *
                      1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) -
                  1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 /
                      (m0 + m1 + m2)) *
                 (1.0 * l1 * m1 *
                      (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                           (m0 + m1 + m2) -
                       1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                           (m0 + m1 + m2)) *
                      cos(x[2]) * 1.0 /
                      (m1 * (l1 * l1) -
                       1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2) -
                       1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                           (m0 + m1 + m2)) /
                      (m0 + m1 + m2) -
                  1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))) /
            sqrt(m0 + m1 + m2) -
        1.0 * l2 * m2 * cos(x[3]) * 1.0 /
            (m2 * (l2 * l2) -
             pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                         (m0 + m1 + m2) -
                     1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                         (m0 + m1 + m2),
                 2) *
                 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) -
             1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
             1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
            (1.0 * l1 * m1 *
                 (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                      (m0 + m1 + m2) -
                  1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                      (m0 + m1 + m2)) *
                 cos(x[2]) * 1.0 /
                 (m1 * (l1 * l1) -
                  1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                  1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                      (m0 + m1 + m2)) /
                 (m0 + m1 + m2) -
             1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)) /
            sqrt(m0 + m1 + m2) +
        pow(m0 + m1 + m2, -1.0 / 2.0)) /
           sqrt(m0 + m1 + m2));
  Fu[6] =
      dt *
      (-l1 * sin(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))));
  Fu[14] =
      dt *
      (l1 * cos(x[2]) *
           pow(m1 * (l1 * l1) -
                   1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                       (m0 + m1 + m2) -
                   1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
               -1.0 / 2.0) *
           (pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -3.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
            pow(m1 * (l1 * l1) -
                    1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2) -
                    1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                        (m0 + m1 + m2),
                -1.0 / 2.0)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))));
  Fu[22] =
      dt *
      (l2 *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           sin(x[3]) * 1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (1.0 * l1 * m1 * sin(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (-1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     sin(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) +
                 1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2))));
  Fu[30] =
      dt *
      (-l2 *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           cos(x[3]) * 1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       pow(m1 * (l1 * l1) -
               1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
               1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2),
           -1.0 / 2.0) *
           (-1.0 * l1 * m1 * cos(x[2]) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) /
                (m0 + m1 + m2) -
            (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
             1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
                pow(m1 * (l1 * l1) -
                        1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2) -
                        1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                            (m0 + m1 + m2),
                    -1.0 / 2.0) *
                1.0 /
                (m2 * (l2 * l2) -
                 pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                             (m0 + m1 + m2) -
                         1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                             (m0 + m1 + m2),
                     2) *
                     1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) -
                 1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
                (1.0 * l1 * m1 *
                     (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                          (m0 + m1 + m2) -
                      1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                          (m0 + m1 + m2)) *
                     cos(x[2]) * 1.0 /
                     (m1 * (l1 * l1) -
                      1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2) -
                      1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 /
                          (m0 + m1 + m2)) /
                     (m0 + m1 + m2) -
                 1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2))));
  Fu[7] =
      dt *
      (l1 *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           sin(x[2]) * 1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Fu[15] =
      dt *
      (-l1 *
           (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) / (m0 + m1 + m2) -
            1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) / (m0 + m1 + m2)) *
           cos(x[2]) * 1.0 /
           (m1 * (l1 * l1) -
            1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) *
           1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)));
  Fu[23] =
      dt *
      (-l2 * sin(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (-1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                sin(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) +
            1.0 * l2 * m2 * sin(x[3]) / (m0 + m1 + m2)));
  Fu[31] =
      dt *
      (l2 * cos(x[3]) * 1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) +
       1.0 /
           (m2 * (l2 * l2) -
            pow(-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                        (m0 + m1 + m2) -
                    1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                        (m0 + m1 + m2),
                2) *
                1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) -
            1.0 * pow(sin(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2) -
            1.0 * pow(cos(x[3]), 2) * l2 * l2 * m2 * m2 / (m0 + m1 + m2)) *
           (1.0 * l1 * m1 *
                (-1.0 * l1 * l2 * m1 * m2 * sin(x[2]) * sin(x[3]) /
                     (m0 + m1 + m2) -
                 1.0 * l1 * l2 * m1 * m2 * cos(x[2]) * cos(x[3]) /
                     (m0 + m1 + m2)) *
                cos(x[2]) * 1.0 /
                (m1 * (l1 * l1) -
                 1.0 * pow(sin(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2) -
                 1.0 * pow(cos(x[2]), 2) * l1 * l1 * m1 * m1 / (m0 + m1 + m2)) /
                (m0 + m1 + m2) -
            1.0 * l2 * m2 * cos(x[3]) / (m0 + m1 + m2)));
}

} // namespace dynobench