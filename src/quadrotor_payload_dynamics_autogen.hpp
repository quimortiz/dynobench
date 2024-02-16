void inline calcFF(Eigen::Ref<Eigen::VectorXd> ff, const double *data,
                   const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &u) {
  // Auto generated 2023-08-24--09-18-48 from sympy
  const double m = data[0];
  const double mp = data[1];
  const double J_v0 = data[2];
  const double J_v1 = data[3];
  const double J_v2 = data[4];
  const double t2t = data[5];
  const double l = data[6];
  const double arm_length = data[7];

  Eigen::Vector3d pos = x.head(3).head<3>();
  Eigen::Vector3d qc = x.segment(3, 3).head<3>();
  Eigen::Vector3d vel = x.segment(6, 3).head<3>();
  Eigen::Vector3d wc = x.segment(9, 3).head<3>();
  Eigen::Vector4d q = x.segment(12, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
  Eigen::Vector3d w = x.segment(16, 3).head<3>();
  ff(0) = vel(0);
  ff(1) = vel(1);
  ff(2) = vel(2);
  ff(3) = (-qc(1) * wc(2) + qc(2) * wc(1)) /
          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  ff(4) = (qc(0) * wc(2) - qc(2) * wc(0)) /
          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  ff(5) = (-qc(0) * wc(1) + qc(1) * wc(0)) /
          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  ff(6) = qc(0) *
          (-l * m * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
           2.4525000000000001 * (m + mp) *
               sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
                2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
                qc(2) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2))) *
               (u(0) + u(1) + u(2) + u(3))) /
          ((m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(7) = qc(1) *
          (-l * m * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
           2.4525000000000001 * (m + mp) *
               sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
                2 * qc(1) * (-q(0) * q(3) + q(1) * q(2)) -
                qc(2) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2))) *
               (u(0) + u(1) + u(2) + u(3))) /
          ((m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(8) = (-l * m * qc(2) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
           2.4525000000000001 * qc(2) * (m + mp) *
               sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
                2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
                qc(2) * (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2))) *
               (u(0) + u(1) + u(2) + u(3)) -
           9.8100000000000005 * (m + mp) *
               pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
          ((m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(9) = 2.4525000000000001 * (m + mp) *
          (qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
           2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) *
          (u(0) + u(1) + u(2) + u(3)) /
          (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(10) =
      -2.4525000000000001 * (m + mp) *
      (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(11) = 4.9050000000000002 * (m + mp) *
           (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
            qc(1) * (q(0) * q(2) + q(1) * q(3))) *
           (u(0) + u(1) + u(2) + u(3)) /
           (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
            (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  ff(12) = (1.0 / 2.0) * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) /
           sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  ff(13) = (1.0 / 2.0) * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) /
           sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  ff(14) = (1.0 / 2.0) * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) /
           sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  ff(15) = -1.0 / 2.0 * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) /
           sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  ff(16) = (J_v1 * w(1) * w(2) - J_v2 * w(1) * w(2) -
            1.7341793804025001 * arm_length * u(0) * (m + mp) -
            1.7341793804025001 * arm_length * u(1) * (m + mp) +
            1.7341793804025001 * arm_length * u(2) * (m + mp) +
            1.7341793804025001 * arm_length * u(3) * (m + mp)) /
           J_v0;
  ff(17) = (-J_v0 * w(0) * w(2) + J_v2 * w(0) * w(2) -
            1.7341793804025001 * arm_length * u(0) * (m + mp) +
            1.7341793804025001 * arm_length * u(1) * (m + mp) +
            1.7341793804025001 * arm_length * u(2) * (m + mp) -
            1.7341793804025001 * arm_length * u(3) * (m + mp)) /
           J_v1;
  ff(18) = (J_v0 * w(0) * w(1) - J_v1 * w(0) * w(1) -
            2.4525000000000001 * t2t * u(0) * (m + mp) +
            2.4525000000000001 * t2t * u(1) * (m + mp) -
            2.4525000000000001 * t2t * u(2) * (m + mp) +
            2.4525000000000001 * t2t * u(3) * (m + mp)) /
           J_v2;
}

void inline calcJ(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                  Eigen::Ref<Eigen::MatrixXd> Jv_u, const double *data,
                  const Eigen::Ref<const Eigen::VectorXd> &x,
                  const Eigen::Ref<const Eigen::VectorXd> &u) {
  const double m = data[0];
  const double mp = data[1];
  const double J_v0 = data[2];
  const double J_v1 = data[3];
  const double J_v2 = data[4];
  const double t2t = data[5];
  const double l = data[6];
  const double arm_length = data[7];

  Eigen::Vector3d pos = x.head(3).head<3>();
  Eigen::Vector3d qc = x.segment(3, 3).head<3>();
  Eigen::Vector3d vel = x.segment(6, 3).head<3>();
  Eigen::Vector3d wc = x.segment(9, 3).head<3>();
  Eigen::Vector4d q = x.segment(12, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
  Eigen::Vector3d w = x.segment(16, 3).head<3>();
  Jv_x(0, 6) = 1;
  Jv_x(1, 7) = 1;
  Jv_x(2, 8) = 1;
  Jv_x(3, 3) = qc(0) * (qc(1) * wc(2) - qc(2) * wc(1)) /
               pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(3, 4) =
      -(pow(qc(0), 2) * wc(2) + qc(1) * qc(2) * wc(1) + pow(qc(2), 2) * wc(2)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(3, 5) =
      (pow(qc(0), 2) * wc(1) + pow(qc(1), 2) * wc(1) + qc(1) * qc(2) * wc(2)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(3, 10) = qc(2) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(3, 11) = -qc(1) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(4, 3) =
      (qc(0) * qc(2) * wc(0) + pow(qc(1), 2) * wc(2) + pow(qc(2), 2) * wc(2)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(4, 4) = qc(1) * (-qc(0) * wc(2) + qc(2) * wc(0)) /
               pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(4, 5) =
      -(pow(qc(0), 2) * wc(0) + qc(0) * qc(2) * wc(2) + pow(qc(1), 2) * wc(0)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(4, 9) = -qc(2) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(4, 11) = qc(0) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(5, 3) =
      -(qc(0) * qc(1) * wc(0) + pow(qc(1), 2) * wc(1) + pow(qc(2), 2) * wc(1)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(5, 4) =
      (pow(qc(0), 2) * wc(0) + qc(0) * qc(1) * wc(1) + pow(qc(2), 2) * wc(0)) /
      pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(5, 5) = qc(2) * (qc(0) * wc(1) - qc(1) * wc(0)) /
               pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0);
  Jv_x(5, 9) = qc(1) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(5, 10) = -qc(0) / sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2));
  Jv_x(6, 3) =
      (l * m * pow(qc(0), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(0), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (4 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(6, 4) =
      qc(0) *
      (l * m * qc(1) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(1) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       4.9050000000000002 * (m + mp) * (q(0) * q(3) - q(1) * q(2)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(6, 5) =
      qc(0) *
      (l * m * qc(2) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(6, 9) = -2 * l * m * qc(0) * wc(0) /
               ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(6, 10) =
      -2 * l * m * qc(0) * wc(1) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(6, 11) =
      -2 * l * m * qc(0) * wc(2) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(6, 12) =
      4.9050000000000002 * qc(0) *
      (-2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (-2 * q(0) * (q(0) * q(2) + q(1) * q(3)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (-q(0) * q(3) + q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(6, 13) =
      4.9050000000000002 * qc(0) *
      (-2 * q(1) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (-2 * q(1) * (q(0) * q(2) + q(1) * q(3)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(6, 14) =
      4.9050000000000002 * qc(0) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(6, 15) =
      4.9050000000000002 * qc(0) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(3) * (-q(0) * q(3) + q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(7, 3) =
      qc(1) *
      (l * m * qc(0) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(0) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) * (q(0) * q(2) + q(1) * q(3)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(7, 4) =
      (l * m * pow(qc(1), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(1), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            4 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(7, 5) =
      qc(1) *
      (l * m * qc(2) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(7, 9) = -2 * l * m * qc(1) * wc(0) /
               ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(7, 10) =
      -2 * l * m * qc(1) * wc(1) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(7, 11) =
      -2 * l * m * qc(1) * wc(2) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(7, 12) =
      -4.9050000000000002 * qc(1) *
      (2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(3) - q(1) * q(2)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(7, 13) =
      4.9050000000000002 * qc(1) *
      (-2 * q(1) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) -
       qc(0) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(7, 14) =
      4.9050000000000002 * qc(1) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(7, 15) =
      4.9050000000000002 * qc(1) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(3) * (-q(0) * q(3) + q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(8, 3) =
      qc(2) *
      (l * m * qc(0) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(0) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) * (q(0) * q(2) + q(1) * q(3)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(8, 4) =
      qc(2) *
      (l * m * qc(1) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       4.9050000000000002 * qc(1) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       4.9050000000000002 * (m + mp) * (q(0) * q(3) - q(1) * q(2)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(8, 5) =
      (l * m * pow(qc(2), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(2), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(8, 9) = -2 * l * m * qc(2) * wc(0) /
               ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(8, 10) =
      -2 * l * m * qc(2) * wc(1) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(8, 11) =
      -2 * l * m * qc(2) * wc(2) /
      ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Jv_x(8, 12) =
      -4.9050000000000002 * qc(2) *
      (2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (-2 * q(0) * (q(0) * q(3) - q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(8, 13) =
      4.9050000000000002 * qc(2) *
      (2 * q(1) * qc(2) * (-pow(q(2), 2) - pow(q(3), 2)) -
       qc(0) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(8, 14) =
      4.9050000000000002 * qc(2) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(8, 15) =
      4.9050000000000002 * qc(2) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(3) * (-q(0) * q(3) + q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(9, 3) =
      2.4525000000000001 * qc(0) * (m + mp) *
      (-qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(9, 4) =
      2.4525000000000001 * (m + mp) *
      (qc(1) * (qc(1) * (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) +
       (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(9, 5) =
      2.4525000000000001 * (m + mp) *
      (qc(2) * (-qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                          pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) +
       2 * (-q(0) * q(3) + q(1) * q(2)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(9, 12) =
      4.9050000000000002 * (m + mp) *
      (2 * q(0) * qc(1) * (pow(q(2), 2) + pow(q(3), 2)) -
       qc(2) * (2 * q(0) * (-q(0) * q(3) + q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(9, 13) =
      4.9050000000000002 * (m + mp) *
      (2 * q(1) * qc(1) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(9, 14) =
      4.9050000000000002 * (m + mp) *
      (-2 * q(2) * qc(1) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(9, 15) =
      -4.9050000000000002 * (m + mp) *
      (2 * q(3) * qc(1) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(10, 3) =
      2.4525000000000001 * (m + mp) *
      (qc(0) * (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) +
       (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(10, 4) =
      2.4525000000000001 * qc(1) * (m + mp) *
      (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(10, 5) =
      2.4525000000000001 * (m + mp) *
      (qc(2) * (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) -
       2 * (q(0) * q(2) + q(1) * q(3)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(10, 12) =
      4.9050000000000002 * (m + mp) *
      (-2 * q(0) * qc(0) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(10, 13) =
      4.9050000000000002 * (m + mp) *
      (-2 * q(1) * qc(0) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(10, 14) =
      4.9050000000000002 * (m + mp) *
      (2 * q(2) * qc(0) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(10, 15) =
      4.9050000000000002 * (m + mp) *
      (2 * q(3) * qc(0) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(11, 3) =
      -4.9050000000000002 * (m + mp) *
      (qc(0) * (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                qc(1) * (q(0) * q(2) + q(1) * q(3))) -
       (q(0) * q(3) - q(1) * q(2)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(11, 4) =
      4.9050000000000002 * (m + mp) *
      (-qc(1) * (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) +
       (q(0) * q(2) + q(1) * q(3)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(11, 5) =
      -4.9050000000000002 * qc(2) * (m + mp) *
      (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
       qc(1) * (q(0) * q(2) + q(1) * q(3))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_x(11, 12) =
      4.9050000000000002 * (m + mp) *
      (qc(0) * (2 * q(0) * (-q(0) * q(3) + q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(11, 13) =
      -4.9050000000000002 * (m + mp) *
      (qc(0) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(11, 14) =
      -4.9050000000000002 * (m + mp) *
      (qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(2) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(11, 15) =
      4.9050000000000002 * (m + mp) *
      (qc(0) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(3) * (-q(0) * q(3) + q(1) * q(2))) -
       qc(1) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Jv_x(12, 12) =
      (1.0 / 2.0) * q(0) * (q(1) * w(2) - q(2) * w(1) - q(3) * w(0)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(12, 13) =
      (1.0 / 2.0) *
      (q(1) * (q(1) * w(2) - q(2) * w(1) - q(3) * w(0)) -
       w(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(12, 14) =
      (1.0 / 2.0) *
      (pow(q(0), 2) * w(1) + pow(q(1), 2) * w(1) + q(1) * q(2) * w(2) -
       q(2) * q(3) * w(0) + pow(q(3), 2) * w(1)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(12, 15) =
      (1.0 / 2.0) *
      (pow(q(0), 2) * w(0) + pow(q(1), 2) * w(0) + q(1) * q(3) * w(2) +
       pow(q(2), 2) * w(0) - q(2) * q(3) * w(1)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(12, 16) =
      (1.0 / 2.0) * q(3) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(12, 17) =
      (1.0 / 2.0) * q(2) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(12, 18) =
      -1.0 / 2.0 * q(1) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(13, 12) =
      (1.0 / 2.0) *
      (q(0) * q(2) * w(0) - q(0) * q(3) * w(1) + pow(q(1), 2) * w(2) +
       pow(q(2), 2) * w(2) + pow(q(3), 2) * w(2)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(13, 13) =
      (1.0 / 2.0) * q(1) * (-q(0) * w(2) + q(2) * w(0) - q(3) * w(1)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(13, 14) =
      (1.0 / 2.0) *
      (q(2) * (-q(0) * w(2) + q(2) * w(0) - q(3) * w(1)) -
       w(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(13, 15) =
      (1.0 / 2.0) *
      (pow(q(0), 2) * w(1) - q(0) * q(3) * w(2) + pow(q(1), 2) * w(1) +
       pow(q(2), 2) * w(1) + q(2) * q(3) * w(0)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(13, 16) =
      -1.0 / 2.0 * q(2) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(13, 17) =
      (1.0 / 2.0) * q(3) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(13, 18) =
      (1.0 / 2.0) * q(0) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(14, 12) =
      (1.0 / 2.0) *
      (q(0) * (q(0) * w(1) - q(1) * w(0) - q(3) * w(2)) -
       w(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(14, 13) =
      (1.0 / 2.0) *
      (pow(q(0), 2) * w(0) + q(0) * q(1) * w(1) - q(1) * q(3) * w(2) +
       pow(q(2), 2) * w(0) + pow(q(3), 2) * w(0)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(14, 14) =
      (1.0 / 2.0) * q(2) * (q(0) * w(1) - q(1) * w(0) - q(3) * w(2)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(14, 15) =
      (1.0 / 2.0) *
      (pow(q(0), 2) * w(2) + q(0) * q(3) * w(1) + pow(q(1), 2) * w(2) -
       q(1) * q(3) * w(0) + pow(q(2), 2) * w(2)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(14, 16) =
      (1.0 / 2.0) * q(1) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(14, 17) =
      -1.0 / 2.0 * q(0) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(14, 18) =
      (1.0 / 2.0) * q(3) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(15, 12) =
      (1.0 / 2.0) *
      (q(0) * q(1) * w(1) + q(0) * q(2) * w(2) - pow(q(1), 2) * w(0) -
       pow(q(2), 2) * w(0) - pow(q(3), 2) * w(0)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(15, 13) =
      (1.0 / 2.0) *
      (-pow(q(0), 2) * w(1) + q(0) * q(1) * w(0) + q(1) * q(2) * w(2) -
       pow(q(2), 2) * w(1) - pow(q(3), 2) * w(1)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(15, 14) =
      (1.0 / 2.0) *
      (-pow(q(0), 2) * w(2) + q(0) * q(2) * w(0) - pow(q(1), 2) * w(2) +
       q(1) * q(2) * w(1) - pow(q(3), 2) * w(2)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(15, 15) =
      (1.0 / 2.0) * q(3) * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) /
      pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 3.0 / 2.0);
  Jv_x(15, 16) =
      -1.0 / 2.0 * q(0) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(15, 17) =
      -1.0 / 2.0 * q(1) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(15, 18) =
      -1.0 / 2.0 * q(2) /
      sqrt(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2));
  Jv_x(16, 17) = w(2) * (J_v1 - J_v2) / J_v0;
  Jv_x(16, 18) = w(1) * (J_v1 - J_v2) / J_v0;
  Jv_x(17, 16) = w(2) * (-J_v0 + J_v2) / J_v1;
  Jv_x(17, 18) = w(0) * (-J_v0 + J_v2) / J_v1;
  Jv_x(18, 16) = w(1) * (J_v0 - J_v1) / J_v2;
  Jv_x(18, 17) = w(0) * (J_v0 - J_v1) / J_v2;

  Jv_u(6, 0) =
      qc(0) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(6, 1) =
      qc(0) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(6, 2) =
      qc(0) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(6, 3) =
      qc(0) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(7, 0) =
      qc(1) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
       4.9050000000000002 * qc(1) * (-q(0) * q(3) + q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(7, 1) =
      qc(1) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
       4.9050000000000002 * qc(1) * (-q(0) * q(3) + q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(7, 2) =
      qc(1) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
       4.9050000000000002 * qc(1) * (-q(0) * q(3) + q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(7, 3) =
      qc(1) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
       4.9050000000000002 * qc(1) * (-q(0) * q(3) + q(1) * q(2)) -
       2.4525000000000001 * qc(2) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(8, 0) =
      qc(2) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
       2.4525000000000001 * qc(2) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(8, 1) =
      qc(2) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
       2.4525000000000001 * qc(2) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(8, 2) =
      qc(2) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
       2.4525000000000001 * qc(2) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(8, 3) =
      qc(2) *
      (4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
       4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
       2.4525000000000001 * qc(2) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(9, 0) =
      (m + mp) *
      (2.4525000000000001 * qc(1) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(9, 1) =
      (m + mp) *
      (2.4525000000000001 * qc(1) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(9, 2) =
      (m + mp) *
      (2.4525000000000001 * qc(1) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(9, 3) =
      (m + mp) *
      (2.4525000000000001 * qc(1) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
       4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(10, 0) =
      -(m + mp) *
      (2.4525000000000001 * qc(0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(10, 1) =
      -(m + mp) *
      (2.4525000000000001 * qc(0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(10, 2) =
      -(m + mp) *
      (2.4525000000000001 * qc(0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(10, 3) =
      -(m + mp) *
      (2.4525000000000001 * qc(0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(11, 0) = 4.9050000000000002 * (m + mp) *
                (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) /
                (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                 (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(11, 1) = 4.9050000000000002 * (m + mp) *
                (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) /
                (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                 (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(11, 2) = 4.9050000000000002 * (m + mp) *
                (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) /
                (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                 (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(11, 3) = 4.9050000000000002 * (m + mp) *
                (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) /
                (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                 (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Jv_u(16, 0) = -1.7341793804025001 * arm_length * (m + mp) / J_v0;
  Jv_u(16, 1) = -1.7341793804025001 * arm_length * (m + mp) / J_v0;
  Jv_u(16, 2) = 1.7341793804025001 * arm_length * (m + mp) / J_v0;
  Jv_u(16, 3) = 1.7341793804025001 * arm_length * (m + mp) / J_v0;
  Jv_u(17, 0) = -1.7341793804025001 * arm_length * (m + mp) / J_v1;
  Jv_u(17, 1) = 1.7341793804025001 * arm_length * (m + mp) / J_v1;
  Jv_u(17, 2) = 1.7341793804025001 * arm_length * (m + mp) / J_v1;
  Jv_u(17, 3) = -1.7341793804025001 * arm_length * (m + mp) / J_v1;
  Jv_u(18, 0) = -2.4525000000000001 * t2t * (m + mp) / J_v2;
  Jv_u(18, 1) = 2.4525000000000001 * t2t * (m + mp) / J_v2;
  Jv_u(18, 2) = -2.4525000000000001 * t2t * (m + mp) / J_v2;
  Jv_u(18, 3) = 2.4525000000000001 * t2t * (m + mp) / J_v2;
}

void inline calcF(Eigen::Ref<Eigen::MatrixXd> Fx,
                  Eigen::Ref<Eigen::MatrixXd> Fu, const double *data,
                  const Eigen::Ref<const Eigen::VectorXd> &x,
                  const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {
  const double m = data[0];
  const double mp = data[1];
  const double J_v0 = data[2];
  const double J_v1 = data[3];
  const double J_v2 = data[4];
  const double t2t = data[5];
  const double l = data[6];
  const double arm_length = data[7];

  Eigen::Vector3d pos = x.head(3).head<3>();
  Eigen::Vector3d qc = x.segment(3, 3).head<3>();
  Eigen::Vector3d vel = x.segment(6, 3).head<3>();
  Eigen::Vector3d wc = x.segment(9, 3).head<3>();
  Eigen::Vector4d q = x.segment(12, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
  Eigen::Vector3d w = x.segment(16, 3).head<3>();
  Fx(0, 0) = 1;
  Fx(0, 6) = dt;
  Fx(1, 1) = 1;
  Fx(1, 7) = dt;
  Fx(2, 2) = 1;
  Fx(2, 8) = dt;
  Fx(3, 3) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      ((dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
        qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(1) + qc(1) * wc(0)) +
                      wc(1) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(2) + qc(2) * wc(0)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) -
            (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
             qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(0) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0))) +
       (dt * qc(0) * (qc(1) * wc(2) - qc(2) * wc(1)) +
        pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0)) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(3, 4) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-dt *
           (qc(1) * (-qc(1) * wc(2) + qc(2) * wc(1)) +
            wc(2) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
        qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                (-(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) *
                       sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (-qc(1) * wc(2) + qc(2) * wc(1)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) -
            (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
             qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(1) * (qc(0) * wc(2) - qc(2) * wc(0)) -
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(3, 5) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (dt *
           (qc(2) * (qc(1) * wc(2) - qc(2) * wc(1)) +
            wc(1) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) -
       (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
        qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                      wc(1) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) +
            (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
             qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(2) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(3, 9) =
      dt *
      (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
       qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (qc(1) * (-dt * (qc(0) * wc(1) - qc(1) * wc(0)) +
                qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) -
       qc(2) * (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) /
      (pow((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
               (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
           3.0 / 2.0) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0));
  Fx(3, 10) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (qc(2) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
        qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(0) *
                (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                 qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) -
            qc(2) * (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                     qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(3, 11) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-qc(1) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
        qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(0) *
                (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                 qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
            qc(1) * (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                     qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(4, 3) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (dt *
           (qc(0) * (-qc(0) * wc(2) + qc(2) * wc(0)) +
            wc(2) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) -
       (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
        qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(1) + qc(1) * wc(0)) +
                      wc(1) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(2) + qc(2) * wc(0)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) -
            (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
             qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(0) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(4, 4) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
         qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                (-(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) *
                       sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (-qc(1) * wc(2) + qc(2) * wc(1)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) -
            (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
             qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(1) * (qc(0) * wc(2) - qc(2) * wc(0)) -
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0))) +
       (-dt * qc(1) * (qc(0) * wc(2) - qc(2) * wc(0)) +
        pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0)) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(4, 5) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-dt *
           (qc(2) * (qc(0) * wc(2) - qc(2) * wc(0)) +
            wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
        qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                      wc(1) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) +
            (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
             qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(2) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(4, 9) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-qc(2) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
        qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(1) *
                (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                 qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
            qc(2) * (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                     qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(4, 10) =
      dt *
      (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
       qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-qc(0) * (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                 qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
       qc(2) * (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) /
      (pow((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
               (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
           3.0 / 2.0) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0));
  Fx(4, 11) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (qc(0) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) -
       (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
        qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(0) *
                (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                 qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
            qc(1) * (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                     qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(5, 3) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-dt *
           (qc(0) * (-qc(0) * wc(1) + qc(1) * wc(0)) +
            wc(1) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
        qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(1) + qc(1) * wc(0)) +
                      wc(1) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(0) * (-qc(0) * wc(2) + qc(2) * wc(0)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) -
            (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
             qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(0) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(5, 4) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (dt *
           (qc(1) * (qc(0) * wc(1) - qc(1) * wc(0)) +
            wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) -
       (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
        qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) -
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(1) * (-qc(1) * wc(2) + qc(2) * wc(1)) +
                      wc(2) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) +
            (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
             qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(1) * (qc(0) * wc(2) - qc(2) * wc(0)) -
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0)))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(5, 5) =
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
         qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (dt *
                ((dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                  qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      wc(0) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
                 (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                  qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                     (qc(2) * (qc(1) * wc(2) - qc(2) * wc(1)) +
                      wc(1) *
                          (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) +
            (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
             qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
                (dt * qc(2) * (qc(0) * wc(1) - qc(1) * wc(0)) +
                 pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2),
                     3.0 / 2.0))) +
       (dt * qc(2) * (qc(0) * wc(1) - qc(1) * wc(0)) +
        pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0)) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2))) /
      (sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                   qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
               2) +
               pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                       qc(1) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2) +
               pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                       qc(0) *
                           sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                   2),
           2));
  Fx(5, 9) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (qc(1) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) -
       (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
        qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(1) *
                (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                 qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) +
            qc(2) * (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                     qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(5, 10) =
      dt *
      sqrt((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (-qc(0) *
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) +
       (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
        qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
           (qc(0) *
                (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                 qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) -
            qc(2) * (dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                     qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) +
                                  pow(qc(2), 2))))) *
      sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) /
      pow(pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                  qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
              2) +
              pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                      qc(1) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2) +
              pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                      qc(0) *
                          sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                  2),
          2);
  Fx(5, 11) =
      dt *
      (dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
       qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (qc(0) * (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) -
       qc(1) * (-dt * (qc(1) * wc(2) - qc(2) * wc(1)) +
                qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)))) /
      (pow((pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
               (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
           3.0 / 2.0) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0));
  Fx(6, 3) =
      dt *
      (l * m * pow(qc(0), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(0), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (4 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(6, 4) =
      dt * qc(0) *
      (l * m * qc(1) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(1) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       4.9050000000000002 * (m + mp) * (q(0) * q(3) - q(1) * q(2)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(6, 5) =
      dt * qc(0) *
      (l * m * qc(2) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(6, 6) = 1;
  Fx(6, 9) = -2 * dt * l * m * qc(0) * wc(0) /
             ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(6, 10) = -2 * dt * l * m * qc(0) * wc(1) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(6, 11) = -2 * dt * l * m * qc(0) * wc(2) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(6, 12) =
      -4.9050000000000002 * dt * qc(0) *
      (2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(3) - q(1) * q(2)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(6, 13) =
      -4.9050000000000002 * dt * qc(0) *
      (2 * q(1) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(6, 14) =
      4.9050000000000002 * dt * qc(0) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(6, 15) =
      4.9050000000000002 * dt * qc(0) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(7, 3) =
      dt * qc(1) *
      (l * m * qc(0) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(0) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) * (q(0) * q(2) + q(1) * q(3)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(7, 4) =
      dt *
      (l * m * pow(qc(1), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(1), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            4 * qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(7, 5) =
      dt * qc(1) *
      (l * m * qc(2) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       2.4525000000000001 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(7, 7) = 1;
  Fx(7, 9) = -2 * dt * l * m * qc(1) * wc(0) /
             ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(7, 10) = -2 * dt * l * m * qc(1) * wc(1) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(7, 11) = -2 * dt * l * m * qc(1) * wc(2) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(7, 12) =
      -4.9050000000000002 * dt * qc(1) *
      (2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(3) - q(1) * q(2)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(7, 13) =
      -4.9050000000000002 * dt * qc(1) *
      (2 * q(1) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(7, 14) =
      4.9050000000000002 * dt * qc(1) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(7, 15) =
      4.9050000000000002 * dt * qc(1) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(8, 3) =
      dt * qc(2) *
      (l * m * qc(0) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(0) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) * (q(0) * q(2) + q(1) * q(3)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(8, 4) =
      dt * qc(2) *
      (l * m * qc(1) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * qc(1) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5.0 / 2.0) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) -
       4.9050000000000002 * (m + mp) * (q(0) * q(3) - q(1) * q(2)) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(8, 5) =
      dt *
      (l * m * pow(qc(2), 2) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 7.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
       l * m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 9.0 / 2.0) *
           (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
       4.9050000000000002 * pow(qc(2), 2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3) *
           (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
            2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3)) +
       4.9050000000000002 * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 4) *
           (qc(0) * (q(0) * q(2) + q(1) * q(3)) -
            qc(1) * (q(0) * q(3) - q(1) * q(2)) -
            qc(2) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
           (u(0) + u(1) + u(2) + u(3))) /
      ((m + mp) * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 5) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(8, 8) = 1;
  Fx(8, 9) = -2 * dt * l * m * qc(2) * wc(0) /
             ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(8, 10) = -2 * dt * l * m * qc(2) * wc(1) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(8, 11) = -2 * dt * l * m * qc(2) * wc(2) /
              ((m + mp) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  Fx(8, 12) =
      -4.9050000000000002 * dt * qc(2) *
      (2 * q(0) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(3) - q(1) * q(2)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(8, 13) =
      -4.9050000000000002 * dt * qc(2) *
      (2 * q(1) * qc(2) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(0) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(8, 14) =
      4.9050000000000002 * dt * qc(2) *
      (2 * q(2) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(0) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3))) +
       qc(1) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(8, 15) =
      4.9050000000000002 * dt * qc(2) *
      (2 * q(3) * qc(2) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(2) + q(1) * q(3))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(9, 3) =
      2.4525000000000001 * dt * qc(0) * (m + mp) *
      (-qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(9, 4) =
      2.4525000000000001 * dt * (m + mp) *
      (qc(1) * (qc(1) * (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) +
       (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(9, 5) =
      2.4525000000000001 * dt * (m + mp) *
      (qc(2) * (-qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                          pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) +
       2 * (-q(0) * q(3) + q(1) * q(2)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(9, 9) = 1;
  Fx(9, 12) =
      4.9050000000000002 * dt * (m + mp) *
      (2 * q(0) * qc(1) * (pow(q(2), 2) + pow(q(3), 2)) -
       qc(2) * (2 * q(0) * (-q(0) * q(3) + q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(9, 13) =
      4.9050000000000002 * dt * (m + mp) *
      (2 * q(1) * qc(1) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(9, 14) =
      -4.9050000000000002 * dt * (m + mp) *
      (2 * q(2) * qc(1) * (pow(q(0), 2) + pow(q(1), 2)) -
       qc(2) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(9, 15) =
      -4.9050000000000002 * dt * (m + mp) *
      (2 * q(3) * qc(1) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(3) * (q(0) * q(3) - q(1) * q(2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(10, 3) =
      2.4525000000000001 * dt * (m + mp) *
      (qc(0) * (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) +
       (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (-pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(10, 4) =
      2.4525000000000001 * dt * qc(1) * (m + mp) *
      (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
       2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(10, 5) =
      2.4525000000000001 * dt * (m + mp) *
      (qc(2) * (qc(0) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                         pow(q(3), 2)) +
                2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) -
       2 * (q(0) * q(2) + q(1) * q(3)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(10, 10) = 1;
  Fx(10, 12) =
      4.9050000000000002 * dt * (m + mp) *
      (-2 * q(0) * qc(0) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(10, 13) =
      4.9050000000000002 * dt * (m + mp) *
      (-2 * q(1) * qc(0) * (pow(q(2), 2) + pow(q(3), 2)) +
       qc(2) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(10, 14) =
      4.9050000000000002 * dt * (m + mp) *
      (2 * q(2) * qc(0) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) * (-q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(2) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(10, 15) =
      4.9050000000000002 * dt * (m + mp) *
      (2 * q(3) * qc(0) * (pow(q(0), 2) + pow(q(1), 2)) +
       qc(2) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(11, 3) =
      -4.9050000000000002 * dt * (m + mp) *
      (qc(0) * (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                qc(1) * (q(0) * q(2) + q(1) * q(3))) -
       (q(0) * q(3) - q(1) * q(2)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(11, 4) =
      4.9050000000000002 * dt * (m + mp) *
      (-qc(1) * (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(1) * (q(0) * q(2) + q(1) * q(3))) +
       (q(0) * q(2) + q(1) * q(3)) *
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(11, 5) =
      -4.9050000000000002 * dt * qc(2) * (m + mp) *
      (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
       qc(1) * (q(0) * q(2) + q(1) * q(3))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fx(11, 11) = 1;
  Fx(11, 12) =
      4.9050000000000002 * dt * (m + mp) *
      (qc(0) * (2 * q(0) * (-q(0) * q(3) + q(1) * q(2)) +
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) -
       qc(1) * (2 * q(0) * (q(0) * q(2) + q(1) * q(3)) -
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(11, 13) =
      -4.9050000000000002 * dt * (m + mp) *
      (qc(0) * (2 * q(1) * (q(0) * q(3) - q(1) * q(2)) +
                q(2) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2))) +
       qc(1) * (2 * q(1) * (q(0) * q(2) + q(1) * q(3)) -
                q(3) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                        pow(q(3), 2)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(11, 14) =
      -4.9050000000000002 * dt * (m + mp) *
      (qc(0) *
           (q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(2) * (q(0) * q(3) - q(1) * q(2))) -
       qc(1) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) -
            2 * q(2) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(11, 15) =
      4.9050000000000002 * dt * (m + mp) *
      (qc(0) *
           (q(0) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2 * q(3) * (-q(0) * q(3) + q(1) * q(2))) -
       qc(1) * (-q(1) * (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) +
                         pow(q(3), 2)) +
                2 * q(3) * (q(0) * q(2) + q(1) * q(3)))) *
      (u(0) + u(1) + u(2) + u(3)) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       pow(pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2), 2));
  Fx(12, 12) =
      (dt * q(0) * q(1) * w(2) - dt * q(0) * q(2) * w(1) -
       dt * q(0) * q(3) * w(0) + 2 * pow(q(1), 2) + 2 * pow(q(2), 2) +
       2 * pow(q(3), 2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 13) =
      -(dt * pow(q(0), 2) * w(2) + dt * q(1) * q(2) * w(1) +
        dt * q(1) * q(3) * w(0) + dt * pow(q(2), 2) * w(2) +
        dt * pow(q(3), 2) * w(2) + 2 * q(0) * q(1)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 14) =
      (dt * pow(q(0), 2) * w(1) + dt * pow(q(1), 2) * w(1) +
       dt * q(1) * q(2) * w(2) - dt * q(2) * q(3) * w(0) +
       dt * pow(q(3), 2) * w(1) - 2 * q(0) * q(2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 15) =
      (dt * pow(q(0), 2) * w(0) + dt * pow(q(1), 2) * w(0) +
       dt * q(1) * q(3) * w(2) + dt * pow(q(2), 2) * w(0) -
       dt * q(2) * q(3) * w(1) - 2 * q(0) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 16) =
      dt *
      (pow(dt, 2) * q(1) * w(0) * w(2) - pow(dt, 2) * q(2) * w(0) * w(1) +
       pow(dt, 2) * q(3) * pow(w(1), 2) + pow(dt, 2) * q(3) * pow(w(2), 2) -
       2 * dt * q(0) * w(0) + 4 * q(3)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 17) =
      dt *
      (pow(dt, 2) * q(1) * w(1) * w(2) + pow(dt, 2) * q(2) * pow(w(0), 2) +
       pow(dt, 2) * q(2) * pow(w(2), 2) - pow(dt, 2) * q(3) * w(0) * w(1) -
       2 * dt * q(0) * w(1) + 4 * q(2)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(12, 18) =
      -dt *
      (pow(dt, 2) * q(1) * pow(w(0), 2) + pow(dt, 2) * q(1) * pow(w(1), 2) +
       pow(dt, 2) * q(2) * w(1) * w(2) + pow(dt, 2) * q(3) * w(0) * w(2) +
       2 * dt * q(0) * w(2) + 4 * q(1)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 12) =
      (dt * q(0) * q(2) * w(0) - dt * q(0) * q(3) * w(1) +
       dt * pow(q(1), 2) * w(2) + dt * pow(q(2), 2) * w(2) +
       dt * pow(q(3), 2) * w(2) - 2 * q(0) * q(1)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 13) =
      (-dt * q(0) * q(1) * w(2) + dt * q(1) * q(2) * w(0) -
       dt * q(1) * q(3) * w(1) + 2 * pow(q(0), 2) + 2 * pow(q(2), 2) +
       2 * pow(q(3), 2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 14) =
      -(dt * pow(q(0), 2) * w(0) + dt * q(0) * q(2) * w(2) +
        dt * pow(q(1), 2) * w(0) + dt * q(2) * q(3) * w(1) +
        dt * pow(q(3), 2) * w(0) + 2 * q(1) * q(2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 15) =
      (dt * pow(q(0), 2) * w(1) - dt * q(0) * q(3) * w(2) +
       dt * pow(q(1), 2) * w(1) + dt * pow(q(2), 2) * w(1) +
       dt * q(2) * q(3) * w(0) - 2 * q(1) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 16) =
      -dt *
      (pow(dt, 2) * q(0) * w(0) * w(2) + pow(dt, 2) * q(2) * pow(w(1), 2) +
       pow(dt, 2) * q(2) * pow(w(2), 2) + pow(dt, 2) * q(3) * w(0) * w(1) +
       2 * dt * q(1) * w(0) + 4 * q(2)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 17) =
      dt *
      (-pow(dt, 2) * q(0) * w(1) * w(2) + pow(dt, 2) * q(2) * w(0) * w(1) +
       pow(dt, 2) * q(3) * pow(w(0), 2) + pow(dt, 2) * q(3) * pow(w(2), 2) -
       2 * dt * q(1) * w(1) + 4 * q(3)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(13, 18) =
      dt *
      (pow(dt, 2) * q(0) * pow(w(0), 2) + pow(dt, 2) * q(0) * pow(w(1), 2) +
       pow(dt, 2) * q(2) * w(0) * w(2) - pow(dt, 2) * q(3) * w(1) * w(2) -
       2 * dt * q(1) * w(2) + 4 * q(0)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 12) =
      -(dt * q(0) * q(1) * w(0) + dt * q(0) * q(3) * w(2) +
        dt * pow(q(1), 2) * w(1) + dt * pow(q(2), 2) * w(1) +
        dt * pow(q(3), 2) * w(1) + 2 * q(0) * q(2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 13) =
      (dt * pow(q(0), 2) * w(0) + dt * q(0) * q(1) * w(1) -
       dt * q(1) * q(3) * w(2) + dt * pow(q(2), 2) * w(0) +
       dt * pow(q(3), 2) * w(0) - 2 * q(1) * q(2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 14) =
      (dt * q(0) * q(2) * w(1) - dt * q(1) * q(2) * w(0) -
       dt * q(2) * q(3) * w(2) + 2 * pow(q(0), 2) + 2 * pow(q(1), 2) +
       2 * pow(q(3), 2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 15) =
      (dt * pow(q(0), 2) * w(2) + dt * q(0) * q(3) * w(1) +
       dt * pow(q(1), 2) * w(2) - dt * q(1) * q(3) * w(0) +
       dt * pow(q(2), 2) * w(2) - 2 * q(2) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 16) =
      dt *
      (pow(dt, 2) * q(0) * w(0) * w(1) + pow(dt, 2) * q(1) * pow(w(1), 2) +
       pow(dt, 2) * q(1) * pow(w(2), 2) - pow(dt, 2) * q(3) * w(0) * w(2) -
       2 * dt * q(2) * w(0) + 4 * q(1)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 17) =
      -dt *
      (pow(dt, 2) * q(0) * pow(w(0), 2) + pow(dt, 2) * q(0) * pow(w(2), 2) +
       pow(dt, 2) * q(1) * w(0) * w(1) + pow(dt, 2) * q(3) * w(1) * w(2) +
       2 * dt * q(2) * w(1) + 4 * q(0)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(14, 18) =
      dt *
      (pow(dt, 2) * q(0) * w(1) * w(2) - pow(dt, 2) * q(1) * w(0) * w(2) +
       pow(dt, 2) * q(3) * pow(w(0), 2) + pow(dt, 2) * q(3) * pow(w(1), 2) -
       2 * dt * q(2) * w(2) + 4 * q(3)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 12) =
      (dt * q(0) * q(1) * w(1) + dt * q(0) * q(2) * w(2) -
       dt * pow(q(1), 2) * w(0) - dt * pow(q(2), 2) * w(0) -
       dt * pow(q(3), 2) * w(0) - 2 * q(0) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 13) =
      (-dt * pow(q(0), 2) * w(1) + dt * q(0) * q(1) * w(0) +
       dt * q(1) * q(2) * w(2) - dt * pow(q(2), 2) * w(1) -
       dt * pow(q(3), 2) * w(1) - 2 * q(1) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 14) =
      (-dt * pow(q(0), 2) * w(2) + dt * q(0) * q(2) * w(0) -
       dt * pow(q(1), 2) * w(2) + dt * q(1) * q(2) * w(1) -
       dt * pow(q(3), 2) * w(2) - 2 * q(2) * q(3)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 15) =
      (dt * q(0) * q(3) * w(0) + dt * q(1) * q(3) * w(1) +
       dt * q(2) * q(3) * w(2) + 2 * pow(q(0), 2) + 2 * pow(q(1), 2) +
       2 * pow(q(2), 2)) /
      ((pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 16) =
      dt *
      (-pow(dt, 2) * q(0) * pow(w(1), 2) - pow(dt, 2) * q(0) * pow(w(2), 2) +
       pow(dt, 2) * q(1) * w(0) * w(1) + pow(dt, 2) * q(2) * w(0) * w(2) -
       2 * dt * q(3) * w(0) - 4 * q(0)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 17) =
      dt *
      (pow(dt, 2) * q(0) * w(0) * w(1) - pow(dt, 2) * q(1) * pow(w(0), 2) -
       pow(dt, 2) * q(1) * pow(w(2), 2) + pow(dt, 2) * q(2) * w(1) * w(2) -
       2 * dt * q(3) * w(1) - 4 * q(1)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(15, 18) =
      dt *
      (pow(dt, 2) * q(0) * w(0) * w(2) + pow(dt, 2) * q(1) * w(1) * w(2) -
       pow(dt, 2) * q(2) * pow(w(0), 2) - pow(dt, 2) * q(2) * pow(w(1), 2) -
       2 * dt * q(3) * w(2) - 4 * q(2)) /
      ((pow(dt, 2) * pow(w(0), 2) + pow(dt, 2) * pow(w(1), 2) +
        pow(dt, 2) * pow(w(2), 2) + 4) *
       sqrt(pow(dt, 2) * pow(q(0), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(0), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(1), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(2), 2) * pow(w(2), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(0), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(1), 2) +
            pow(dt, 2) * pow(q(3), 2) * pow(w(2), 2) + 4 * pow(q(0), 2) +
            4 * pow(q(1), 2) + 4 * pow(q(2), 2) + 4 * pow(q(3), 2)));
  Fx(16, 16) = 1;
  Fx(16, 17) = dt * w(2) * (J_v1 - J_v2) / J_v0;
  Fx(16, 18) = dt * w(1) * (J_v1 - J_v2) / J_v0;
  Fx(17, 16) = dt * w(2) * (-J_v0 + J_v2) / J_v1;
  Fx(17, 17) = 1;
  Fx(17, 18) = dt * w(0) * (-J_v0 + J_v2) / J_v1;
  Fx(18, 16) = dt * w(1) * (J_v0 - J_v1) / J_v2;
  Fx(18, 17) = dt * w(0) * (J_v0 - J_v1) / J_v2;
  Fx(18, 18) = 1;

  Fu(6, 0) = -dt * qc(0) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(6, 1) = -dt * qc(0) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(6, 2) = -dt * qc(0) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(6, 3) = -dt * qc(0) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(7, 0) = -dt * qc(1) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(7, 1) = -dt * qc(1) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(7, 2) = -dt * qc(1) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(7, 3) = -dt * qc(1) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(8, 0) = -dt * qc(2) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(8, 1) = -dt * qc(2) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(8, 2) = -dt * qc(2) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(8, 3) = -dt * qc(2) *
             (-4.9050000000000002 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
              4.9050000000000002 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
              2.4525000000000001 * qc(2) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2))) /
             ((pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(9, 0) = dt * (m + mp) *
             (2.4525000000000001 * qc(1) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
              4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
             (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(9, 1) = dt * (m + mp) *
             (2.4525000000000001 * qc(1) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
              4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
             (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(9, 2) = dt * (m + mp) *
             (2.4525000000000001 * qc(1) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
              4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
             (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(9, 3) = dt * (m + mp) *
             (2.4525000000000001 * qc(1) *
                  (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) -
              4.9050000000000002 * qc(2) * (q(0) * q(3) - q(1) * q(2))) /
             (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(10, 0) = -dt * (m + mp) *
              (2.4525000000000001 * qc(0) *
                   (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
               4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(10, 1) = -dt * (m + mp) *
              (2.4525000000000001 * qc(0) *
                   (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
               4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(10, 2) = -dt * (m + mp) *
              (2.4525000000000001 * qc(0) *
                   (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
               4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(10, 3) = -dt * (m + mp) *
              (2.4525000000000001 * qc(0) *
                   (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
               4.9050000000000002 * qc(2) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(11, 0) = 4.9050000000000002 * dt * (m + mp) *
              (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
               qc(1) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(11, 1) = 4.9050000000000002 * dt * (m + mp) *
              (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
               qc(1) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(11, 2) = 4.9050000000000002 * dt * (m + mp) *
              (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
               qc(1) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(11, 3) = 4.9050000000000002 * dt * (m + mp) *
              (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
               qc(1) * (q(0) * q(2) + q(1) * q(3))) /
              (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
               (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  Fu(16, 0) = -1.7341793804025001 * arm_length * dt * (m + mp) / J_v0;
  Fu(16, 1) = -1.7341793804025001 * arm_length * dt * (m + mp) / J_v0;
  Fu(16, 2) = 1.7341793804025001 * arm_length * dt * (m + mp) / J_v0;
  Fu(16, 3) = 1.7341793804025001 * arm_length * dt * (m + mp) / J_v0;
  Fu(17, 0) = -1.7341793804025001 * arm_length * dt * (m + mp) / J_v1;
  Fu(17, 1) = 1.7341793804025001 * arm_length * dt * (m + mp) / J_v1;
  Fu(17, 2) = 1.7341793804025001 * arm_length * dt * (m + mp) / J_v1;
  Fu(17, 3) = -1.7341793804025001 * arm_length * dt * (m + mp) / J_v1;
  Fu(18, 0) = -2.4525000000000001 * dt * t2t * (m + mp) / J_v2;
  Fu(18, 1) = 2.4525000000000001 * dt * t2t * (m + mp) / J_v2;
  Fu(18, 2) = -2.4525000000000001 * dt * t2t * (m + mp) / J_v2;
  Fu(18, 3) = 2.4525000000000001 * dt * t2t * (m + mp) / J_v2;
}

void inline calcStep(Eigen::Ref<Eigen::VectorXd> xnext, const double *data,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {
  const double m = data[0];
  const double mp = data[1];
  const double J_v0 = data[2];
  const double J_v1 = data[3];
  const double J_v2 = data[4];
  const double t2t = data[5];
  const double l = data[6];
  const double arm_length = data[7];

  Eigen::Vector3d pos = x.head(3).head<3>();
  Eigen::Vector3d qc = x.segment(3, 3).head<3>();
  Eigen::Vector3d vel = x.segment(6, 3).head<3>();
  Eigen::Vector3d wc = x.segment(9, 3).head<3>();
  Eigen::Vector4d q = x.segment(12, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
  Eigen::Vector3d w = x.segment(16, 3).head<3>();
  xnext(0) = dt * vel(0) + pos(0);
  xnext(1) = dt * vel(1) + pos(1);
  xnext(2) = dt * vel(2) + pos(2);
  xnext(3) =
      (-dt * (qc(1) * wc(2) - qc(2) * wc(1)) +
       qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) /
      (sqrt(
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
       sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  xnext(4) =
      (dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
       qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) /
      (sqrt(
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
       sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  xnext(5) =
      (-dt * (qc(0) * wc(1) - qc(1) * wc(0)) +
       qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) /
      (sqrt(
           (pow(dt * (qc(0) * wc(1) - qc(1) * wc(0)) -
                    qc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(0) * wc(2) - qc(2) * wc(0)) +
                    qc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2) +
            pow(dt * (qc(1) * wc(2) - qc(2) * wc(1)) -
                    qc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)),
                2)) /
           (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2))) *
       sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)));
  xnext(6) =
      (-dt * qc(0) *
           (l * m * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
                (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2.4525000000000001 * (m + mp) *
                sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
                 2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(2) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                          pow(q(3), 2))) *
                (u(0) + u(1) + u(2) + u(3))) +
       vel(0) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(7) =
      (-dt * qc(1) *
           (l * m * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
                (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2.4525000000000001 * (m + mp) *
                sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
                 2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(2) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                          pow(q(3), 2))) *
                (u(0) + u(1) + u(2) + u(3))) +
       vel(1) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(8) =
      (-dt *
           (l * m * qc(2) * (pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (pow(wc(0), 2) + pow(wc(1), 2) + pow(wc(2), 2)) *
                (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)) +
            2.4525000000000001 * qc(2) * (m + mp) *
                sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                (-2 * qc(0) * (q(0) * q(2) + q(1) * q(3)) +
                 2 * qc(1) * (q(0) * q(3) - q(1) * q(2)) +
                 qc(2) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                          pow(q(3), 2))) *
                (u(0) + u(1) + u(2) + u(3)) +
            9.8100000000000005 * (m + mp) *
                pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
                (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) +
       vel(2) * (m + mp) *
           pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      ((m + mp) *
       pow(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2), 3.0 / 2.0) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(9) = (2.4525000000000001 * dt * (m + mp) *
                  (qc(1) * (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) -
                            pow(q(3), 2)) -
                   2 * qc(2) * (q(0) * q(3) - q(1) * q(2))) *
                  (u(0) + u(1) + u(2) + u(3)) +
              m * wc(0) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
                  (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
             (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
              (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(10) =
      (-2.4525000000000001 * dt * (m + mp) *
           (qc(0) *
                (pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2)) +
            2 * qc(2) * (q(0) * q(2) + q(1) * q(3))) *
           (u(0) + u(1) + u(2) + u(3)) +
       m * wc(1) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(11) =
      (4.9050000000000002 * dt * (m + mp) *
           (qc(0) * (q(0) * q(3) - q(1) * q(2)) +
            qc(1) * (q(0) * q(2) + q(1) * q(3))) *
           (u(0) + u(1) + u(2) + u(3)) +
       m * wc(2) * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
           (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2))) /
      (m * sqrt(pow(qc(0), 2) + pow(qc(1), 2) + pow(qc(2), 2)) *
       (pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2)));
  xnext(12) =
      (dt * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) + 2 * q(0)) /
      sqrt(pow(dt * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) - 2 * q(3), 2) +
           pow(dt * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) + 2 * q(2), 2) +
           pow(dt * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) + 2 * q(1), 2) +
           pow(dt * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) + 2 * q(0), 2));
  xnext(13) =
      (dt * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) + 2 * q(1)) /
      sqrt(pow(dt * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) - 2 * q(3), 2) +
           pow(dt * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) + 2 * q(2), 2) +
           pow(dt * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) + 2 * q(1), 2) +
           pow(dt * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) + 2 * q(0), 2));
  xnext(14) =
      (dt * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) + 2 * q(2)) /
      sqrt(pow(dt * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) - 2 * q(3), 2) +
           pow(dt * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) + 2 * q(2), 2) +
           pow(dt * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) + 2 * q(1), 2) +
           pow(dt * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) + 2 * q(0), 2));
  xnext(15) =
      (-dt * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) + 2 * q(3)) /
      sqrt(pow(dt * (q(0) * w(0) + q(1) * w(1) + q(2) * w(2)) - 2 * q(3), 2) +
           pow(dt * (-q(0) * w(1) + q(1) * w(0) + q(3) * w(2)) + 2 * q(2), 2) +
           pow(dt * (q(0) * w(2) - q(2) * w(0) + q(3) * w(1)) + 2 * q(1), 2) +
           pow(dt * (-q(1) * w(2) + q(2) * w(1) + q(3) * w(0)) + 2 * q(0), 2));
  xnext(16) =
      (J_v0 * w(0) + dt * (J_v1 * w(1) * w(2) - J_v2 * w(1) * w(2) -
                           1.7341793804025001 * arm_length * u(0) * (m + mp) -
                           1.7341793804025001 * arm_length * u(1) * (m + mp) +
                           1.7341793804025001 * arm_length * u(2) * (m + mp) +
                           1.7341793804025001 * arm_length * u(3) * (m + mp))) /
      J_v0;
  xnext(17) =
      (J_v1 * w(1) - dt * (J_v0 * w(0) * w(2) - J_v2 * w(0) * w(2) +
                           1.7341793804025001 * arm_length * u(0) * (m + mp) -
                           1.7341793804025001 * arm_length * u(1) * (m + mp) -
                           1.7341793804025001 * arm_length * u(2) * (m + mp) +
                           1.7341793804025001 * arm_length * u(3) * (m + mp))) /
      J_v1;
  xnext(18) =
      (J_v2 * w(2) + dt * (J_v0 * w(0) * w(1) - J_v1 * w(0) * w(1) -
                           2.4525000000000001 * t2t * u(0) * (m + mp) +
                           2.4525000000000001 * t2t * u(1) * (m + mp) -
                           2.4525000000000001 * t2t * u(2) * (m + mp) +
                           2.4525000000000001 * t2t * u(3) * (m + mp))) /
      J_v2;
}
