#include <boost/test/unit_test.hpp>

#include "dynobench/motions.hpp"
#include "dynobench/quadrotor_payload.hpp"
#include "dynobench/quadrotor_payload_n.hpp"

// #define base_path "../../dynobench/"
#define base_path "../dynoplan/dynobench/"

using namespace dynobench;


BOOST_AUTO_TEST_CASE(t_quadrotor_payload_2_p_dynamics) {

  std::cout << "Hello Qium :)" << std::endl;
  dynobench::Quad3dpayload_n_params params;
  params.read_from_yaml(base_path "models/point_2.yaml");
  params.point_mass = true;
  params.num_robots = 2;

  auto model = mk<dynobench::Model_quad3dpayload_n>(params);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model->nx);

  int nx = model->nx;
  int nu = model->nu;

  Eigen::VectorXd x_default(nx), u_default(nu);
  x_default.setZero();
  x_default = model->get_x0(x_default);
  // std::cout << x_default << std::endl;
  // std::cout << "state: " << nx << std::endl;
  // std::cout << "input: " << nu << std::endl;
  // exit(3);
  u_default = model->u_0;

  Eigen::VectorXd xrand(nx), urand(nu), xrandnoise(nx), urandnoise(nx);
  xrand.setZero(); // TODO: DONE
  xrand << 3., 3., 1., 1., 2., 5., 0.3, 0.2, -0.4, 0.1, 0.13, 0.43, -0.3, 0.4,
      -1., 0.66, 0.8, 0.93, 0.1, 0.2, 0.3, 0.6, 3., 2., 0.6, 0.4, 0.3, 0.8, 0.2,
      1., 2., 3.;
  urand << 0.3, 0.6, 0.8, 0.4, 0.5, 0.2, 0.7, 0.1;

  xrandnoise = xrand + 0.01 * Eigen::VectorXd::Random(nx);
  model->ensure(xrandnoise);
  urandnoise = urand + 0.01 * Eigen::VectorXd::Random(nu);

  Eigen::MatrixXd Jx_diff(nx, nx), Ju_diff(nx, nu), Jx(nx, nx), Ju(nx, nu);
  Eigen::MatrixXd Sx_diff(nx, nx), Su_diff(nx, nu), Sx(nx, nx), Su(nx, nu);

  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> xu_s;

  xu_s.push_back({x_default, u_default});
  xu_s.push_back({xrand, urand});
  xu_s.push_back({xrandnoise, urandnoise});

  double dt = model->ref_dt;

  for (const auto &k : xu_s) {
    const auto &x0 = k.first;
    const auto &u0 = k.second;

    for (auto &m_ptr :
         {&Jx_diff, &Ju_diff, &Jx, &Ju, &Sx_diff, &Su_diff, &Sx, &Su}) {
      m_ptr->setZero();
    }

    CSTR_V(x0);
    CSTR_V(u0);

    model->calcDiffV(Jx, Ju, x0, u0);
    model->stepDiff(Sx, Su, x0, u0, dt);

    finite_diff_jac(
        [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
          model->calcV(y, x, u0);
        },
        x0, nx, Jx_diff);

    finite_diff_jac(
        [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
          model->calcV(y, x0, u);
        },
        u0, nx, Ju_diff);

    finite_diff_jac(
        [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
          model->step(y, x, u0, dt);
        },
        x0, nx, Sx_diff);

    finite_diff_jac(
        [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
          model->step(y, x0, u, dt);
        },
        u0, nx, Su_diff);
    std::cout << "Jx: \n" << Jx << std::endl;
    std::cout << "Jx_diff: \n" << Jx_diff << std::endl;

    std::cout << "-----------\n"
              << "report Jx " << std::endl;
    approx_equal_report(Jx, Jx_diff);
    std::cout << "report Ju " << std::endl;
    approx_equal_report(Ju, Ju_diff);

    std::cout << "report Sx " << std::endl;
    approx_equal_report(Sx, Sx_diff);
    std::cout << "report Su " << std::endl;
    approx_equal_report(Su, Su_diff);

    BOOST_TEST((Jx - Jx_diff).norm() <= 10 * 1e-5);
    BOOST_TEST((Ju - Ju_diff).norm() <= 10 * 1e-5);

    BOOST_TEST((Sx - Sx_diff).norm() <= 10 * 1e-5);
    BOOST_TEST((Su - Su_diff).norm() <= 10 * 1e-5);
  }
}



// BOOST_AUTO_TEST_CASE(t_quadrotor_payload_3_p_dynamics) {

//   dynobench::Quad3dpayload_n_params params;
//   params.read_from_yaml(base_path "models/point_3.yaml");


//   // params.point_mass = true;
//   // params.num_robots = 3;

//   auto model = mk<dynobench::Model_quad3dpayload_n>(params);
//   Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model->nx);

//   int nx = model->nx;
//   int nu = model->nu;

//   Eigen::VectorXd x_default(nx), u_default(nu);
//   x_default.setZero();
//   x_default = model->get_x0(x_default);
//   // std::cout << x_default << std::endl;
//   // std::cout << "state: " << nx << std::endl;
//   // std::cout << "input: " << nu << std::endl;
//   // exit(3);
//   u_default = model->u_0;

//   Eigen::VectorXd xrand(nx), urand(nu) , xrandnoise(nx) , urandnoise(nx);
//   xrand.setZero(); // TODO: DONE
//   xrand << 3., 3., 1., 1., 2., 5.,
//   0.3, 0.2, -0.4, 0.1, 0.13, 0.43,  -0.3, 0.4, -1., 0.66, 0.8, 0.93,  -0.24, 0.56, 0.56, 0.3, 0.1, 0.1,
//   0.1, 0.2, 0.3, 0.6,   3., 2., 0.6,     0.4, 0.3, 0.8, 0.2,   1., 2., 3., 0.2, 0.44, 0.32, 0.11, 1.31, 2.12, 2.12;
//   urand << 0.3, 0.6, 0.8, 0.4,  0.5, 0.2, 0.7, 0.1,  0.4, 0.3, 0.3, 1.;


//   xrandnoise = xrand + 0.01 * Eigen::VectorXd::Random(nx);
//   model->ensure(xrandnoise);
//   urandnoise = urand + 0.01 * Eigen::VectorXd::Random(nu);


//   Eigen::MatrixXd Jx_diff(nx, nx), Ju_diff(nx, nu), Jx(nx, nx), Ju(nx, nu);
//   Eigen::MatrixXd Sx_diff(nx, nx), Su_diff(nx, nu), Sx(nx, nx), Su(nx, nu);

//   std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> xu_s;

//   xu_s.push_back({x_default, u_default});
//   xu_s.push_back({xrand, urand});
//   xu_s.push_back({xrandnoise, urandnoise});

//   double dt = model->ref_dt;

//   for (const auto &k : xu_s) {
//     const auto &x0 = k.first;
//     const auto &u0 = k.second;

//     for (auto &m_ptr :
//          {&Jx_diff, &Ju_diff, &Jx, &Ju, &Sx_diff, &Su_diff, &Sx, &Su}) {
//       m_ptr->setZero();
//     }

//     CSTR_V(x0);
//     CSTR_V(u0);

//     model->calcDiffV(Jx, Ju, x0, u0);
//     model->stepDiff(Sx, Su, x0, u0, dt);

//     finite_diff_jac(
//         [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
//           model->calcV(y, x, u0);
//         },
//         x0, nx, Jx_diff);

//     finite_diff_jac(
//         [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
//           model->calcV(y, x0, u);
//         },
//         u0, nx, Ju_diff);

//     finite_diff_jac(
//         [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
//           model->step(y, x, u0, dt);
//         },
//         x0, nx, Sx_diff);

//     finite_diff_jac(
//         [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
//           model->step(y, x0, u, dt);
//         },
//         u0, nx, Su_diff);
//     std::cout << "Jx: \n" << Jx << std::endl;
//     std::cout << "Jx_diff: \n" << Jx_diff << std::endl;

//     std::cout << "-----------\n"
//               << "report Jx " << std::endl;
//     approx_equal_report(Jx, Jx_diff);
//     std::cout << "report Ju " << std::endl;
//     approx_equal_report(Ju, Ju_diff);

//     std::cout << "report Sx " << std::endl;
//     approx_equal_report(Sx, Sx_diff);
//     std::cout << "report Su " << std::endl;
//     approx_equal_report(Su, Su_diff);

//     BOOST_TEST((Jx - Jx_diff).norm() <= 10 * 1e-3);
//     BOOST_TEST((Ju - Ju_diff).norm() <= 10 * 1e-3);

//     BOOST_TEST((Sx - Sx_diff).norm() <= 10 * 1e-3);
//     BOOST_TEST((Su - Su_diff).norm() <= 10 * 1e-3);
//   }

// }
