#include <boost/test/unit_test.hpp>

#include "dynobench/motions.hpp"
#include "dynobench/quadrotor_payload.hpp"

#define base_path "../../"

using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_hello_quadrotor_payload) {

  std::cout << "Hello Khaled" << std::endl;
  auto model = mk<dynobench::Model_quad3dpayload>();
  std::cout << "Model has been created" << std::endl;
  BOOST_TEST(true);

  Problem problem1(
    base_path "envs/quad3d_payload/empty_0.yaml");

  Problem problem2(
    base_path "envs/quad3d_payload/empty_1.yaml");

}

BOOST_AUTO_TEST_CASE(t_quadrotor_payload_dynamics) {

  std::cout << "hello " << std::endl;
  auto model = mk<dynobench::Model_quad3dpayload>();

  int nx = model->nx;
  int nu = model->nu;

  Eigen::VectorXd x0(nx), u0(nu);
  x0.setZero();
  x0 = model->get_x0(x0);
  u0 = model->u_0;

  Eigen::VectorXd xrand(nx), urand(nu);
  xrand.setZero(); // TODO: DONE
  xrand << 3., 3., 1., 0., 0., -1., 0., 0., 0., 0., 0. , 0., 0., 0., 0., 1., 0., 0., 0.;
  urand << .1, 1., .5, 0;

  Eigen::MatrixXd Jx_diff(nx, nx), Ju_diff(nx, nu), Jx(nx, nx), Ju(nx, nu);
  Eigen::MatrixXd Sx_diff(nx, nx), Su_diff(nx, nu), Sx(nx, nx), Su(nx, nu);

  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> xu_s;

  xu_s.push_back({x0, u0});
  xu_s.push_back({xrand, urand});


  double dt = model->ref_dt;

  for (auto &[x, u] : xu_s) {

    for (auto &m_ptr :
         {&Jx_diff, &Ju_diff, &Jx, &Ju, &Sx_diff, &Su_diff, &Sx, &Su}) {
      m_ptr->setZero();
    }

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


    std::cout << (Jx - Jx_diff).norm() << std::endl;
    std::cout << (Ju - Ju_diff).norm() << std::endl;


    BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
    BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);

    BOOST_TEST((Sx - Sx_diff).norm() < 1e-5);
    BOOST_TEST((Su - Su_diff).norm() < 1e-5);
  }
}

BOOST_AUTO_TEST_CASE(t_quadrotor_payload_collisions) {

  // TODO: quim: create a sensible test case!
  BOOST_TEST(false);
}
