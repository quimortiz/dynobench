#include <boost/test/unit_test.hpp>
#include "dynobench/motions.hpp"
#include "dynobench/mujoco_quadrotor.hpp"
#include "dynobench/quadrotor.hpp"

#include <GLFW/glfw3.h>

#define base_path "../dynoplan/dynobench/"

using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_compare_mujoco_and_quad3d) {

    if (!glfwInit()) {
        BOOST_FAIL("GLFW init failed");
    }

    // ==== Load MujocoQuad ====
    MujocoQuad_params mj_params;
    mj_params.read_from_yaml((std::string(base_path) + "models/mujocoquad_empty.yaml").c_str());
    Eigen::VectorXd p_lb(3); p_lb << -1000, -1000, -1000;
    Eigen::VectorXd p_ub(3); p_ub << 1000, 1000, 1000;
    auto model_mj = mk<Model_MujocoQuad>(mj_params, p_lb, p_ub);

    // ==== Load Quad3D ====
    Quad3d_params q3d_params;
    q3d_params.read_from_yaml((std::string(base_path) + "models/quad3d_v0.yaml").c_str());
    auto model_q3d = mk<Model_quad3d>(q3d_params, p_lb, p_ub);

    // ==== Random test state/control ====
    Eigen::VectorXd x0(13);
    Eigen::VectorXd u(4);

    // Random position in [-2, 2]
    x0.segment<3>(0) = 4.0 * Eigen::Vector3d::Random();

    // Random quaternion, normalized
    Eigen::Vector4d q_rand = Eigen::Vector4d::Random();
    q_rand.normalize();
    x0.segment<4>(3) = q_rand;

    // Random linear velocity in [-3, 3]
    x0.segment<3>(7) = 6.0 * Eigen::Vector3d::Random();

    // Random angular velocity in [-2, 2]
    x0.segment<3>(10) = 4.0 * Eigen::Vector3d::Random();

    // Random control input in [0.05, 0.2]
    u = 0.05 * Eigen::Vector4d::Ones() +
        (0.2 - 0.05) * Eigen::Vector4d::Random().cwiseAbs();

    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "u: " << u.transpose() << std::endl;
    // ==== Prepare matrices ====
    Eigen::MatrixXd Jx_mj(2 * model_mj->m->nv, model_mj->m->nq + model_mj->m->nv);
    Jx_mj.setZero();
    Eigen::MatrixXd Ju_mj(2 * model_mj->m->nv, model_mj->nu);
    Ju_mj.setZero();
    Eigen::MatrixXd Jx_q3d(12, 13); // 3d quadrotor always 12x13 for calcV
    Jx_q3d.setZero();
    Eigen::MatrixXd Ju_q3d(12, 4);
    Ju_q3d.setZero();

    // ==== Prepare matrices ====
    Eigen::MatrixXd Fx_mj(model_mj->m->nq + model_mj->m->nv, model_mj->m->nq + model_mj->m->nv);
    Fx_mj.setZero();
    Eigen::MatrixXd Fu_mj(model_mj->m->nq + model_mj->m->nv, model_mj->nu);
    Fu_mj.setZero();
    Eigen::MatrixXd Fx_q3d(13, 13); // 3d quadrotor always 13x13 for calcV
    Fx_q3d.setZero();
    Eigen::MatrixXd Fu_q3d(13, 4);
    Fu_q3d.setZero();


    // ==== Calculate Mujoco Jacobian ====
    model_mj->calcDiffV(Jx_mj, Ju_mj, x0, u);
    model_mj->stepDiff(Fx_mj, Fu_mj, x0, u, 0.01);

    // ==== Calculate Quad3D Jacobian ====
    model_q3d->calcDiffV(Jx_q3d, Ju_q3d, x0, u);
    model_q3d->stepDiff(Fx_q3d, Fu_q3d, x0,  u, 0.01);

    std::cout << "Jx_mj: \n" << Jx_mj << std::endl;
    std::cout << "Jx_qd3: \n" << Jx_q3d << std::endl;
    // ==== Report ====
    std::cout << "\n===== Mujoco vs Finite-Diff =====\n";
    approx_equal_report(Jx_mj, Jx_q3d);
    std::cout << "\n" << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << "\n" << std::endl;
    
    std::cout << "Fx_mj: \n" << Jx_mj << std::endl;
    std::cout << "Fx_qd3: \n" << Jx_q3d << std::endl;
    approx_equal_report(Fx_mj, Fx_q3d);

    // Exit without opening a viewer
    glfwTerminate();
}
