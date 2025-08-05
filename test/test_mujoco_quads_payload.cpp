#include <boost/test/unit_test.hpp>
#include "dynobench/motions.hpp"
#include "dynobench/quadrotor_payload.hpp"
#include "dynobench/quadrotor_payload_n.hpp"
#include "dynobench/mujoco_quadrotors_payload.hpp"

#include <GLFW/glfw3.h>
#define base_path "../dynoplan/dynobench/"

using namespace dynobench;

BOOST_AUTO_TEST_CASE(t_mujoco_quads_payload) {

    // Init GLFW (required by MuJoCo GUI)
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW\n";
        BOOST_FAIL("GLFW init failed");
    }

    dynobench::MujocoQuadsPayload_params params;
    const std::string yaml_path_str = std::string(base_path) + "models/mujoco_payload1.yaml";
    std::cout << "Reading YAML from: " << yaml_path_str << std::endl;
    params.read_from_yaml(yaml_path_str.c_str());  // ✅ pass const char*
    Eigen::VectorXd p_lb(3);
    p_lb << -1000, -1000, -1000;
    Eigen::VectorXd p_ub(3);
    p_ub << 1000, 1000, 1000;    
    auto model = mk<dynobench::Model_MujocoQuadsPayload>(params, p_lb, p_ub); 
    if (!model || !model->m || !model->d) {
        std::cerr << "Model or MuJoCo components are NULL!" << std::endl;
        BOOST_FAIL("Failed to initialize MuJoCo model or data");
    }

    const int nb = params.num_robots + 1;

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model->nx);
    Eigen::MatrixXd Jx(2 * model->m->nv, model->m->nq + model->m->nv);
    Eigen::MatrixXd Jx_diff(2 * model->m->nv, model->m->nq + model->m->nv);
    Eigen::MatrixXd Ju(2 * model->m->nv, model->nu);

    Eigen::VectorXd pose0(7 * nb);
    Eigen::VectorXd vel0(6 * nb);

    pose0 <<
        -0.367647, 0.000556508, 0.5230775, 0.0, 0.0, 0.0, 1.0,
        -0.1017685, -0.000258452, 0.946525, 0.00129351, 0.0368085, 0.0389976, 0.99856;

    vel0 <<
        1.08576, 0.00375725, 0.0478173, 0.0, 0.0, 0.0,
        1.339227162945728, -0.0224965454957, -0.111382754024272, -0.301425, -0.837661, 0.283565;

    x0.head(7 * nb) = pose0;
    x0.tail(model->m->nv) = vel0;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(model->nu);
    u << 0.087309, 0.087309, 0.087309, 0.087309;

    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(model->nx);
    Eigen::VectorXd pose1(7 * nb);
    pose1 <<
        0.0, 0.0, 0.1, 0, 0, 0, 1,
        0.2, 0.0, 0.6, 0, 0, 0, 1;
    x1.head(7 * nb) = pose1;
    x1.tail(model->m->nv).setZero();

    double test_distance = model->distance(x1, x0);
    std::cout << "Distance is: " << test_distance << "\nBetween: \n"
              << x0.transpose() << "\n" << x1.transpose() << std::endl;

    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        BOOST_FAIL("GLFW window creation failed");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvScene scn;
    mjrContext con;
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(model->m, &scn, 2000);
    mjr_makeContext(model->m, &con, mjFONTSCALE_150);

    mjvCamera cam;
    mjv_defaultCamera(&cam);
    mjvOption opt;
    mjv_defaultOption(&opt);

    auto ctrl = mjVec(model->d->ctrl, model->m->nu);
    auto qpos_mj = mjVec(model->d->qpos, model->m->nq);
    auto qvel_mj = mjVec(model->d->qvel, model->m->nq);

    while (!glfwWindowShouldClose(window)) {
        int i = 0;
        qvel_mj.setZero();
        while (i < 1) {
            ctrl = u;

            model->calcDiffV(Jx, Ju, x0, u);

            finite_diff_jac(
                [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
                    model->calcV(y, x, u);
                },
                x0, 2 * model->m->nv, Jx_diff);

            std::cout << "-----------\n"
                      << "Report Jx:\n";
            approx_equal_report(Jx, Jx_diff);

            x0.swap(x1);  // dummy update to continue loop

            mjv_updateScene(model->m, model->d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            mjrRect viewport = {0, 0, width, height};
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();

            ++i;
        }
        exit(3);  // Terminate after one iteration
    }

    mj_deleteData(model->d);
    mj_deleteData(model->tmp);
    mj_deleteModel(model->m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
}
