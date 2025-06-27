#include <boost/test/unit_test.hpp>

#include "dynobench/motions.hpp"
#include "dynobench/quadrotor_payload.hpp"
#include "dynobench/quadrotor_payload_n.hpp"
// #include "dynobench/quadrotor.hpp"
#include "dynobench/mujoco_quadrotors_payload.hpp"
#include <GLFW/glfw3.h>
// #include <mujoco/mujoco.h>
// #define base_path "../../dynobench/"
#define base_path "../"

using namespace dynobench;


BOOST_AUTO_TEST_CASE(t_mujoco_quads_payload) {

    // Init GLFW (required by MuJoCo GUI)
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW\n";
    }

    dynobench::MujocoQuadsPayload_params params;
    params.read_from_yaml(base_path "models/mujoco_payload.yaml");

    auto model = mk<dynobench::Model_MujocoQuadsPayload>(params);
    const int nb = params.num_robots + 1;        // payload + drones
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model->nx);
    x0.setZero(model->nx);
    Eigen::MatrixXd Jx(model->nx, model->nx), Ju(model->nx, model->nu);
    Eigen::MatrixXd Sx(model->nx, model->nx), Su(model->nx, model->nu);
    /* payload + one quad: [p  q_xyzw] ------------------------------------- */
    Eigen::VectorXd pose0(7*nb);
    std::cout << "model size: " << model->nx << std::endl;
    pose0 <<
        0.0, 0.0, 0.0,   0, 0, 0, 1,          // payload  (x y z  qx qy qz qw)
        0.2, 0.0, 0.4,   0, 0, 0, 1,          // quadrotor 1
       -0.2, 0.0, 0.4,   0, 0, 0, 1;          // quadrotor 2
    x0.head(7*nb) = pose0;
    x0.tail(model->m->nv).setZero();      // zero velocities
    Eigen::VectorXd xnext = Eigen::VectorXd::Zero(model->nx);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(model->nu);
    u << 0.087309, 0.087309, 0.087309, 0.087309,  0.087309, 0.087309, 0.087309, 0.087309;



    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(model->nx);
    x1.setZero(model->nx);
    Eigen::VectorXd pose1(7*nb);
    std::cout << "model size: " << model->nx << std::endl;
    pose1 <<
        0.0, 0.0, 0.3,   0, 0, 0, 1,          // payload  (x y z  qx qy qz qw)
        0.2, 0.0, 0.4,   0, 0, 0, 1,          // quadrotor 1
       -0.2, 0.0, 0.4,   0, 0, 0, 1;          // quadrotor 2
    x1.head(7*nb) = pose1;
    x1.tail(model->m->nv).setZero();      // zero velocities

    double test_distance = model->distance(x1, x0);
    std::cout << "distance is: " << test_distance << "\nbetween: \n" << x0.transpose() << "\n" <<
    x1.transpose() << std::endl;

    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", nullptr, nullptr);

    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Init visualization
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
        while (i < 1000) {
            ctrl = u;
            model->step(xnext, x0, u, model->ref_dt);
            model->calcDiffV(Jx, Ju, x0, u);
            model->stepDiff(Sx, Su, x0, u, model->ref_dt);
            x0.swap(xnext);
            ++i;
            mjv_updateScene(model->m, model->d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            mjrRect viewport = {0, 0, width, height};
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }
    // Cleanup
    mj_deleteData(model->d);
    mj_deleteData(model->tmp);
    mj_deleteModel(model->m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
}
