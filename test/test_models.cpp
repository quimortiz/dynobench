#include "dynobench/math_utils.hpp"
#include "dynobench/multirobot_trajectory.hpp"
#include "dynobench/robot_models.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

// #include <boost/test/unit_test_suite.hpp>
// #define BOOST_TEST_DYN_LINK
// #include <boost/test/unit_test.hpp>

// see
// https://www.boost.org/doc/libs/1_81_0/libs/test/doc/html/boost_test/usage_variants.html
// #define BOOST_TEST_MODULE test module name

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "Eigen/Core"
#include <boost/program_options.hpp>

// #include "collision_checker.hpp"

// save data without the cluster stuff

#include <filesystem>
#include <random>
#include <regex>
#include <type_traits>

#include <filesystem>
#include <regex>

#include "dynobench/motions.hpp"
#include <Eigen/Dense>
#include <iostream>

#include "dynobench/car.hpp"
#include "dynobench/car2.hpp"
#include "dynobench/unicycle1.hpp"
#include "dynobench/unicycle2.hpp"

#include "dynobench/acrobot.hpp"
#include "dynobench/car.hpp"
#include "dynobench/integrator1_2d.hpp"
#include "dynobench/integrator2_2d.hpp"
#include "dynobench/joint_robot.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include "dynobench/quadrotor.hpp"
#include "dynobench/differential_drive.hpp"

using namespace std;
using namespace dynobench;

Eigen::VectorXd default_vector;

// #define base_path "../../dynobench/"
#define base_path "../../"

struct Fake_opt {
  Fake_opt() = default;
  std::string filename = "tmp.yaml";
  int max_it = 10;

  // NLOHMANN_DEFINE_TYPE_INTRUSIVE(Fake_opt, filename, max_it);
  NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(Fake_opt, filename, max_it);
};

// for string delimiter
std::vector<std::string> split(std::string s, std::string delimiter) {
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    if (token.size())
      res.push_back(token);
  }

  if (s.substr(pos_start).size())
    res.push_back(s.substr(pos_start));
  for (auto &r : res) {
    std::cout << "- " << r << std::endl;
  }
  return res;
}

void tokenize(std::string const &str, const char delim,
              std::vector<std::string> &out) {
  // construct a stream from the string
  std::stringstream ss(str);

  std::string s;
  while (std::getline(ss, s, delim)) {
    if (s.size())
      out.push_back(s);
  }
}

Fake_opt fakeCLIParser(const std::string &argv) {

  auto outs = split(argv, "--");

  std::ofstream o(base_path "fake_cli.json");
  o << "{\n";

  for (size_t i = 0; i < outs.size(); i++) {
    auto out = outs[i];

    std::vector<std::string> _out;
    tokenize(out, ' ', _out);
    DYNO_CHECK_EQ(_out.size(), 2, "");
    // if (i < outs.size() - 1)
    o << "  \"" << _out[0] << "\": ";

    try {
      double d = std::stod(_out[1]);
      o << _out[1];
    } catch (const std::exception &e) {
      o << "\"" << _out[1] << "\"";
    }

    if (i < outs.size() - 1) {
      o << ",\n";
    } else {
      o << "\n";
    }
  }

  o << "}\n";
  o.close();

  std::ifstream ii(base_path "fake_cli.json");
  json j = json::parse(ii); //

  Fake_opt opt = j;
  return opt;
}

BOOST_AUTO_TEST_CASE(t_traj_to_json) {

  Trajectory traj;

  traj.states.push_back(Eigen::VectorXd::Zero(3));
  traj.states.push_back(Eigen::VectorXd::Ones(3));
  traj.states.push_back(-1 * Eigen::VectorXd::Zero(3));

  traj.actions.push_back(Eigen::VectorXd::Zero(2));
  traj.actions.push_back(Eigen::VectorXd::Ones(2));

  json j = traj;
  std::cout << j.dump(2) << std::endl;

  auto filename = "/tmp/dynobench/traj.json";
  create_dir_if_necessary(filename);
  std::ofstream out(filename);
  out << j;
  out.close();

  std::ifstream in(filename);
  json j2;
  in >> j2;

  Trajectory traj2 = j2;
  BOOST_TEST(traj.states == traj2.states);
  BOOST_TEST(traj.actions == traj2.actions);

  {
    Trajectories trajs;
    trajs.data.push_back(traj);
    trajs.data.push_back(traj);
    trajs.data.push_back(traj);
    trajs.data.push_back(traj);
    trajs.data.push_back(traj);

    auto filename = "/tmp/dynobench/trajs.json";
    create_dir_if_necessary(filename);
    std::ofstream out(filename);
    out << json(trajs);
    out.close();

    {
      std::vector<std::uint8_t> v_msgpack = json::to_msgpack(json(trajs));

      ofstream fout("data.dat", ios::out | ios::binary);
      fout.write((const char *)v_msgpack.data(), v_msgpack.size());
      fout.close();

      ifstream fin("data.dat", ios::in | ios::binary);

      // std::vector<uint8_t> contents((std::istreambuf_iterator<char>(fin)),
      //                               std::istreambuf_iterator<char>());

      std::vector<uint8_t> contents;
      fin.seekg(0, std::ios::end);
      contents.resize(fin.tellg());
      fin.seekg(0, std::ios::beg);
      fin.read((char *)contents.data(), contents.size());
      in.close();

      json j = json::from_msgpack(contents);
      std::cout << j.dump(2) << std::endl;
      Trajectories trajsX = j;
    }

    // lets load from msgpack

    std::ifstream in(filename);
    json j;
    in >> j;
    Trajectories trajs2 = j;
  }
}

BOOST_AUTO_TEST_CASE(t_jsonX) {}

BOOST_AUTO_TEST_CASE(t_fakeCLIParser) {
  Fake_opt opt =
      fakeCLIParser("--filename ../models/quad2d_v0.yaml --max_it   100");
  std::cout << opt.max_it << " " << opt.filename << std::endl;

  {
    Fake_opt opt =
        fakeCLIParser("--filename ../models/quad2d_v0.yaml     --max_it   100");
    std::cout << opt.max_it << " " << opt.filename << std::endl;
  }

  {
    Fake_opt opt =
        fakeCLIParser("--filename ../models/quad2d_v0.yaml --max_it 100");
    std::cout << opt.max_it << " " << opt.filename << std::endl;
  }

  {
    Fake_opt opt = fakeCLIParser("--max_it 100");
    std::cout << opt.max_it << " " << opt.filename << std::endl;
  }
}

// BOOST_AUTO_TEST_CASE(yaml_json) {
//
//   YAML::Node node = YAML::LoadFile(base_path "tets_yaml_json.yaml");
//
//   json j = tojson::detail::yaml2json(node);
//   std::cout << j.dump() << std::endl;
//
//   std::cout << tojson::emitters::toyaml(j) << std::endl;
//   std::ofstream o(base_path "tmp.yaml");
//   std::ofstream o2(base_path "tmp2.yaml");
//   o << tojson::emitters::toyaml(j) << std::endl;
//
//   YAML::Emitter ee;
//   ee << node;
//   o2 << ee.c_str() << std::endl;
//   // how to test that they are the same?
// }

BOOST_AUTO_TEST_CASE(t_load_json) {
  Unicycle1_paramsJ aa;

  aa.read_from_yaml(
      (std::string(base_path) + "models/unicycle1_v0.yaml").c_str());

  aa.write_yaml(std::cout);
}

BOOST_AUTO_TEST_CASE(t_load_model_yaml) {

  std::shared_ptr<Model_robot> robot = std::make_shared<Model_quad2d>(
      (std::string(base_path) + "models/quad2d_v0.yaml").c_str());
}

// BOOST_AUTO_TEST_CASE(t_extract_primitives) {
//
//   Trajectory traj;
//
//   // TODO: change
//   traj.read_from_yaml("../results_new/quad2d/quad2d_recovery_obs/idbastar_v0/"
//                       "2023-06-21--12-40-54/"
//                       "run_0_out.yaml.trajraw-0.yaml-fakeprimitives.yaml");
//
//   std::shared_ptr<Model_robot> robot =
//       std::make_shared<Model_quad2d>("../models/quad2d_v0.yaml");
//
//   std::vector<Trajectory> primitives =
//   traj.find_discontinuities(robot);
//
//   // lets write the trajectories
//   std::ofstream out("fileout_primitives.yaml");
//   traj.to_yaml_format(out);
//
//   auto prefix1 = "  ";
//   auto prefix2 = "    ";
//   out << "primitives:" << std::endl;
//   for (auto &p : primitives) {
//     out << prefix1 << "- " << std::endl;
//     p.to_yaml_format(out, prefix2);
//   }
// }

BOOST_AUTO_TEST_CASE(acrobot_rollout) {
  size_t T = 100; // one second
  // auto dyn = mk<Dynamics_acrobot>();
  auto model = mk<Model_acrobot>();

  std::vector<Eigen::VectorXd> us;
  std::vector<Eigen::VectorXd> xs;
  for (size_t i = 0; i < T; i++) {
    us.push_back(model->params.max_torque * .01 * Eigen::VectorXd::Random(1));
  }

  // rollout

  Eigen::VectorXd xnext(4);
  Eigen::VectorXd xold(4);
  xold << 0, 0, 0, 0;
  xs.push_back(xold);
  double dt = .01;

  for (size_t i = 0; i < T; i++) {
    std::cout << "u " << us.at(i).format(FMT) << std::endl;
    std::cout << "xold " << xold.format(FMT) << std::endl;
    model->step(xnext, xold, us.at(i), dt);
    std::cout << "xnext " << xnext.format(FMT) << std::endl;
    xold = xnext;
    xs.push_back(xold);
  }

  std::cout << "final state" << xs.back().format(FMT) << std::endl;
}

BOOST_AUTO_TEST_CASE(acrobot_rollout_free) {
  auto model = mk<Model_acrobot>();
  double dt = .01;
  size_t T = 1. / dt; // one second

  std::vector<Eigen::VectorXd> us;
  std::vector<Eigen::VectorXd> xs;
  for (size_t i = 0; i < T; i++) {
    us.push_back(Eigen::VectorXd::Zero(1));
  }

  // rollout

  Eigen::VectorXd xnext(4);
  Eigen::VectorXd xold(4);
  {
    xold << 2.8, 0, 0, 0;
    xs.push_back(xold);
    double original_energy = model->calcEnergy(xold);
    for (size_t i = 0; i < T; i++) {
      std::cout << "i: " << i << std::endl;
      std::cout << "u " << us.at(i).format(FMT) << std::endl;
      std::cout << "xold " << xold.format(FMT) << std::endl;
      model->step(xnext, xold, us.at(i), dt);
      std::cout << "xnext " << xnext.format(FMT) << std::endl;
      xold = xnext;
      model->calcEnergy(xold);
      xs.push_back(xold);
    }

    double last_energy = model->calcEnergy(xs.back());
    BOOST_TEST(std::abs(original_energy - last_energy) <
               4); // euler integration is very bad here!!
  }

  {
    std::cout << "R4 " << std::endl;
    xs.clear();
    xold << 2.8, 0, 0, 0;
    double original_energy = model->calcEnergy(xold);
    for (size_t i = 0; i < T; i++) {
      std::cout << "i: " << i << std::endl;
      std::cout << "u " << us.at(i).format(FMT) << std::endl;
      std::cout << "xold " << xold.format(FMT) << std::endl;
      model->stepR4(xnext, xold, us.at(i), dt);
      std::cout << "xnext " << xnext.format(FMT) << std::endl;
      xold = xnext;
      model->calcEnergy(xold);
      xs.push_back(xold);
    }
    double last_energy = model->calcEnergy(xs.back());
    BOOST_TEST(std::abs(original_energy - last_energy) < 1e-2);
  }

  // std::cout << "final state" << xs.back().format(FMT)
  // << std::endl;

  // dyn->max_torque =
}

BOOST_AUTO_TEST_CASE(t_qintegrate) {

  Eigen::Quaterniond q = Eigen::Quaterniond(0, 0, 0, 1);
  double dt = .01;
  Eigen::Vector3d omega{0, 0, 1};

  Eigen::Vector4d out;
  Eigen::Vector4d ye;
  Eigen::Vector4d deltaQ;
  __get_quat_from_ang_vel_time(omega * dt, deltaQ, nullptr);
  quat_product(q.coeffs(), deltaQ, out, nullptr, nullptr);

  std::cout << "out\n" << out << std::endl;

  Eigen::MatrixXd JqD(4, 4);
  double eps = 1e-6;
  for (size_t i = 0; i < 4; i++) {
    Eigen::Vector4d qe;
    // Eigen::Vector3d ye;
    qe = q.coeffs();
    qe(i) += eps;
    // qe.normalize();

    Eigen::Vector4d ye;
    Eigen::Vector4d deltaQ;
    __get_quat_from_ang_vel_time(omega * dt, deltaQ, nullptr);
    quat_product(qe, deltaQ, ye, nullptr, nullptr);
    auto df = (ye - out) / eps;
    JqD.col(i) = df;
  }

  Eigen::MatrixXd JomegaD(4, 3);
  for (size_t i = 0; i < 3; i++) {
    Eigen::Vector3d omegae;
    omegae = omega;
    omegae(i) += eps;
    Eigen::Vector4d ye;
    Eigen::Vector4d deltaQ;
    __get_quat_from_ang_vel_time(omegae * dt, deltaQ, nullptr);
    quat_product(q.coeffs(), deltaQ, ye, nullptr, nullptr);

    auto df = (ye - out) / eps;
    JomegaD.col(i) = df;
  }

  std::cout << "omega" << std::endl;
  std::cout << JomegaD << std::endl;
  std::cout << "q" << std::endl;
  std::cout << JqD << std::endl;
  // TODO: check the diffs against analytic!!
}

BOOST_AUTO_TEST_CASE(t_quat_product) {

  Eigen::Vector4d p{1, 2, 3, 4};
  p.normalize();

  Eigen::Vector4d q{1, .2, .3, .4};
  p.normalize();

  Eigen::Vector4d out;
  Eigen::Matrix4d Jp;
  Eigen::Matrix4d Jq;

  quat_product(p, q, out, &Jp, &Jq);

  Eigen::Quaterniond out_eigen = Eigen::Quaterniond(p) * Eigen::Quaterniond(q);

  bool check1 = (out_eigen.coeffs() - out).cwiseAbs().maxCoeff() < 1e-10;

  if (!check1) {

    std::cout << "out_eigen" << std::endl;
    std::cout << out_eigen.coeffs() << std::endl;
    std::cout << "out" << std::endl;
    std::cout << out << std::endl;
    std::cout << "out_eigen.coeffs() - out" << std::endl;
    std::cout << out_eigen.coeffs() - out << std::endl;
  }

  CHECK(check1, AT);

  Eigen::Matrix4d __Jp;
  Eigen::Matrix4d __Jq;

  Eigen::Matrix4d JpD;
  Eigen::Matrix4d JqD;

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        quat_product(x, q, y, &__Jp, &__Jq);
      },
      p, 4, JpD);

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        quat_product(p, x, y, &__Jp, &__Jq);
      },
      p, 4, JqD);

  double eps = 1e-6;

  bool check2 = (Jq - JqD).cwiseAbs().maxCoeff() < 10 * eps;

  if (!check2) {

    std::cout << "Jq" << std::endl;
    std::cout << Jq << std::endl;
    std::cout << "JqD" << std::endl;
    std::cout << JqD << std::endl;
    std::cout << "Jq - JqD" << std::endl;
    std::cout << Jq - JqD << std::endl;
  }
  CHECK(check2, AT);

  bool check3 = (Jp - JpD).cwiseAbs().maxCoeff() < 10 * eps;

  if (!check3) {

    std::cout << "Jp" << std::endl;
    std::cout << Jp << std::endl;
    std::cout << "JpD" << std::endl;
    std::cout << JpD << std::endl;
    std::cout << "Jp - JpD" << std::endl;
    std::cout << Jp - JpD << std::endl;
  }
  CHECK(check3, AT);
}

BOOST_AUTO_TEST_CASE(exp_map_quat) {

  using Matrix43d = Eigen::Matrix<double, 4, 3>;
  {
    Eigen::Vector3d v(-.1, .1, .2);
    Eigen::Vector4d q;
    Matrix43d J(4, 3);
    Matrix43d Jd(4, 3);

    __get_quat_from_ang_vel_time(v, q, &J);

    std::cout << "q " << q.format(FMT) << std::endl;

    Eigen::Quaterniond ref;
    ref = get_quat_from_ang_vel_time(v);
    std::cout << "ref " << ref.coeffs().format(FMT) << std::endl;

    finite_diff_jac(
        [&](const Eigen::VectorXd &xx, Eigen::Ref<Eigen::VectorXd> y) {
          return __get_quat_from_ang_vel_time(xx, y);
        },
        v, 4, Jd, 1e-8);

    std::cout << "error \n"
              << (Jd - J) << std::endl
              << "norm " << (Jd - J).norm() << std::endl;

    BOOST_TEST((ref.coeffs() - q).norm() <= 1e-7);
    BOOST_TEST((Jd - J).norm() <= 1e-7);
  }

  {
    Eigen::Vector3d v(-.1, .1, .2);
    v *= 1e-12;
    Eigen::Vector4d q;

    Matrix43d J(4, 3);
    Matrix43d Jd(4, 3);
    __get_quat_from_ang_vel_time(v, q, &J);

    std::cout << "q " << q.format(FMT) << std::endl;

    Eigen::Quaterniond ref;
    ref = get_quat_from_ang_vel_time(v);
    std::cout << "ref " << ref.coeffs().format(FMT) << std::endl;

    finite_diff_jac(
        [&](const Eigen::VectorXd &xx, Eigen::Ref<Eigen::VectorXd> y) {
          return __get_quat_from_ang_vel_time(xx, y);
        },
        v, 4, Jd, 1e-9);

    std::cout << "error \n"
              << (Jd - J) << std::endl
              << "norm " << (Jd - J).norm() << std::endl;

    BOOST_TEST((Jd - J).norm() <= 1e-7);
    BOOST_TEST((ref.coeffs() - q).norm() <= 1e-7);
  }

  {
    Eigen::Vector3d v(0., 0., 0.);
    v *= 1e-12;
    Eigen::Vector4d q;

    Matrix43d J(4, 3);
    Matrix43d Jd(4, 3);
    __get_quat_from_ang_vel_time(v, q, &J);

    std::cout << "q " << q.format(FMT) << std::endl;

    Eigen::Quaterniond ref;
    ref = get_quat_from_ang_vel_time(v);
    std::cout << "ref " << ref.coeffs().format(FMT) << std::endl;

    finite_diff_jac(
        [&](const Eigen::VectorXd &xx, Eigen::Ref<Eigen::VectorXd> y) {
          return __get_quat_from_ang_vel_time(xx, y);
        },
        v, 4, Jd, 1e-13);

    std::cout << Jd << std::endl;
    std::cout << J << std::endl;
    std::cout << "error \n"
              << (Jd - J) << std::endl
              << "norm " << (Jd - J).norm() << std::endl;

    BOOST_TEST((Jd - J).norm() <= 1e-7);
    BOOST_TEST((ref.coeffs() - q).norm() <= 1e-7);
  }
}

BOOST_AUTO_TEST_CASE(linear_interpolation) {

  Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(10, 0, 9);

  std::vector<Eigen::VectorXd> xs_vec(10, Eigen::VectorXd(2));

  for (size_t i = 0; i < 10; i++) {
    Eigen::VectorXd x(2);
    x << i, 2 * i;
    xs_vec.at(i) = x;
  }

  ptr<Interpolator> path = mk<Interpolator>(ts, xs_vec);

  Eigen::VectorXd x(2);
  Eigen::VectorXd J(2);

  for (size_t i = 0; i < 10; i++) {
    path->interpolate(ts(i), x, J);
    std::cout << x << std::endl;
    BOOST_TEST((x - xs_vec.at(i)).norm() < 1e-12);
  }

  path->interpolate(10, x, J);
  std::cout << x << std::endl;
  path->interpolate(11, x, J);
  std::cout << x << std::endl;

  path->interpolate(.5, x, J);
  std::cout << x << std::endl;
  path->interpolate(.1, x, J);
  std::cout << x << std::endl;

  path->interpolate(.89, x, J);
  std::cout << x << std::endl;

  // TODO: add some test!
}

BOOST_AUTO_TEST_CASE(t_normalize) {
  Eigen::Vector4d q(1, 2, 1, 2.);
  Eigen::Vector4d y;
  Eigen::Matrix4d Jq;

  normalize(q, y, Jq);

  Eigen::Matrix4d __Jq;
  Eigen::MatrixXd JqD(4, 4);
  double eps = 1e-6;

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        return normalize(x, y, __Jq);
      },
      q, 4, JqD);

  bool check2 = (Jq - JqD).cwiseAbs().maxCoeff() < 10 * eps;

  if (!check2) {
    std::cout << "Jq" << std::endl;
    std::cout << Jq << std::endl;
    std::cout << "JqD" << std::endl;
    std::cout << JqD << std::endl;
    std::cout << "Jq - JqD" << std::endl;
    std::cout << Jq - JqD << std::endl;
    CHECK(((Jq - JqD).cwiseAbs().maxCoeff() < 10 * eps), AT);
  }
}

BOOST_AUTO_TEST_CASE(matrix_rotation) {
  // very big error. Compute the rotation of a
  // vector. check with finite diff.

  Eigen::MatrixXd Jq(3, 4);
  Eigen::Matrix3d Ja;

  // Eigen::Vector4d q(1, 2, 1, 2.);
  Eigen::Vector4d q(0, 0, 0, 1.);
  q.normalize();
  Eigen::Vector3d a(1, 2, 3);
  Eigen::Vector3d y;

  rotate_with_q(q, a, y, Jq, Ja);

  // with finite diff

  Eigen::MatrixXd __Jq(3, 4);
  Eigen::Matrix3d __Ja;
  Eigen::MatrixXd JqD(3, 4);
  double eps = 1e-6;
  for (size_t i = 0; i < 4; i++) {
    Eigen::Vector4d qe;
    Eigen::Vector3d ye;
    qe = q;
    qe(i) += eps;
    // qe.normalize();
    rotate_with_q(qe, a, ye, __Jq, __Ja);
    auto df = (ye - y) / eps;
    std::cout << "ye " << ye << std::endl;
    JqD.col(i) = df;
  }

  Eigen::Matrix3d JaD;
  for (size_t i = 0; i < 3; i++) {
    Eigen::Vector3d ae;
    Eigen::Vector3d ye;
    ae = a;
    ae(i) += eps;
    rotate_with_q(q, ae, ye, __Jq, __Ja);
    auto df = (ye - y) / eps;
    JaD.col(i) = df;
  }

  bool check1 = (Ja - JaD).cwiseAbs().maxCoeff() < 10 * eps;
  bool check2 = (Jq - JqD).cwiseAbs().maxCoeff() < 10 * eps;

  if (!check1) {
    std::cout << "Ja" << std::endl;
    std::cout << Ja << std::endl;
    std::cout << "JaD" << std::endl;
    std::cout << JaD << std::endl;
    std::cout << "Ja - JaD" << std::endl;
    std::cout << Ja - JaD << std::endl;
    CHECK(((Ja - JaD).cwiseAbs().maxCoeff() < 10 * eps), AT);
  }

  if (!check2) {
    std::cout << "Jq" << std::endl;
    std::cout << Jq << std::endl;
    std::cout << "JqD" << std::endl;
    std::cout << JqD << std::endl;
    std::cout << "Jq - JqD" << std::endl;
    std::cout << Jq - JqD << std::endl;
    CHECK(((Jq - JqD).cwiseAbs().maxCoeff() < 10 * eps), AT);
  }
}

BOOST_AUTO_TEST_CASE(t_slerp) {

  Eigen::Vector4d a(1, 0, 0, 0);
  Eigen::Vector4d v(0, 0, 0, 1);

  Eigen::Vector4d out(0, 0, 0, 1);

  std::cout << Eigen::Quaterniond(a).slerp(0, Eigen::Quaterniond(v)).coeffs()
            << std::endl;
  std::cout << Eigen::Quaterniond(a).slerp(0.5, Eigen::Quaterniond(v)).coeffs()
            << std::endl;
  std::cout << Eigen::Quaterniond(a).slerp(1., Eigen::Quaterniond(v)).coeffs()
            << std::endl;
}

BOOST_AUTO_TEST_CASE(tcol_unicycle) {

  auto env = std ::string(base_path) + "envs/unicycle1_v0/parallelpark_0.yaml";

  Problem problem;
  problem.read_from_yaml(env.c_str());

  auto unicycle = Model_unicycle1();
  load_env(unicycle, problem);

  Eigen::Vector3d x(.7, .8, 0);

  CollisionOut col;

  unicycle.collision_distance(x, col);

  BOOST_CHECK(std::fabs(col.distance - .25) < 1e-7);

  x = Eigen::Vector3d(1.9, .3, 0);
  unicycle.collision_distance(x, col);

  BOOST_CHECK(std::fabs(col.distance - .3) < 1e-7);

  col.write(std::cout);

  x = Eigen::Vector3d(1.5, .3, .1);
  unicycle.collision_distance(x, col);
  col.write(std::cout);

  BOOST_CHECK(std::fabs(col.distance - (-0.11123)) < 1e-5);
}

BOOST_AUTO_TEST_CASE(col_car_with_trailer) {

  auto env = std::string(base_path) + "envs/car1_v0/bugtrap_0.yaml";
  auto car = Model_car_with_trailers();

  Problem problem;
  problem.read_from_yaml(env.c_str());

  load_env(car, problem);

  Eigen::Vector4d x(3.4, 3, 3.14, 3.14);
  CollisionOut col;
  car.collision_distance(x, col);
  col.write(std::cout);

  x = Eigen::Vector4d(5.2, 3, 1.55, 1.55);
  car.collision_distance(x, col);
  col.write(std::cout);

  x = Eigen::Vector4d(3.6, 1, 1.55, 1.55);
  car.collision_distance(x, col);
  col.write(std::cout);

  x = Eigen::Vector4d(5.2, 3, 1.55, .3);
  car.collision_distance(x, col);
  col.write(std::cout);
}

BOOST_AUTO_TEST_CASE(tcol_quad3d) {

  auto env = std::string(base_path) + "envs/quadrotor_v0/quad_one_obs.yaml";

  Problem problem;
  problem.read_from_yaml(env.c_str());

  Eigen::VectorXd x(13);
  auto quad3d = Model_quad3d();
  load_env(quad3d, problem);
  CollisionOut col;

  x << 1., 1., 1., 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  quad3d.collision_distance(x, col);
  col.write(std::cout);

  BOOST_TEST(std::fabs(col.distance - 0.824745) < 1e-5);
  BOOST_TEST(std::fabs(col.distance - (col.p1 - col.p2).norm() < 1e-5));

  x << 1.2, 1.5, 2., 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  quad3d.collision_distance(x, col);
  col.write(std::cout);

  BOOST_TEST(std::fabs(col.distance - (-0.0999999)) < 1e-5);

  x << 5., 5., 1., 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  quad3d.collision_distance(x, col);
  col.write(std::cout);
  BOOST_TEST(std::fabs(col.distance - 0.824745) < 1e-5);

  x << 3., 1., 3., 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  quad3d.collision_distance(x, col);
  col.write(std::cout);
  BOOST_TEST(std::fabs(col.distance - (.1)) < 5 * 1e-5);
}

BOOST_AUTO_TEST_CASE(col_acrobot) {

  Problem problem;
  auto env = std::string(base_path) + "envs/acrobot_v0/swing_up_obs.yaml";

  problem.read_from_yaml(env.c_str());
  const double tol = 1e-4;

  Eigen::Vector4d x;
  auto acrobot = Model_acrobot();
  load_env(acrobot, problem);

  CollisionOut col;
  x << 0, 0, 0, 0;
  acrobot.collision_distance(x, col);
  col.write(std::cout);

  BOOST_TEST(std::fabs(col.distance - 1.59138) < tol);
  BOOST_TEST(std::fabs(col.distance - (col.p1 - col.p2).norm() < 1e-5));

  x << 3.14159, 0, 0, 0;
  acrobot.collision_distance(x, col);
  col.write(std::cout);
  BOOST_TEST(std::fabs(col.distance - 1.1) < tol);

  x << M_PI / 2., M_PI / 2., 0, 0;
  acrobot.collision_distance(x, col);
  col.write(std::cout);
  BOOST_TEST(std::fabs(col.distance - 1.59138) < tol);

  x << 2.37, 1.4, 0, 0;
  acrobot.collision_distance(x, col);
  col.write(std::cout);
  BOOST_TEST(std::fabs(col.distance - 1.21897) < tol);
}

// BOOST_AUTO_TEST_CASE(col_quad3d_v2) {
//
//   Problem
//   problem("../benchmark/quadrotor_0/obstacle_flight.yaml");
//
//   std::shared_ptr<Model_robot> robot =
//       robot_factory(robot_type_to_path(problem.robotType).c_str());
//   load_env(*robot, problem);
//
//   Eigen::VectorXd x(13);
//   x.setZero();
//   x.head(3) << -.1, -1.3, 1.;
//   x(6) = 1.;
//
//   CollisionOut out;
//   robot->collision_distance(x, out);
//
//   out.write(std::cout);
// };

BOOST_AUTO_TEST_CASE(t_serialization) {

  Trajectory traj1, traj2;

  auto v1 = Eigen::Vector3d(0, 0, 1);
  auto v2 = Eigen::Vector3d(0, 1, 1);
  auto v3 = Eigen::Vector3d(1, 0, 1);

  auto a1 = Eigen::Vector2d(0, 0);
  auto a2 = Eigen::Vector2d(0, 1);

  traj1.states = std::vector<Eigen::VectorXd>{v1, v2, v3};
  traj1.actions = std::vector<Eigen::VectorXd>{a1, a2};

  std::filesystem::create_directory("/tmp/croco/");

  auto filename = "/tmp/croco/test_serialize.bin";

  traj1.save_file_boost(filename);

  traj2.load_file_boost(filename);

  BOOST_TEST(traj1.distance(traj2) < 1e-10);
  BOOST_TEST(traj2.distance(traj1) < 1e-10);

  Trajectories trajs_A{.data = {traj1, traj2}};
  Trajectories trajs_B{.data = {traj1, traj2}};

  auto filename_trajs = "/tmp/croco/test_serialize_trajs.bin";

  trajs_A.save_file_boost(filename_trajs);
  trajs_B.load_file_boost(filename_trajs);

  BOOST_TEST(trajs_A.data.at(0).distance(trajs_B.data.at(0)) < 1e-10);
  BOOST_TEST(trajs_A.data.at(1).distance(trajs_B.data.at(1)) < 1e-10);
}

BOOST_AUTO_TEST_CASE(t_Integrator2_2d) {
  auto model = mk<Integrator2_2d>();

  Eigen::VectorXd x0(4), u0(2);
  x0 << .1, .2, .3, .4;
  u0 << -.1, .2;

  Eigen::MatrixXd Jx_diff(4, 4), Ju_diff(4, 2), Jx(4, 4), Ju(4, 2);
  Jx.setZero();
  Ju.setZero();
  Jx_diff.setZero();
  Ju_diff.setZero();

  model->calcDiffV(Jx, Ju, x0, u0);

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x, u0);
      },
      x0, 4, Jx_diff);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x0, u);
      },
      u0, 4, Ju_diff);

  BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
  BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);
}

BOOST_AUTO_TEST_CASE(t_Integrator1_2d) {
  auto model = mk<Integrator1_2d>();

  Eigen::VectorXd x0(2), u0(2);
  x0 << .1, .2;
  u0 << -.1, .2;

  Eigen::MatrixXd Jx_diff(2, 2), Ju_diff(2, 2), Jx(2, 2), Ju(2, 2);
  Jx.setZero();
  Ju.setZero();
  Jx_diff.setZero();
  Ju_diff.setZero();

  model->calcDiffV(Jx, Ju, x0, u0);

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x, u0);
      },
      x0, 2, Jx_diff);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x0, u);
      },
      u0, 2, Ju_diff);

  BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
  BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);
}

BOOST_AUTO_TEST_CASE(t_joint_robot) {

  std::vector<std::shared_ptr<Model_robot>> robots;
  robots.push_back(std::make_unique<Model_unicycle1>());
  robots.push_back(std::make_unique<Model_unicycle2>());
  robots.push_back(std::make_unique<Model_car_with_trailers>());
  robots.push_back(std::make_unique<Integrator2_2d>());
  robots.push_back(std::make_unique<Integrator1_2d>());

  auto model =
      mk<Joint_robot>(robots, Eigen::Vector2d(2, 2), Eigen::Vector2d(2, 2));

  int nx = model->nx;
  int nu = model->nu;
  Eigen::VectorXd x0(nx), u0(nu);
  x0.setRandom();
  u0.setRandom();
  // x0 << .1, .2 ;
  //
  // u0 << -.1, .2;

  Eigen::MatrixXd Jx_diff(nx, nx), Ju_diff(nx, nu), Jx(nx, nx), Ju(nx, nu);
  Jx.setZero();
  Ju.setZero();
  Jx_diff.setZero();
  Ju_diff.setZero();

  model->calcDiffV(Jx, Ju, x0, u0);

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

  BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
  BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);
}

BOOST_AUTO_TEST_CASE(t_joint_robot_env) {

  std::string env =
      base_path "envs/multirobot/example/gen_p10_n2_6_hetero.yaml";

  Problem problem(env);

  std::string robot_type = problem.robotType;

  std::string _base_path = base_path "models/";
  std::unique_ptr<Model_robot> joint_robot = joint_robot_factory(
      problem.robotTypes, _base_path, problem.p_lb, problem.p_ub);

  load_env(*joint_robot, problem);

  // check that start and goal are collision free.
  //

  std::cout << "start " << problem.start.format(FMT) << std::endl;
  std::cout << "goal " << problem.goal.format(FMT) << std::endl;

  CollisionOut out;
  joint_robot->collision_distance(problem.start, out);

  BOOST_TEST(out.distance > 0);

  CollisionOut out2;
  joint_robot->collision_distance(problem.goal, out);

  BOOST_TEST(out.distance > 0);

  {
    // Modify the state. Now the two car are one on top of each other.
    Eigen::VectorXd x = problem.start;

    x(3) = x(0) + .01;
    x(4) = x(1) + .01;

    CollisionOut out3;
    joint_robot->collision_distance(x, out);
    BOOST_TEST(out.distance < 0);
  }

  {
    // Modify the state. Put on car on top of one obstacle
    Eigen::VectorXd x = problem.start;

    x(3) = problem.obstacles.at(0).center(0) + .001;
    x(4) = problem.obstacles.at(0).center(1) + .001;

    CollisionOut out3;
    joint_robot->collision_distance(x, out);
    BOOST_TEST(out.distance < 0);
  }
}

BOOST_AUTO_TEST_CASE(t_check_traj_swap2_trailer) {

  std::string env = base_path "envs/multirobot/example/swap2_trailer.yaml";

  Problem problem(env);
  std::string robot_type = problem.robotType;

  std::string _base_path = base_path "models/";

  problem.models_base_path = _base_path;
  std::unique_ptr<Model_robot> joint_robot = joint_robot_factory(
      problem.robotTypes, _base_path, problem.p_lb, problem.p_ub);

  load_env(*joint_robot, problem);

  {
    std::string result_file =
        base_path "envs/multirobot/results/swap2_trailer_solution.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible);
  }

  {

    std::string result_file =
        base_path "envs/multirobot/results/swap2_trailer_db.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible == false);
  }
}

BOOST_AUTO_TEST_CASE(t_check_traj_swap4_unicycle) {

  std::string env = base_path "envs/multirobot/example/swap4_unicycle.yaml";

  Problem problem(env);
  std::string robot_type = problem.robotType;

  std::string _base_path = base_path "models/";

  problem.models_base_path = _base_path;
  std::unique_ptr<Model_robot> joint_robot = joint_robot_factory(
      problem.robotTypes, _base_path, problem.p_lb, problem.p_ub);

  load_env(*joint_robot, problem);

  {
    std::string result_file =
        base_path "envs/multirobot/results/swap4_unicycle_solution.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible);
  }

  {

    std::string result_file =
        base_path "envs/multirobot/results/swap4_unicycle_db.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible == false);
  }
}

BOOST_AUTO_TEST_CASE(t_check_traj_swap2_unicycle2) {

  std::string env = base_path "envs/multirobot/example/swap2_unicycle2.yaml";

  Problem problem(env);
  std::string robot_type = problem.robotType;

  std::string _base_path = base_path "models/";

  problem.models_base_path = _base_path;
  std::unique_ptr<Model_robot> joint_robot = joint_robot_factory(
      problem.robotTypes, _base_path, problem.p_lb, problem.p_ub);

  load_env(*joint_robot, problem);

  {
    std::string result_file =
        base_path "envs/multirobot/results/swap2_unicycle2_solution.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible);
  }

  {

    std::string result_file =
        base_path "envs/multirobot/results/swap2_unicycle2_db.yaml";
    MultiRobotTrajectory multirobot_traj;

    multirobot_traj.read_from_yaml(result_file.c_str());

    Trajectory traj;

    traj = multirobot_traj.transform_to_joint_trajectory();
    traj.start = problem.start;
    traj.goal = problem.goal;

    std::vector<std::string> robotTypes = problem.robotTypes;

    std::cout << "robot types are " << std::endl;

    std::shared_ptr<Model_robot> robot = joint_robot_factory(
        robotTypes, problem.models_base_path, problem.p_lb, problem.p_ub);

    load_env(*robot, problem);

    bool verbose = true;

    Feasibility_thresholds feasibility_thresholds;

    traj.check(robot, verbose);
    traj.update_feasibility(feasibility_thresholds);

    BOOST_TEST(traj.feasible == false);
  }
}

BOOST_AUTO_TEST_CASE(t_differential_drive) {
  auto model = mk<Model_differential_drive>();

  Eigen::VectorXd x0(3), u0(2);
  x0 << .1, .2, .3;
  u0 << -.1, .2;

  Eigen::MatrixXd Jx_diff(3, 3), Ju_diff(3, 2), Jx(3, 3), Ju(3, 2);
  Jx.setZero();
  Ju.setZero();
  Jx_diff.setZero();
  Ju_diff.setZero();

  model->calcDiffV(Jx, Ju, x0, u0);

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x, u0);
      },
      x0, 3, Jx_diff);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x0, u);
      },
      u0, 3, Ju_diff);

  BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
  BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);
}
