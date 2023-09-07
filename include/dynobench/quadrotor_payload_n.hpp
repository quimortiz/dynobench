#pragma once

#include "dynobench/croco_macros.hpp"
#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

inline Eigen::VectorXd create_vector(const std::vector<double> &v) {
  Eigen::VectorXd out(v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    out(i) = v[i];
  }
  return out;
}

namespace dynobench {

struct Quad3dpayload_n_params {

  Quad3dpayload_n_params(const char *file) { read_from_yaml(file); }
  Quad3dpayload_n_params() = default;

  int num_robots;// = 2;
  bool point_mass = true;

  double col_size_robot = .15;    // radius
  double col_size_payload = .01; // radius
  //
  //
  //

  double max_vel = 4;
  double max_angular_vel = 8;

  double max_acc = 25;
  double max_angular_acc = 20;

  bool motor_control = true;

  Eigen::VectorXd m; // kg

  Eigen::VectorXd l_payload; // m
  Eigen::VectorXd J_vx; 
  Eigen::VectorXd J_vy; 
  Eigen::VectorXd J_vz; 
  double m_payload = 0.0054; // kg

  double g = 9.81;
  double max_f = 1.3;        // thrust to weight ratio -- Khaled DONE
  double arm_length = 0.046; // m
  double t2t = 0.006;        // thrust-to-torque ratio
  double dt = .01;


  std::string shape = "sphere";
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, 1, .1, .1);
  Eigen::Vector4d u_ub;
  Eigen::Vector4d u_lb;

  Eigen::Vector3d J_v =
      Eigen::Vector3d(16.571710e-6, 16.655602e-6, 29.261652e-6);

  Eigen::VectorXd size = Eigen::Matrix<double, 1, 1>(.4);

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(filename, af) << std::endl;
    out << be << STR(point_mass, af) << std::endl;
    out << be << STR(num_robots, af) << std::endl;

    out << be << STR(col_size_robot, af) << std::endl;
    out << be << STR(col_size_payload, af) << std::endl;

    out << be << STR(m_payload, af) << std::endl;

    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_acc, af) << std::endl;
    out << be << STR(motor_control, af) << std::endl;
    out << be << STR(g, af) << std::endl;
    out << be << STR(max_f, af) << std::endl;
    out << be << STR(arm_length, af) << std::endl;
    out << be << STR(t2t, af) << std::endl;
    out << be << STR(dt, af) << std::endl;
    out << be << STR(shape, af) << std::endl;

    out << be << STR_VV(m, af) << std::endl;
    out << be << STR_VV(l_payload, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(J_v, af)  << std::endl;
    out << be << STR_VV(J_vx, af) << std::endl;
    out << be << STR_VV(J_vy, af) << std::endl;
    out << be << STR_VV(J_vz, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct Model_quad3dpayload_n : Model_robot {

  using Matrix34 = Eigen::Matrix<double, 3, 4>;


  // Regularization in the optimization problem.
  // @KHALED please check this in the constructor!! --
  // you have to make this genereal
  Eigen::VectorXd state_weights;
  Eigen::VectorXd state_ref ;


  std::vector<std::unique_ptr<fcl::CollisionObjectd>>
      collision_objects; // QUIM : TODO move this to the base class!

  virtual ~Model_quad3dpayload_n() = default;

  // KHALED: is this even necessary?
  // I don't think this is necessary. I dont use this in any of the models
  // struct Data {
  //   Eigen::Vector3d f_u;
  //   Eigen::Vector3d tau_u;
  //   Eigen::VectorXd xnext;  // remember to allocate memory in constructor!
  //   Matrix34 Jx;
  //   Eigen::Matrix3d Ja;
  // } data;

  Eigen::VectorXd ff; // TODO: remember to allocate memory in constructor!
  Quad3dpayload_n_params params;

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    NOT_IMPLEMENTED;
  }

  void get_payload_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector3d> out) {
    // NOT_IMPLEMENTED_TODO; // @KHALED
    out = x.head<3>();
  }

  void get_qc_i(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                Eigen::Ref<Eigen::Vector3d> out) {
    // NOT_IMPLEMENTED_TODO; // @KHALED
    // This is for pointmass payload only, if rigid body --> 13+3*i
    int qc_idx = 6 + 6 * i;
    out = x.segment(qc_idx, 3);
  }

  void get_payload_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector3d> out) {
    // NOT_IMPLEMENTED_TODO; // @KHALED
    out = x.segment(3, 3);
  }

  void get_wc_i(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                Eigen::Ref<Eigen::Vector3d> out) {
    CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED
    // 6 for payload pos and vel + 6*i + 3 (for 2 uavs: 3, 9)
    int wc_idx = 6 + 6 * i + 3;
    out = x.segment(wc_idx, 3);
  }

  void get_payload_q(const Eigen::Ref<const Eigen::VectorXd> &x,
                     Eigen::Ref<Eigen::Vector4d> out) {
    NOT_IMPLEMENTED_TODO; // @KHALED: This is for the rigid case: Not
                          // implemented now
  }

  void get_payload_w(const Eigen::Ref<const Eigen::VectorXd> &x,
                     Eigen::Ref<Eigen::Vector3d> out) {
    NOT_IMPLEMENTED_TODO; // @KHALED: RIGID Payload
  }

  void get_robot_w_i(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                     Eigen::Ref<Eigen::Vector3d> out) {
    CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED
    int w_idx = 6 + 6 * params.num_robots + 7 * i + 3;
    out = x.segment(w_idx, 3);
  }

  virtual void get_position_robot_i(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    int i, Eigen::Ref<Eigen::Vector3d> out) {
    CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED

    Eigen::Vector3d payload_pos;
    get_payload_pos(x, payload_pos);
    Eigen::Vector3d qc;
    get_qc_i(x, i, qc);
    out = payload_pos - qc * params.l_payload(i);
  }

  virtual void
  get_orientation_robot_i(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                          Eigen::Ref<Eigen::Vector4d> out) {
    CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED
    int q_idx = 6 + 6 * params.num_robots + 7 * i;
    out = x.segment(q_idx, 4);
  }

  virtual void
  get_position_center_cable(const Eigen::Ref<const Eigen::VectorXd> &x,
                            Eigen::Ref<Eigen::Vector3d> out, int i) {
    CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED

    //
    Eigen::Vector3d payload_pos;
    get_payload_pos(x, payload_pos);
    Eigen::Vector3d qc;
    get_qc_i(x, i, qc);
    out = payload_pos - qc * params.l_payload(i) * 0.5;
  }

  // NOTE: there are infinite solutions to this problem
  // we just take the "smallest" rotation
  // I just way this to update the capsule orientation
  virtual void quaternion_cable_i(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  int i, Eigen::Ref<Eigen::Vector4d> out) {

    CHECK_LEQ(i, params.num_robots - 1, "");
    Eigen::Vector3d from(0., 0., -1.);
    Eigen::Vector3d to;
    get_qc_i(x, i, to);
    out = Eigen::Quaternion<double>::FromTwoVectors(from, to).coeffs();
  }

  double arm;
  double g = 9.81;

  double u_nominal;
  double m_inv;
  double m;
  Eigen::Vector3d inverseJ_v;

  Eigen::Matrix3d inverseJ_M;
  Eigen::Matrix3d J_M;

  Eigen::Matrix3d inverseJ_skew;
  Eigen::Matrix3d J_skew;

  Eigen::Vector3d grav_v;

  Eigen::Matrix4d B0;
  Eigen::Matrix4d B0inv;

  Matrix34 Fu_selection;
  Matrix34 Ftau_selection;

  Matrix34 Fu_selection_B0;
  Matrix34 Ftau_selection_B0;

  const bool adapt_vel = true;
  bool check_inner = true;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  Model_quad3dpayload_n(const Model_quad3dpayload_n &) = default;

  Model_quad3dpayload_n(const char *file,
                        const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                        const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_quad3dpayload_n(Quad3dpayload_n_params(file), p_lb, p_ub) {}

  Model_quad3dpayload_n(
      const Quad3dpayload_n_params &params = Quad3dpayload_n_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual std::map<std::string, std::vector<double>>
  get_info(const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    // state (size): [x_load(3,)  q_cable(3,)   v_load(3,)   w_cable(3,)
    // quat(4,)     w_uav(3)]
    //         idx:  [(0, 1, 2), (3,  4,  5),  (6,  7,  8), (9,  10, 11),
    //         (12,13,14,15), (16, 17, 18)]
    // xout.segment<4>(12).normalize();
    // xout.segment<3>(3).normalize();

    for (int i = 0; i < params.num_robots; ++i) {
      xout.segment(6 + 6 * i, 3).normalize();
      xout.segment(6 + 6 * params.num_robots + 7 * i, 4).normalize();
    }
  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override;

  virtual void
  motorForcesFromThrust(Eigen::Ref<Eigen::VectorXd> f,
                        const Eigen::Ref<const Eigen::VectorXd> tm) {

    // Eigen::Vector4d eta = B0 * u_nominal * f;
    // f_u << 0, 0, eta(0);
    // tau_u << eta(1), eta(2), eta(3);
    f = B0inv * tm / u_nominal;
  }

  virtual void
  transform_primitive(const Eigen::Ref<const Eigen::VectorXd> &p,
                      const std::vector<Eigen::VectorXd> &xs_in,
                      const std::vector<Eigen::VectorXd> &us_in,
                      std::vector<Eigen::VectorXd> &xs_out,
                      std::vector<Eigen::VectorXd> &us_out) override;

  virtual void offset(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> p) override {
    // Not sure what to do here
    NOT_IMPLEMENTED;
  }

  virtual size_t get_offset_dim() override {
    // Not sure what to do here
    NOT_IMPLEMENTED;
  }

  virtual void canonical_state(const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {
    // Not sure what to do here
    NOT_IMPLEMENTED;
  }

  virtual void transform_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                               const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {
    // Not sure what to do here
    NOT_IMPLEMENTED;
  }

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void step(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u,
                    double dt) override;

  virtual void stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u,
                        double dt) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override;
};

} // namespace dynobench
