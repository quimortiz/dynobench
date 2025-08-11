#pragma once

#include "dynobench/dyno_macros.hpp"
#include "dynobench/robot_models_base.hpp"
#include <mujoco/mujoco.h>

inline Eigen::Map<Eigen::VectorXd> mj_Vec(double* data, int n)              // read-write
{
    return Eigen::Map<Eigen::VectorXd>(data, n);
}

inline Eigen::Map<const Eigen::VectorXd> mj_VecConst(const double* data, int n)   // read-only
{
    return Eigen::Map<const Eigen::VectorXd>(data, n);
}


//-------------------------------------------------------------
//  MuJoCo  (w x y z)  ->  Dynobench (x y z w)
//-------------------------------------------------------------
inline void mj2dyno_quat(const Eigen::Ref<const Eigen::VectorXd>& mj,
                        int  nbodies,
                        Eigen::Ref<Eigen::VectorXd>       dyno)
{
    for (int b = 0; b < nbodies; ++b)
    {
      const int idx = 7 * b;          // start of this body

      dyno.segment<3>(idx) = mj.segment<3>(idx);
      dyno.segment<4>(idx + 3) <<
            mj[idx + 4],      // x
            mj[idx + 5],      // y
            mj[idx + 6],      // z
            mj[idx + 3];      // w
    }
}

inline void dyno2mj_quat(const Eigen::Ref<const Eigen::VectorXd>& dyno,
                        int  nbodies,
                        Eigen::Ref<Eigen::VectorXd>       mj_out)
{
    for (int b = 0; b < nbodies; ++b)
    {
      const int idx = 7 * b;
      mj_out.segment<3>(idx) = dyno.segment<3>(idx);
      mj_out[idx + 3] = dyno[idx + 6];  // w
      mj_out[idx + 4] = dyno[idx + 3];  // x
      mj_out[idx + 5] = dyno[idx + 4];  // y
      mj_out[idx + 6] = dyno[idx + 5];  // z
    }
}

namespace dynobench {

struct MujocoQuad_params {

  MujocoQuad_params(const char *file) { read_from_yaml(file); }
  MujocoQuad_params() = default;


  double size = .1;    // radius
  std::string model_path; // absolute path of the model "/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/xml/1cfs_payload_tendons.xml";
  double max_vel = 4;
  double max_angular_vel = 8;
  double max_acc = 25;
  double max_angular_acc = 20;

  double mass = 0.0356; // mass of the quadrotor [kg]
  double g = 9.81;
  double max_f = 1.3;        // thrust to weight ratio -- Khaled DONE
  double arm_length = 0.046; // m
  double t2t = 0.006;        // thrust-to-torque ratio
  double dt = .01;
  bool visualize = true;
  Eigen::Vector3d J_v =
    Eigen::Vector3d(16.571710e-6, 16.655602e-6, 29.261652e-6);
  std::string shape = "sphere";
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, .5, .1, .1);
  Eigen::VectorXd u_ub;
  Eigen::VectorXd u_lb;

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(filename, af) << std::endl;
    out << be << STR(model_path, af) << std::endl;

    out << be << STR(size, af) << std::endl;

    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_acc, af) << std::endl;
    out << be << STR(g, af) << std::endl;
    out << be << STR(max_f, af) << std::endl;
    out << be << STR(arm_length, af) << std::endl;
    out << be << STR(t2t, af) << std::endl;
    out << be << STR(dt, af) << std::endl;
    out << be << STR(shape, af) << std::endl;
    out << be << STR(mass, af) << std::endl;

    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct Model_MujocoQuad : Model_robot {
  // state x: [qpos qvel]
  // xpos_{name}: [position quaternion]_{name of the body} \in R^{7x1}
  // xvel_{name}: [velocity ang velocity in body frame]_{name of the body} \in R^{6x1}
  // qpos: [xpos_{quad}] \in R^{7}
  // qvel: [xvel_{quad1}] \in R^{6}


  // Regularization in the optimization problem.
  // you have to make this genereal
  Eigen::VectorXd state_weights;
  Eigen::VectorXd state_ref;

  std::vector<std::unique_ptr<fcl::CollisionObjectd>>
      collision_objects; // QUIM : TODO move this to the base class!

  virtual ~Model_MujocoQuad() = default;
  struct Data {
    Eigen::Vector3d f_u;
    Eigen::Vector3d tau_u;
    Eigen::Matrix<double, 13, 1> xnext;
    Matrix34 Jx;
    Eigen::Matrix3d Ja;
  } data;

  // Eigen::VectorXd ff; // TODO: remember to allocate memory in constructor!
  MujocoQuad_params params;

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x.segment<6>(7).setZero();
  }

  double arm;
  double g = 9.81;

  double u_nominal;
  double m_inv;
  double mass;
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
  mjModel* m;
  mjData* d;
  bool viewer_ready_ = false;
  mjvScene   scn_;
  mjrContext con_;
  mjvCamera  cam_;
  mjvOption  opt_;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  Model_MujocoQuad(const Model_MujocoQuad &) = default;

  Model_MujocoQuad(const char *file,
                        const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                        const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_MujocoQuad(MujocoQuad_params(file), p_lb, p_ub) {}

  Model_MujocoQuad(
      const MujocoQuad_params &params = MujocoQuad_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());


  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    xout.segment(3 , 4).normalize();
  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override;

  virtual void
  transform_primitive_last_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                                 const std::vector<Eigen::VectorXd> &xs_in,
                                 const std::vector<Eigen::VectorXd> &us_in,
                                 Eigen::Ref<Eigen::VectorXd> x_out) override {

    assert(p.size() == 3 || 6);

    if (p.size() == 3) {
      Model_robot::transform_primitive_last_state(p, xs_in, us_in, x_out);

    } else {
      x_out = xs_in.back();
      x_out.head<3>() +=
          p.head<3>() + us_in.size() * ref_dt * p.tail<3>(); // velocity
      x_out.segment<3>(7) += p.tail<3>();                    // velocity
    }
  }

  virtual void transform_primitive_last_state_backward(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in,
      Eigen::Ref<Eigen::VectorXd> x_out) override {

    assert(p.size() == 3 || 6);

    if (p.size() == 3) {
      Model_robot::transform_primitive_last_state(p, xs_in, us_in, x_out);

    } else {
      x_out.head<3>() = xs_in.back().head<3>() + p.head<3>() -
                        (xs_in.size() - 1) * ref_dt * p.tail<3>();
      x_out.segment<4>(3) = xs_in.back().segment<4>(3);
      x_out.segment<3>(7) = xs_in.back().segment<3>(7) + p.tail<3>();
      x_out.tail<3>() = xs_in.back().tail<3>();
    }
  }

  virtual void transform_primitiveDirect(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) {

    assert(is_valid_fun == nullptr);
    assert(num_valid_states == nullptr);
    assert(p.size() == 6);

    for (size_t i = 0; i < traj_out.get_size(); i++) {
      traj_out.get_state(i).head<3>() =
          xs_in[i].head<3>() + p.head<3>() + i * ref_dt * p.tail<3>();
      traj_out.get_state(i).segment<4>(3) = xs_in[i].segment<4>(3);
      traj_out.get_state(i).segment<3>(7) =
          xs_in[i].segment<3>(7) + p.tail<3>();
      traj_out.get_state(i).tail<3>() = xs_in[i].tail<3>();
      if (i < traj_out.get_size() - 1) {
        traj_out.get_action(i).head<4>() = us_in[i].head<4>();
      }
    }
  }

  virtual void transform_primitiveDirectReverse(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      // std::vector<Eigen::VectorXd> &xs_out,
      // std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) {

    assert(is_valid_fun == nullptr);
    assert(num_valid_states == nullptr);
    assert(xs_in.size());
    assert(xs_in.size() == us_in.size() + 1);

    for (size_t i = 0; i < traj_out.get_size(); i++) {
      traj_out.get_state(i).head<3>() =
          xs_in[i].head<3>() + p.head<3>() - i * ref_dt * p.tail<3>();
      traj_out.get_state(i).segment<4>(3) = xs_in[i].segment<4>(3);
      traj_out.get_state(i).segment<3>(7) =
          xs_in[i].segment<3>(7) + p.tail<3>();
      traj_out.get_state(i).tail<3>() = xs_in[i].tail<3>();
      if (i < traj_out.get_size() - 1) {
        traj_out.get_action(i).head<4>() = us_in[i].head<4>();
      }
    }
  }

  void virtual transform_primitive(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      // std::vector<Eigen::VectorXd> &xs_out,
      // std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) override;

  virtual void offset(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> p) override {
    DYNO_CHECK_EQ(p.size(), 6, AT);
    if (adapt_vel) {
      p.head<3>() = xin.head<3>();
      p.tail<3>() = xin.segment<3>(7);
    } else {
      Model_robot::offset(xin, p);
    }
  }

  virtual size_t get_offset_dim() override { return adapt_vel ? 6 : 3; }

  virtual void canonical_state(const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    if (adapt_vel) {
      xout = xin;
      xout.head<3>().setZero();
      xout.segment<3>(7).setZero();
    } else {
      Model_robot::canonical_state(xin, xout);
    }
  }

  virtual void transform_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                               const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    CHECK((p.size() == 3 || p.size() == 6), AT);
    if (p.size() == 3) {
      Model_robot::transform_state(p, xin, xout);
    } else if (p.size() == 6) {
      xout = xin;
      xout.head<3>() += p.head<3>();
      xout.segment<3>(7) += p.tail<3>();
    }
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

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override;

  void init_mujoco_viewer() override {
    if (viewer_ready_) return;          // already done

    mjv_defaultScene  (&scn_);
    mjr_defaultContext(&con_);
    mjv_defaultCamera (&cam_);
    mjv_defaultOption (&opt_);

    mjv_makeScene   (m, &scn_, 2000);
    mjr_makeContext (m, &con_, mjFONTSCALE_150);

    viewer_ready_ = true;
  };
  void render(int w, int h) override {
    if (params.visualize && viewer_ready_) {
      mjr_render({0,0,w,h}, &scn_, &con_);
    }
  }
};

} // namespace dynobench
