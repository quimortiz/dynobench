#pragma once

#include "dynobench/dyno_macros.hpp"
#include "dynobench/robot_models_base.hpp"
#include <mujoco/mujoco.h>



// convert MuJoCo quat  [w x y z] → Eigen order [x y z w]
inline Eigen::Vector4d wxyz2xyzw(const double* q_mj)
{
    return {q_mj[1], q_mj[2], q_mj[3], q_mj[0]};
}

// convert back  [x y z w] → [w x y z]
inline void xyzw2wxyz(const Eigen::Vector4d& q_xyzw, double* buf4)
{
    buf4[0] = q_xyzw[3];
    buf4[1] = q_xyzw[0];
    buf4[2] = q_xyzw[1];
    buf4[3] = q_xyzw[2];
}


inline Eigen::Map<Eigen::VectorXd> mjVec(double* data, int n)              // read-write
{
    return Eigen::Map<Eigen::VectorXd>(data, n);
}

inline Eigen::Map<const Eigen::VectorXd> mjVecConst(const double* data, int n)   // read-only
{
    return Eigen::Map<const Eigen::VectorXd>(data, n);
}


//-------------------------------------------------------------
//  MuJoCo  (w x y z)  ->  Dynobench (x y z w)
//-------------------------------------------------------------
inline void mj2dyno_pos(const Eigen::Ref<const Eigen::VectorXd>& mj,
                        int  nbodies,
                        Eigen::Ref<Eigen::VectorXd>       dyno)
{
    for (int b = 0; b < nbodies; ++b)
    {
        const int idx = 7 * b;          // start of this body

        /* copy position -------------------------------------------------- */
        dyno.segment<3>(idx) = mj.segment<3>(idx);

        /* reorder quaternion -------------------------------------------- */
        // mj : [w x y z]   ->   dyno : [x y z w]
        dyno.segment<4>(idx + 3) <<
              mj[idx + 4],      // x
              mj[idx + 5],      // y
              mj[idx + 6],      // z
              mj[idx + 3];      // w
    }
}

//-------------------------------------------------------------
//  Dynobench (x y z w)  ->  MuJoCo  (w x y z)
//-------------------------------------------------------------
inline void dyno2mj_pos(const Eigen::Ref<const Eigen::VectorXd>& dyno,
                        int  nbodies,
                        Eigen::Ref<Eigen::VectorXd>       mj_out)
{
    for (int b = 0; b < nbodies; ++b)
    {
        const int idx = 7 * b;

        /* copy position -------------------------------------------------- */
        mj_out.segment<3>(idx) = dyno.segment<3>(idx);

        /* reorder quaternion -------------------------------------------- */
        // dyno : [x y z w] -> mj : [w x y z]
        mj_out[idx + 3] = dyno[idx + 6];  // w
        mj_out[idx + 4] = dyno[idx + 3];  // x
        mj_out[idx + 5] = dyno[idx + 4];  // y
        mj_out[idx + 6] = dyno[idx + 5];  // z
    }
}

namespace dynobench {

struct MujocoQuadsPayload_params {

  MujocoQuadsPayload_params(const char *file) { read_from_yaml(file); }
  MujocoQuadsPayload_params() = default;

  int num_robots; //

  double col_size_robot = .1;    // radius
  double col_size_payload = .01; // radius
  std::string model_path; // absolute path of the model "/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/xml/1cfs_payload_tendons.xml";
  double max_vel = 4;
  double max_angular_vel = 8;
  double max_acc = 25;
  double max_angular_acc = 20;

  bool motor_control = true;
  Eigen::VectorXd m; // mass of the quadrotor
  double g = 9.81;
  double max_f = 1.4;        // thrust to weight ratio -- Khaled DONE
  double arm_length = 0.046; // m
  double t2t = 0.006;        // thrust-to-torque ratio
  double dt = .01;
  bool visualize = true;
  std::string shape = "sphere";
  Eigen::Vector2d distance_weights_payload_pose = Eigen::Vector2d(1, 0);
  Eigen::Vector2d distance_weights_payload_vel  = Eigen::Vector2d(.5, 0);
  Eigen::Vector2d distance_weights_quads_pose   = Eigen::Vector2d(1, 0.5);
  Eigen::Vector2d distance_weights_quads_vel    = Eigen::Vector2d(.5, .05);
  Eigen::VectorXd u_ub;
  Eigen::VectorXd u_lb;

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(filename, af) << std::endl;
    out << be << STR(num_robots, af) << std::endl;
    out << be << STR(model_path, af) << std::endl;

    out << be << STR(col_size_robot, af) << std::endl;
    out << be << STR(col_size_payload, af) << std::endl;

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
    out << be << STR_VV(m, af) << std::endl;

    out << be << STR_VV(distance_weights_payload_pose, af) << std::endl;
    out << be << STR_VV(distance_weights_payload_vel, af) << std::endl;
    out << be << STR_VV(distance_weights_quads_pose, af) << std::endl;
    out << be << STR_VV(distance_weights_quads_vel, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct Model_MujocoQuadsPayload : Model_robot {
  // state x: [qpos qvel]
  // xpos_{name}: [position quaternion]_{name of the body} \in R^{7x1}
  // xvel_{name}: [velocity ang velocity in body frame]_{name of the body} \in R^{6x1}
  // qpos: [xpos_{payload}, xpos_{quad1}, ..., xpos_{quadn}] \in R^{7*(n+1)}
  // qvel: [xvel_{payload}, xvel_{quad1}, ..., xvel_{quadn}] \in R^{6*(n+1)}


  // Regularization in the optimization problem.
  // you have to make this genereal
  Eigen::VectorXd state_weights;
  Eigen::VectorXd state_ref;

  std::vector<std::unique_ptr<fcl::CollisionObjectd>>
      collision_objects; // QUIM : TODO move this to the base class!

  virtual ~Model_MujocoQuadsPayload() = default;

  // Eigen::VectorXd ff; // TODO: remember to allocate memory in constructor!
  MujocoQuadsPayload_params params;

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    NOT_IMPLEMENTED;
  }

  void get_payload_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector3d> out) {
    out = x.head<3>();
  }

  void get_payload_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector3d> out) {
    out = x.segment(7*(params.num_robots+1), 3);
  }

  void get_payload_q(const Eigen::Ref<const Eigen::VectorXd> &x,
                     Eigen::Ref<Eigen::Vector4d> out) {
    out = x.segment(3, 4);
  }

  void get_payload_w(const Eigen::Ref<const Eigen::VectorXd> &x,
                     Eigen::Ref<Eigen::Vector3d> out) {
    out = x.segment(7*(params.num_robots+1)+3, 3);
  }

  virtual void get_robot_i_position(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    int i, Eigen::Ref<Eigen::Vector3d> out) {
    DYNO_CHECK_LEQ(i, params.num_robots - 1, "");

    out = x.segment(7 + 7*i,3);
  }
  virtual void get_robot_i_velocity(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    int i, Eigen::Ref<Eigen::Vector3d> out) {
    DYNO_CHECK_LEQ(i, params.num_robots - 1, "");

    out = x.segment(7*(params.num_robots+1) + 6*i, 3);
  }


  void get_robot_i_w(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                     Eigen::Ref<Eigen::Vector3d> out) {
    DYNO_CHECK_LEQ(i, params.num_robots - 1, "");
    out = x.segment(7*(params.num_robots+1) + 6*i + 3, 3);
  }


  virtual void
  get_robot_i_quat(const Eigen::Ref<const Eigen::VectorXd> &x, int i,
                          Eigen::Ref<Eigen::Vector4d> out) {
    DYNO_CHECK_LEQ(i, params.num_robots - 1, "");
    // NOT_IMPLEMENTED_TODO; // @KHALED
    out = x.segment(7 + 7*i + 3, 4);
  }

  double arm;
  double g = 9.81;

  double u_nominal;
  Eigen::Matrix4d B0;
  Eigen::Matrix4d B0inv;

  const bool adapt_vel = true;
  bool check_inner = true;
  mjModel* m;
  mjData* d;
  mjData* tmp;
  bool viewer_ready_ = false;
  mjvScene   scn_;
  mjrContext con_;
  mjvCamera  cam_;
  mjvOption  opt_;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  Model_MujocoQuadsPayload(const Model_MujocoQuadsPayload &) = default;

  Model_MujocoQuadsPayload(const char *file,
                        const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                        const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_MujocoQuadsPayload(MujocoQuadsPayload_params(file), p_lb, p_ub) {}

  Model_MujocoQuadsPayload(
      const MujocoQuadsPayload_params &params = MujocoQuadsPayload_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual std::map<std::string, std::vector<double>>
  get_info(const Eigen::Ref<const Eigen::VectorXd> &x) override;

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    for (int i = 0; i < params.num_robots + 1; ++i) {
      xout.segment(7 * i + 3 , 4).normalize();
    }
  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override;

  virtual void transform_primitive(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      // std::vector<Eigen::VectorXd> &xs_out,
      // std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) override {

    NOT_IMPLEMENTED
  }

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
