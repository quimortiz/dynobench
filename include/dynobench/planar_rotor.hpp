
#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Quad2d_params {

  Quad2d_params(const char *file) { read_from_yaml(file); }

  Quad2d_params() = default;

  double max_f = 1.3;
  double dt = .01;

  double max_vel = 4;
  double max_angular_vel = 8;
  double max_acc = 25;
  double max_angular_acc = 25;

  // Big drone
  // double m = 2.5;
  // double I = 1.2;
  // double l = .5;

  // Crazy fly - style
  double m = 0.034;
  double I = 1e-4;
  double l = 0.1;

  bool drag_against_vel = false;
  double k_drag_linear = .0001;
  double k_drag_angular = .0001;
  std::string shape = "box";

  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, .5, .2, .2);

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void inline write(std::ostream &out) const {

    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(max_f, af) << std::endl;
    out << be << STR(dt, af) << std::endl;

    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_acc, af) << std::endl;

    // Big drone
    // double m = 2.5;
    // double I = 1.2;
    // double l = .5;

    // Crazy fly - style
    out << be << STR(m, af) << std::endl;
    out << be << STR(I, af) << std::endl;
    out << be << STR(l, af) << std::endl;

    out << be << STR(drag_against_vel, af) << std::endl;
    out << be << STR(k_drag_linear, af) << std::endl;
    out << be << STR(k_drag_angular, af) << std::endl;
    out << be << STR(shape, af) << std::endl;

    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR(filename, af) << std::endl;
  }
};

struct Model_quad2d : Model_robot {

  virtual ~Model_quad2d() = default;
  Quad2d_params params;

  const double g = 9.81;
  double u_nominal;

  Model_quad2d(const char *file,
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_quad2d(Quad2d_params(file), p_lb, p_ub) {}

  Model_quad2d(const Quad2d_params &params = Quad2d_params(),
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout(2) = wrap_angle(xin(2));
  }

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x(3) = 0;
    x(4) = 0;
    x(5) = 0;
  }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override {
    DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
    Eigen::VectorXd out(nx);
    out.setZero();
    out.head(2) = x.head(2);
    return out;
  }

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> v,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void offset(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> p) override {

    CHECK((static_cast<size_t>(p.size()) == 4 ||
           static_cast<size_t>(p.size()) == 2),
          AT);

    if (p.size() == 4) {
      p.head(2) = xin.head(2);       // x,y
      p.tail(2) = xin.segment(3, 2); // vx,vy
    } else {
      NOT_IMPLEMENTED;
    }
  }

  virtual void canonical_state(const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout.head(2).setZero();
    xout.segment(3, 2).setZero();
  }

  virtual size_t get_offset_dim() override { return 4; }

  virtual void transform_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                               const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    CHECK((p.size() == 2 || p.size() == 4), AT);
    if (p.size() == 2) {
      Model_robot::transform_state(p, xin, xout);
    } else if (p.size() == 4) {
      xout.head<2>() += p.head<2>();
      xout.segment<2>(3) += p.tail<2>();
    }
  }

  virtual void transform_primitive(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in,
      std::vector<Eigen::VectorXd> &xs_out,
      std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) override {

    CHECK((p.size() == 2 || 4), AT);

    if (p.size() == 2) {
      Model_robot::transform_primitive(p, xs_in, us_in, xs_out, us_out,
                                       is_valid_fun, num_valid_states);
    } else {
      Model_robot::transform_primitive2(p, xs_in, us_in, xs_out, us_out,
                                        is_valid_fun, num_valid_states);
    }
  }
};
} // namespace dynobench
