

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {
struct Unicycle_sphere_params {
  Unicycle_sphere_params(const char *file) { read_from_yaml(file); }
  Unicycle_sphere_params() = default;

  double max_vel = .5;
  double min_vel = -.5;
  double max_angular_vel = .5;
  double min_angular_vel = -.5;
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
  Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5);
  std::string shape = "sphere";
  double radius = 0.4;
  double dt = .1;
  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);
  std::string filename = "";
  void inline write(std::ostream &out) const {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(min_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(min_angular_vel, af) << std::endl;
    out << be << STR(shape, af) << std::endl;
    out << be << STR(dt, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR(filename, af) << std::endl;
  }
};

struct Model_unicycle_sphere : Model_robot {

  virtual ~Model_unicycle_sphere() = default;

  Unicycle_sphere_params params;

  Model_unicycle_sphere(const char *file,
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_unicycle_sphere(Unicycle_sphere_params(file), p_lb, p_ub) {}

  virtual void write_params(std::ostream &out) override { params.write(out); }

  Model_unicycle_sphere(const Unicycle_sphere_params &params = Unicycle_sphere_params(),
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout(2) = wrap_angle(xin(2));
  }
  virtual int number_of_r_dofs();
  virtual int number_of_so2();
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect);

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
                      const Eigen::Ref<const Eigen::VectorXd> &y) override {

    return lower_bound_time(x, y);
  }

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    return 0;
  }
};
} // namespace dynobench
