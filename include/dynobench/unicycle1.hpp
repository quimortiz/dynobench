#include <nlohmann/json.hpp>

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"
#include "dynobench/tojson.hpp"

// #include "eigen_conversions.hpp"

using json = nlohmann::json;

namespace dynobench {
//
// namespace ns {
// // a simple struct to model a person
// struct person {
//   std::string name;
//   std::string address;
//   int age;
// };
//
// namespace ns {
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(person, name, address, age)
// }
//
// } // namespace ns

struct Unicycle1_paramsJ {
  Unicycle1_paramsJ() = default;

  double max_vel = .5;
  double min_vel = -.5;
  double max_angular_vel = .5;
  double min_angular_vel = -.5;
  std::string shape = "box";
  double dt = .1;
  Eigen::VectorXd size;
  Eigen::VectorXd distance_weights;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Unicycle1_paramsJ, max_vel, min_vel,
                                 max_angular_vel, min_angular_vel, shape, dt,
                                 size, distance_weights);

  void read_from_json(const char *file) {
    std::ifstream f(file);
    json data = json::parse(f);
    *this = data.template get<Unicycle1_paramsJ>();
  }

  void read_from_yaml(const char *file) {
    std::cout << "loading file: " << file << std::endl;
    YAML::Node node = YAML::LoadFile(file);

    {
      Eigen::VectorXd X(2);
      X << 1, 2;
      json j;
      j = X;
      std::cout << j.dump() << std::endl;

      // Eigen::VectorXd Y(2);

      // from_json(j, Y);
      // std::cout << Y << std::endl;

      auto Y = j.template get<Eigen::VectorXd>();
      // j.from_json(
      std::cout << Y << std::endl;

      // Y << j;
      // std::cout << Y << std::endl;
    }

    YAML::Emitter emitter;
    emitter << node;
    std::cout << "Node :" << emitter.c_str() << std::endl;
    json j = tojson::detail::yaml2json(node);
    std::cout << j.dump() << std::endl;
    *this = j.template get<Unicycle1_paramsJ>();
  }

  void write_yaml(std::ostream &os) const {
    json j = *this;

    std::cout << j.dump() << std::endl;

    os << tojson::emitters::toyaml(j) << std::endl;
  }
};

struct Unicycle1_params {
  Unicycle1_params(const char *file) { read_from_yaml(file); }
  Unicycle1_params() = default;

  double max_vel = .5;
  double min_vel = -.5;
  double max_angular_vel = .5;
  double min_angular_vel = -.5;
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
  Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5);
  std::string shape = "box";
  double dt = .1;

  NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Unicycle1_params, max_vel, min_vel,
                                     max_angular_vel, min_angular_vel);
  //
  // void read_from_json(const char *file) {
  //
  //   std::ifstream f(file);
  //   json data = json::parse(f);
  //   this = data.template get<Unicycle1_params>();
  // }

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

// void file_to_unicycle1_params(const char *file) {
//   std::ifstream f(file);
//   json data = json::parse(f);
//   auto params = data.template get<Unicycle1_paramsJ>();
// }

struct Model_unicycle1 : Model_robot {

  virtual ~Model_unicycle1() = default;

  Unicycle1_params params;

  Model_unicycle1(const char *file,
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_unicycle1(Unicycle1_params(file), p_lb, p_ub) {}

  virtual void write_params(std::ostream &out) override { params.write(out); }

  Model_unicycle1(const Unicycle1_params &params = Unicycle1_params(),
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout(2) = wrap_angle(xin(2));
  }

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xinout) override {
    xinout(2) = wrap_angle(xinout(2));
  }

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
