#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {
    struct Differential_drive_params {
        Differential_drive_params(const char *file) { read_from_yaml(file); }
        Differential_drive_params() = default;

        double max_vel = .5;
        double min_vel = -.5;
        double max_angular_vel = .5;
        double min_angular_vel = -.5;
        Eigen::VectorXd size;
        Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5);
        double radius = 0.1;
        std::string shape = "sphere";
        double dt = .1;
        double r = 0.4;      //wheel radius
        double L = 1.0;      // wheel distance

        NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Differential_drive_params, max_vel, min_vel,
                                     max_angular_vel, min_angular_vel);


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
            out << be << STR_VV(distance_weights, af) << std::endl;
            out << be << STR(radius, af) << std::endl;
            out << be << STR(r, af) << std::endl;
            out << be << STR(L, af) << std::endl;
            out << be << STR(filename, af) << std::endl;
        }
    };

    struct Model_differential_drive : Model_robot {
        virtual ~Model_differential_drive() = default;

        Differential_drive_params params;

        Model_differential_drive(const char *file,
                        const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                        const Eigen::VectorXd &p_ub = Eigen::VectorXd())
            : Model_differential_drive(Differential_drive_params(file), p_lb, p_ub) {}

        virtual void write_params(std::ostream &out) override { params.write(out); }

        Model_differential_drive(const Differential_drive_params &params = Differential_drive_params(),
                        const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                        const Eigen::VectorXd &p_ub = Eigen::VectorXd());

        virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

        virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
            xout(2) = wrap_angle(xout(2));
        }
        virtual int number_of_r_dofs() override;
        virtual int number_of_so2() override;
        virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override;
        virtual int number_of_robot() override { return 1; }

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
}

