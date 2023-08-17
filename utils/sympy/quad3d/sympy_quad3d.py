import sympy as sp
from helper import *
from sympy import latex
###############################################################################################################
def computeWritestepDiff(f, transition_func, state, action, params):
        m, m_inv, J, J_inv, dt = params
        pos = state[0:3]
        q   = state[3:7]
        vel = state[7:10]
        w   = state[10:13]
        
        # Construct the final Fx and Fu matrices
        Fx = transition_func.jacobian(state)
        Fu = transition_func.jacobian(action)
        stepd_name = """void Model_quad3d::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                Eigen::Ref<Eigen::MatrixXd> Fu,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u,
                                double dt) {
        """
        stepd_vars = """        calcDiffV(__Jv_x, __Jv_u, x, u);
                                Eigen::Vector4d eta = B0 * u;
                                auto const &J_v = params.J_v;
                                Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
                                Eigen::Vector3d vel = x.segment(7, 3).head<3>();
                                Eigen::Vector3d w = x.segment(10, 3).head<3>();

        """
        calcFx = """ 
        """
        for i in range(Fx.rows):
                for j in range(Fx.cols):
                        if Fx[i,j] != 0:
                                calcFx += "       Fx({},{}) = {};\n".format(i,j, sp.ccode(Fx[i,j]))

        calcFu = """ 
        """
        for i in range(Fu.rows):
                for j in range(Fu.cols):
                        if Fu[i,j] != 0:
                                calcFu += "       Fu({},{}) = {};\n".format(i,j, sp.ccode(Fu[i,j]))

        calcFu += """} """
        
        stepDiff = stepd_name + stepd_vars + calcFx + calcFu
        return stepDiff, Fx, Fu

def computeJacobians(f, state, action, params):
        m, m_inv, J, J_inv, dt = params

        # Calculate Jacobians: Jx, Ju:
        Jx = f.jacobian(state)
        Ju = f.jacobian(action)
        calcDiffV = """void Model_quad3d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) { """

        calcDiffV_vars = """    Eigen::Vector4d eta = B0 * u;
                                auto const &J_v = params.J_v;
                                Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
                                Eigen::Vector3d vel = x.segment(7, 3).head<3>();
                                Eigen::Vector3d w = x.segment(10, 3).head<3>();
                                """

        calcJx = """
                
        """
        for i in range(Jx.rows):
                for j in range(Jx.cols):
                        if Jx[i,j] != 0:
                                calcJx += "    Jv_x({},{}) = {};\n".format(i,j, sp.ccode(Jx[i,j]))

        calcJu = """ 
        """
        for i in range(Ju.rows):
                for j in range(Ju.cols):
                        if Ju[i,j] != 0:
                                calcJu += "    Jv_u({},{}) = {};\n".format(i,j, sp.ccode(Ju[i,j]))

        calcJu += """} """
        calcDiffV = calcDiffV + calcDiffV_vars + calcJx + calcJu

        return calcDiffV, Jx, Ju

def computeandWriteStepFunc(f, state, action, params):
        # Step Function: 
        m, m_inv, J, J_inv, dt = params
        pos = state[0:3]
        q   = state[3:7]
        vel = state[7:10]
        w   = state[10:13]

        stepSym = sp.zeros(13,1)

        pos_next =  sp.Matrix(pos) + dt *sp.Matrix(f[0:3])
        q_next   = integrate(q,w,dt) 
        vel_next = sp.Matrix(vel) + dt * sp.Matrix(f[7:10])
        w_next   = sp.Matrix(w) + dt * sp.Matrix(f[10:13]) 

        stepSym[0] = pos_next[0]
        stepSym[1] = pos_next[1]
        stepSym[2] = pos_next[2]
        stepSym[3] = q_next[0]
        stepSym[4] = q_next[1]
        stepSym[5] = q_next[2]
        stepSym[6] = q_next[3]
        stepSym[7] = vel_next[0]
        stepSym[8] = vel_next[1]
        stepSym[9] = vel_next[2]
        stepSym[10] = w_next[0]
        stepSym[11] = w_next[1]
        stepSym[12] = w_next[2]

        step_name = """void Model_quad3d::step(Eigen::Ref<Eigen::VectorXd> xnext,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {
        """

        step_vars = """         calcV(ff, x, u);
                                Eigen::Vector4d eta = B0 * u;
                                auto const &J_v = params.J_v;
                                Eigen::Ref<const Eigen::Vector3d> pos = x.head(3).head<3>();
                                Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
                                CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
                                Eigen::Ref<const Eigen::Vector3d> vel = x.segment(7, 3).head<3>();
                                Eigen::Ref<const Eigen::Vector3d> w = x.segment(10, 3).head<3>();
                                Eigen::Matrix4d Jqnorm;
                                Eigen::Ref<Eigen::Vector3d> pos_next = xnext.head(3);
                                Eigen::Ref<Eigen::Vector4d> q_next = xnext.segment(3, 4);
                                Eigen::Ref<Eigen::Vector3d> vel_next = xnext.segment(7, 3);
                                Eigen::Ref<Eigen::Vector3d> w_next = xnext.segment(10, 3);

        """

        step_comp = """         Eigen::Vector3d posN({}, {}, {});
                                pos_next = posN;
                                
                                Eigen::Vector4d qn({}, {}, {}, {});
                                q_next = qn.normalized();
                                CHECK_LEQ(std::abs((q_next.norm() - 1.0)), 1e-6, AT);
                                Eigen::Vector3d velN({}, {}, {});
                                vel_next = velN;

                                Eigen::Vector3d wN({}, {}, {});
                                w_next = wN;
                                }}
         """.format(sp.ccode(stepSym[0]), sp.ccode(stepSym[1]), sp.ccode(stepSym[2]), # pos
                sp.ccode(stepSym[3]), sp.ccode(stepSym[4]), sp.ccode(stepSym[5]), sp.ccode(stepSym[6]), # q 
                sp.ccode(stepSym[7]), sp.ccode(stepSym[8]), sp.ccode(stepSym[9]), # vel 
                sp.ccode(stepSym[10]), sp.ccode(stepSym[11]), sp.ccode(stepSym[12])) # w

        step = step_name + step_vars + step_comp
        
        return step, stepSym

def computeandWritef(state, action, params):
        m, m_inv, J, J_inv, dt = params
        pos = state[0:3]
        q   = state[3:7]
        vel = state[7:10]
        w   = state[10:13]
        
        q_dot = quat_diff(q,w)

        # acceleration: 
        a = sp.Matrix([0,0,-9.81]) + m_inv*sp.Matrix(quat_qvrot(q,[0,0,action[0]]))

        # compute omega dot
        J_omega =  sp.Matrix(J)*sp.Matrix(w)
        wdot = J_inv * (sp.Matrix(vcross(J_omega, w)) + sp.Matrix(action[1:4]))

        f = sp.Matrix(
                [
                vel[0],
                vel[1],
                vel[2],
                q_dot[0],
                q_dot[1],
                q_dot[2],
                q_dot[3],
                a[0],
                a[1],
                a[2],
                wdot[0],
                wdot[1],
                wdot[2]
                ]
        ) 
        # Function name: calcV
        calcV_name = """void Model_quad3d::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {"""
        
        calcV_vars = """        Eigen::Vector4d eta = B0 * u;
                                Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
                                Eigen::Vector3d vel = x.segment(7, 3).head<3>();
                                Eigen::Vector3d w = x.segment(10, 3).head<3>();
                                auto const &J_v = params.J_v;"""

        calcV_linAcc = """      Eigen::Vector3d a({}, {}, {}); 
        """.format(sp.ccode(f[7]), sp.ccode(f[8]), sp.ccode(f[9]))

        calcV_assignment_ff = """        ff.head<3>() = vel;
                                         Eigen::Vector4d quat_dot({}, {}, {}, {});
                                         ff.segment<4>(3) = quat_dot.normalized();
                                         ff.segment<3>(7) = a;
                                         Eigen::Vector3d wdot({}, {} ,{});
                                         ff.segment<3>(10) = wdot;
                                 }}
        """.format(sp.ccode(f[3]), sp.ccode(f[4]), sp.ccode(f[5]), sp.ccode(f[6]), sp.ccode(f[10]), sp.ccode(f[11]), sp.ccode(f[12]))

        calcV = calcV_name + calcV_vars + calcV_linAcc + calcV_assignment_ff

        return f, calcV

def createSyms():
        # Define symbolic variables
        x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = sp.symbols('x(0), x(1), x(2), q(0), q(1), q(2), q(3), vel(0), vel(1), vel(2), w(0), w(1), w(2)')
        m = sp.symbols('m')
        m_inv = sp.symbols('m_inv')
        dt = sp.symbols('dt')

        Ixx, Iyy, Izz = sp.symbols('J_v(0) J_v(1) J_v(2)')
        state = [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
        eta0 = sp.symbols('eta(0)')
        eta1 = sp.symbols('eta(1)')
        eta2 = sp.symbols('eta(2)')
        eta3 = sp.symbols('eta(3)')
        action = sp.Matrix([eta0, eta1, eta2, eta3])

        J = sp.Matrix([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])
        J_inv = J**(-1)
        omega = sp.Matrix([wx, wy, wz])
        J_omega = sp.Matrix([J*omega])
        params = [m, m_inv, J, J_inv ,dt]
        return state, action, params

def writeinFile(calcV, step, calcDiffV, stepDiff):
        with open("quad3d.cpp", "w") as f:
                f.write(calcV)
                f.write(step)
                f.write(calcDiffV)
                f.write(stepDiff)

def main():

        state, action, params = createSyms()
        f, calcV = computeandWritef(*createSyms())
        step, stepSym = computeandWriteStepFunc(f, state, action, params)
        calcDiffV, Jx, Ju = computeJacobians(f,state,action,params)
        stepDiff, Fx, Fu = computeWritestepDiff(f,stepSym,state,action,params)

        writeinFile(calcV, step, calcDiffV, stepDiff)




if __name__ == "__main__":
        main()
