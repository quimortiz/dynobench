

import sympy as sp
from multiprocessing import Pool
import os

import sympy as sp
from sympy import Matrix
from sympy.utilities.iterables import numbered_symbols
from collections import OrderedDict
import argparse
from datetime import datetime

import numpy as np 
import argparse
from itertools import chain


from multiprocessing import Pool
import os
import time
import sys
from pathlib import Path
from datetime import datetime

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from sympy.codegen.rewriting import create_expand_pow_optimization


expand_opt = create_expand_pow_optimization(2)

from helper import *


def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return sp.Matrix([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]])


def flatten(w_tilde):
    w1 = w_tilde[2,1]
    w2 = w_tilde[0,2]
    w3 = w_tilde[1,0]
    return sp.Matrix([w1,w2,w3])


def computef(*data):
    print("compute f(x,u)...")
    state, action, params = data
    if params[1] == "point":
        num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
        start_idx = 6
    elif params[1] == "rigid":
        num_uavs, payloadType, mi, Ji, mp, Jp, attPi, li, motor_params, dt, B = params
        start_idx = 13
    
    f = sp.zeros(start_idx + 6*num_uavs + 7*num_uavs,1)
    vp = state[3:6]
    # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_1, wc_dot_1, ..., qc_dot_{n-1}, wc_dot_{n-1}]
    qwcdot = []
    ap_ = sp.zeros(3,1)
    Mq = (mp)*sp.eye(3)

    cableSt = state[start_idx : start_idx + num_uavs]
    uavStates = state[start_idx + num_uavs:  start_idx + 6 + 7 + num_uavs]
    # acceleration of the payload
    for c_idx, cable in enumerate(cableSt):
        qc = sp.Matrix(qvnormalize(cable[0:3]))
        wc = cable[3:6]
        uavState = uavStates[c_idx]
        q = qnormalize(sp.Matrix(uavState[0:4])) 
        eta = B[c_idx]*sp.Matrix(action[c_idx])
        fu = sp.Matrix([0,0,eta[0]])
        u_i = sp.Matrix(quat_qvrot(q,fu)) 
        ap_ += (qc*qc.T*u_i - (mi[c_idx]*li[c_idx]*vdot(wc,wc))*qc) 
        Mq += qc*mi[c_idx]*qc.T
    print("computing inverse of Mq...")
    # ap = Mq.cholesky_solve(ap_) - sp.Matrix([0, 0, 9.81])
    print("finished inverse computation...")

    # Backward substitution: Solve L.T * ap = y
    ap = Mq.LUsolve(ap_) - sp.Matrix([0, 0, 9.81])

    #Final Equation for the payload acceleration
    # qwcdot vector computation:        
    # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_1, wc_dot_1, ..., qc_dot_{n-1}, wc_dot_{n-1}]
    for c_idx, cable in enumerate(cableSt):
        qc = sp.Matrix(qvnormalize(cable[0:3]))
        wc = sp.Matrix(cable[3:6])
        uavState = uavStates[c_idx]
        q = qnormalize(sp.Matrix(uavState[0:4])) 
        eta = B[c_idx]*sp.Matrix(action[c_idx])
        fu = sp.Matrix([0,0,eta[0]])
        u_i = sp.Matrix(quat_qvrot(q,fu))
        apgrav =  ap + sp.Matrix([0,0,9.81]) 
        wcdot = 1/li[c_idx] * sp.Matrix(vcross(qc, apgrav)) - (1/(mi[c_idx]*li[c_idx])) * sp.Matrix(vcross(qc,u_i)) 
        qcdot = sp.Matrix(vcross(wc, qc))
        qwcdot.append(qcdot)
        qwcdot.append(wcdot)

    # uav states: [quat_0, w_0, quat_1, w_1, ..., quat_{n-1}, quat_{n-1}]
    uavSt_dot = []
    for u_idx, uavState in enumerate(uavStates):
        q = qnormalize(sp.Matrix(uavState[0:4]))
        w = sp.Matrix(uavState[4::])
        uavSt_dot.extend(quat_diff(q,w).tolist())
        J_uav = sp.diag(Ji[u_idx][0], Ji[u_idx][1], Ji[u_idx][2])
        J_uav_inv = J_uav**(-1)
        J_omega = J_uav * sp.Matrix(w)
        eta = B[u_idx]*sp.Matrix(action[u_idx])
        tau = sp.Matrix(eta[1:4])
        wdot =  J_uav_inv * (sp.Matrix(vcross(J_omega, w)) + tau)
        uavSt_dot.extend(wdot.tolist())
    
    
    if payloadType == "point":
        payload_f = sp.Matrix(
            [
                [vp[0]], [vp[1]], [vp[2]], # payload velocity
                [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            ]
            )

        f[0:start_idx,:] = payload_f
        f[start_idx:start_idx+6*num_uavs,:] = qwcdot
        f[start_idx+6*num_uavs: start_idx+7*num_uavs+6*num_uavs,:] = uavSt_dot
    
    
    if payloadType == "rigid":
        ## NOT IMPLEMENTED ###
       
        qp = sp.Matrix(qnormalize(sp.Matrix(state[3:7])))
        wp = sp.Matrix(state[10:13])
                
        qpd = quat_diff(qp, wp)

        ap_grav = sp.MatrixSymbol('ap_grav', 3, 1) 
        wpdot = sp.MatrixSymbol('wpdot', 3, 1)
        wpdot = sp.Matrix(sp.MatrixSymbol('wpdot', 3, 1))
        ap_grav = sp.Matrix(sp.MatrixSymbol('ap_grav', 3, 1)) 
        # Equation 5 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7843619
        ap_ = sp.zeros(3,1)
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = cable[3:6]
            uavState = uavStates[c_idx]
            q = qnormalize(sp.Matrix(uavState[0:4])) 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu)) 
            attPi[c_idx] = sp.Matrix(attPi[c_idx])
            # print(mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot)))
            # exit()
            ap_ += (u_i - mi[c_idx]*li[c_idx]*vdot(wc,wc)*qc - mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(wp) * skew(wp) * attPi[c_idx]))  +  mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot)) )
        
        eq1 = sp.Eq(Mq**(-1)*ap_, ap_grav)

        # Equation 6 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7843619
        wpdot_ = sp.zeros(3,1)
        
        term2 = sp.zeros(3,1)
        
        term3_otherside = sp.zeros(3,1)
        
        Jp_tilde_2 = sp.zeros(3,1)
        
        Jp_diag = sp.diag(Jp[0], Jp[1], Jp[2])
        
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = cable[3:6]
            uavState = uavStates[c_idx]
            q = qnormalize(sp.Matrix(uavState[0:4])) 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu)) 
            attPi[c_idx] = sp.Matrix(attPi[c_idx])

            # compute Inertia matrix
            
            Jp_tilde_2 += mi[c_idx] * skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp), qc*qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot )) ) )
            

            term2 += mi[c_idx] * skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp), qc * qc.T * ap_grav)) 

            term3_otherside += skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp) , (u_i - mi[c_idx] * li[c_idx] * vdot(wc,wc) * qc  - mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(wp) * skew(wp) * attPi[c_idx]))  ) ) ) 

        # print(skew(wp) * Jp_diag * wp)
        # exit()
        term1 = Jp_diag * wpdot - Jp_tilde_2
        
        eq2 = sp.Eq(term1 + term2 + skew(wp) * Jp_diag * wp, term3_otherside)

        solution = sp.solve((eq1, eq2), (ap_grav, wpdot))


        print(solution)
        exit()



        payload_f = sp.Matrix(
            [[vp[0]], [vp[1]], [vp[2]], # payload velocity
            [qpd[0]], [qpd[1]], [qpd[2]], [qpd[3]], # quaternion diff
            [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            [wpdot[0]], [wpdot[1]], [wpdot[2]], # payload angular acceleration
            ]
        )
        f[0:start_idx,:] = payload_f
    cse_replacements_f, f_simplified = sp.cse(f, order='canonical')
    return f, f_simplified, cse_replacements_f


# Utility Functions
def deduplicate_replacements(*replacements_lists):
    """
    Combine multiple lists of CSE replacements and remove duplicates while respecting dependencies.
    """
    combined_replacements = OrderedDict()
    for replacements in replacements_lists:
        for lhs, rhs in replacements:
            if lhs not in combined_replacements:
                combined_replacements[lhs] = rhs
    return list(combined_replacements.items())



def substitute_expression(args):
    """
    Substitutes replacements into a symbolic expression.

    Args:
        args: A tuple (expr, replacement_map).

    Returns:
        Substituted symbolic expression.
    """
    expr, replacement_map = args
    return expr.xreplace(replacement_map)


def batch_substitute_parallel(expr_list, replacement_map, num_processes=4):
    """
    Perform batch substitutions in parallel using multiprocessing.

    Args:
        expr_list: List of symbolic expressions.
        replacement_map: Dictionary of replacements to apply.
        num_processes: Number of parallel processes.

    Returns:
        List of substituted expressions.
    """
    with Pool(processes=num_processes) as pool:
        substituted_exprs = pool.map(substitute_expression, [(expr, replacement_map) for expr in expr_list])
    return substituted_exprs


def generate_code_with_cse(expr_list, cse_replacements, output_list_name=None, prefix="", num_processes=40):
    print(type(expr_list))
    # Flatten matrix expressions
    if isinstance(expr_list, sp.Matrix):
        expr_list = expr_list.tolist()
        expr_list = [expr for sublist in expr_list for expr in sublist]  # Flatten
    else:
        expr_list = list(expr_list)
    print(type(expr_list))

    # Prepare raw variables to skip unresolved vars
    raw_vars = {sym for sym, _ in cse_replacements}

    # Step 1: Prepare replacements with prefixed variable names
    replacement_map = {}
    prefixed_code_lines = []  # Collect replacement definitions
    final_code_lines = []  # Collect final output expressions

    # Generate prefixed replacement variables
    print("Generating replacements...")
    for lhs_sym, rhs_expr in cse_replacements:
        # if lhs_sym not in raw_vars:  # Exclude raw variables
        # Generate a prefixed name for the replacement
        # lhs_name = f"{prefix}_{sp.ccode(lhs_sym)}"
        lhs_name = f"{sp.ccode(lhs_sym)}"
        replacement_map[lhs_sym] = sp.Symbol(lhs_name)  # Map original to prefixed
        
        # Add the replacement to the prefixed_code_lines
        rhs_expr_resolved = rhs_expr.xreplace(replacement_map)
        prefixed_code_lines.append(f"    double {lhs_name} = {sp.ccode(rhs_expr_resolved)};")

    # # Step 2: Substitute replacements into the final expressions in parallel
    # print("Substituting final expressions in parallel...")
    # updated_expr_list = batch_substitute_parallel(expr_list, replacement_map, num_processes=num_processes)

    # Step 3: Generate final output expressions
    if output_list_name:
        for i, expr in enumerate(expr_list[0]):
            if expr != 0:  # Skip zero-valued expressions for efficiency
                final_code_lines.append(f"    {output_list_name}[{i}] = {sp.ccode(expr)};")
    else:
        for expr in updated_expr_list:
            final_code_lines.append(f"    {sp.ccode(expr)};")

    # # Check for unresolved variables
    # unresolved_vars = set.union(*(expr.free_symbols for expr in updated_expr_list)) - set(replacement_map.values())
    # if unresolved_vars:
    #     print(f"Warning: Unresolved variables in final expressions: {unresolved_vars}")

    # Combine all lines into the final generated code
    return '\n'.join(prefixed_code_lines + final_code_lines)
    # return '\n'.join(final_code_lines)


# Top-level function for computing the Jacobian of a block
def compute_jacobian_block(args):
    """
    Compute the Jacobian of a single block with respect to given variables.
    Logs the process ID to verify parallel execution.

    Parameters:
    - args: Tuple containing (f_block, vars).

    Returns:
    - Jacobian of the block as a sympy Matrix.
    """
    f_block, vars = args
    # print(f"[Process {os.getpid()}] Computing Jacobian for block...")
    start_time = time.time()
    result = f_block.jacobian(vars)
    # print(f"[Process {os.getpid()}] Finished in {time.time() - start_time:.2f}s.")
    return result


def parallel_jacobian_blocks(f_blocks, vars):
    """
    Compute Jacobian blocks in parallel.

    Parameters:
    - f_blocks: List of blocks of functions (submatrices of f).
    - vars: Variables to differentiate with respect to.

    Returns:
    - List of Jacobian blocks.
    """
    # Prepare arguments for parallel processing
    args = [(f_block, vars) for f_block in f_blocks]

    print(f"[Main Process] Starting parallel computation with {len(f_blocks)} blocks...")
    start_time = time.time()

    with Pool() as pool:
        jacobian_blocks = pool.map(compute_jacobian_block, args)

    print(f"[Main Process] Parallel computation finished in {time.time() - start_time:.2f}s.")
    return jacobian_blocks


def computeJ_parallel(f, *data):
    """
    Compute Jx and Ju using parallelized block-wise Jacobian computation.

    Parameters:
    - f: Symbolic function f (dynamics).
    - data: Tuple of state, action, and params.

    Returns:
    - Jx_simplified: Parallelized block-wise Jacobian w.r.t. state.
    - Ju_simplified: Parallelized block-wise Jacobian w.r.t. action.
    - unified_replacements: Unified CSE replacements.
    """
    state, action, params = data
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)
    f_matrix = sp.Matrix(f)
    # # Step 1: Divide `f_matrix` into smaller blocks
    # # block_size = int(f_matrix.rows/5)   # Adjust based on problem size
    # block_size = max(1, int(f_matrix.rows / 10))
    # block_size = 32
    # f_blocks_state = [f_matrix[i : i + block_size, :] for i in range(0, f_matrix.rows, block_size)]
    # # exit()
    # # Step 2: Compute Jacobian blocks for state and action in parallel
    # print("Parallel computation of Jx blocks...")
    # Jx_blocks = parallel_jacobian_blocks(f_blocks_state, state)

    # # block_size = int(len(action)/10) +1  # Adjust based on problem size
    # block_size = max(1, int(f_matrix.rows / 10))  # Adjust based on problem size
    # block_size = 32
    # f_blocks_action = [f_matrix[i : i + block_size, :] for i in range(0, f_matrix.rows, block_size)]
    
    # print("Parallel computation of Ju blocks...")
    # Ju_blocks = parallel_jacobian_blocks(f_blocks_action, action)

    # Step 3: Combine Jacobian blocks into full matrices
    # Jx = sp.BlockMatrix(Jx_blocks).as_explicit()
    # Ju = sp.BlockMatrix(Ju_blocks).as_explicit()
    Jx = f.jacobian(state)
    Ju = f.jacobian(action)
    # Step 4: Apply CSE to Jx and Ju independently for optimization
    cse_replacements_Jx, Jx_simplified = sp.cse(Jx, order="canonical", symbols=numbered_symbols("Jx_tmp"))
    cse_replacements_Ju, Ju_simplified = sp.cse(Ju, order="canonical", symbols=numbered_symbols("Ju_tmp"))

    # Step 5: Combine all CSE replacements
    # unified_replacements = deduplicate_replacements(cse_replacements_Jx, cse_replacements_Ju)

    return Jx_simplified, Ju_simplified, cse_replacements_Jx, cse_replacements_Ju


def computeF_parallel(step, *data):
    """
    Compute Fx and Fu using parallelized block-wise Jacobian computation.
    Jacobians are computed directly from the original step function.

    Parameters:
    - step: Symbolic step function (discrete dynamics).
    - data: Tuple of state, action, and params.

    Returns:
    - Fx_simplified: Parallelized block-wise Jacobian of step w.r.t. state.
    - Fu_simplified: Parallelized block-wise Jacobian of step w.r.t. action.
    - unified_replacements: Unified CSE replacements.
    """
    state, action, params = data
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)
    step_matrix = sp.Matrix(step)  # Original step function

    # Step 1: Divide `step_matrix` into smaller blocks
    # block_size = max(1, int(step_matrix.rows / 10))  # Adjust based on problem size
    # block_size = 32  # Adjust based on problem size
    # step_blocks_state = [step_matrix[i : i + block_size, :] for i in range(0, step_matrix.rows, block_size)]

    # # Step 2: Compute Jacobian blocks for state and action in parallel
    # print("Parallel computation of Fx blocks...")
    # Fx_blocks = parallel_jacobian_blocks(step_blocks_state, state)

    # print("Parallel computation of Fu blocks...")
    # # block_size = int(len(action)/10) +1  # Adjust based on problem size
    # block_size = 32  # Adjust based on problem size
    # step_blocks_action = [step_matrix[i : i + block_size, :] for i in range(0, step_matrix.rows, block_size)]
    # Fu_blocks = parallel_jacobian_blocks(step_blocks_action, action)

    # # Step 3: Combine Jacobian blocks into full matrices
    # Fx = sp.BlockMatrix(Fx_blocks).as_explicit()
    # Fu = sp.BlockMatrix(Fu_blocks).as_explicit()

    Fx = step.jacobian(state)
    Fu = step.jacobian(action)


    # Step 4: Apply CSE to Fx and Fu independently for optimization
    cse_replacements_Fx, Fx_simplified = sp.cse(Fx, order="canonical", symbols=numbered_symbols("Fx_tmp"))
    cse_replacements_Fu, Fu_simplified = sp.cse(Fu, order="canonical", symbols=numbered_symbols("Fu_tmp"))

    # Step 5: Deduplicate replacements
    # unified_replacements = deduplicate_replacements(cse_replacements_Fx, cse_replacements_Fu)

    return Fx_simplified, Fu_simplified, cse_replacements_Fx, cse_replacements_Fu


def computeStep(f, *data):
    """
    Compute the step function with normalization, without simplifying it.
    The original step function is returned for further Jacobian computations.

    Parameters:
    - f: Symbolic function f (dynamics).
    - data: Tuple of state, action, and params.

    Returns:
    - stepFunc: The original step function (without simplification).
    - cse_replacements_step: Empty list (no replacements done here).
    """
    state, action, params = data
    num_uavs, payloadType, *_ = params
    state = flatten_symbolic_structure(state)
    stepFunc = Matrix(state) + f * params[-2]  # `dt`

    # Normalize relevant parts (specific to UAV dynamics)
    for i in range(0, 3 * num_uavs, 3):
        qc_norm = qvnormalize(Matrix(stepFunc[6 + 2 * i: 6 + 2 * i + 3]))
        stepFunc[6 + 2 * i: 6 + 2 * i + 3, :] = qc_norm
    for i in range(0, 7 * num_uavs, 7):
        qint_norm = qnormalize(Matrix(stepFunc[6 + 6 * num_uavs + i: 6 + 6 * num_uavs + i + 4]))
        stepFunc[6 + 6 * num_uavs + i: 6 + 6 * num_uavs + i + 4, :] = qint_norm

    # Step 1: Apply CSE to the step function matrix
    cse_replacements_step, simplified_step = sp.cse(stepFunc, order='canonical')

    return stepFunc, simplified_step, cse_replacements_step

def writeSptoC(f, Jx, Ju, Fx, Fu, step, replacements, data):
    """
    Write generated C++ code with unified CSE replacements.

    Parameters:
    - f: Dynamics function.
    - Jx, Ju, Fx, Fu: Jacobian matrices.
    - step: Discrete step function.
    - replacements: Dictionary of replacements for each function.
    - data: Additional symbolic data.
    """
    state, action, params = data
    num_uavs, payloadType, *_ = params
    now = datetime.now()
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    tp = "p" if payloadType == "point" else "b"
    id = f"n{num_uavs}_{tp}"

    # File paths
    base_path = "../../../src/"
    file_out_hpp = f"{base_path}quadrotor_payload_dynamics_autogen_{id}.hpp"
    file_out_cpp = f"{base_path}quadrotor_payload_dynamics_autogen_{id}.cpp"

    # Function headers
    params_str = "double mp, double arm_length, double t2t, const double *m, const double *J_vx, const double *J_vy, const double *J_vz, const double *l"
    headers = {
        "f": f"void calcV_{id}(double* ff, {params_str}, const double *x, const double *u)",
        "step": f"void calcStep_{id}(double* xnext, {params_str}, const double *x, const double *u, double dt)",
        # "J": f"void calcJ_{id}(double* Jx, double* Ju, {params_str}, const double *x, const double *u)",
        # "F": f"void calcF_{id}(double* Fx, double* Fu, {params_str}, const double *x, const double *u, double dt)",
    }

    # Generate code
    print("Generating f code...")
    f_code = generate_code_with_cse(f, replacements['f'], "ff", prefix="f")
    print("Generating step code...")
    step_code = generate_code_with_cse(step, replacements['step'], "xnext", prefix="step")
    # print("Generating Jx code...")
    # Jx_code = generate_code_with_cse(Jx, replacements['Jx'], "Jx", prefix="Jx")
    # print("Generating Ju code...")
    # Ju_code = generate_code_with_cse(Ju, replacements['Ju'], "Ju", prefix="Ju")
    # print("Generating Fx code...")
    # Fx_code = generate_code_with_cse(Fx, replacements['Fx'], "Fx", prefix="Fx")
    # print("Generating Fu code...")
    # Fu_code = generate_code_with_cse(Fu, replacements['Fu'], "Fu", prefix="Fu")

    # Write C++ files
    with open(file_out_hpp, "w") as hpp:
        hpp.write("#pragma once\n\n")
        hpp.write(f"// Generated on {date_time}\n\nnamespace dynobench {{\n")
        for header in headers.values():
            hpp.write(header + ";\n")
        hpp.write("} // namespace dynobench\n")

    with open(file_out_cpp, "w") as cpp:
        cpp.write(f'#include "quadrotor_payload_dynamics_autogen_{id}.hpp"\n#include <cmath>\n')
        cpp.write(f"// Generated on {date_time}\n\nnamespace dynobench {{\n")
        cpp.write(headers["f"] + " {\n" + f_code + "\n}\n")
        cpp.write(headers["step"] + " {\n" + step_code + "\n}\n")
        # cpp.write(headers["J"] + " {\n" + Jx_code + "\n" + Ju_code + "\n}\n")
        # cpp.write(headers["F"] + " {\n" + Fx_code + "\n" + Fu_code + "\n}\n")
        cpp.write("} // namespace dynobench\n")



def createSyms(num_uavs=1, payloadType='point', writeC=False):
    # uavs parameter symbols
    mi = [sp.symbols("m[{}]".format(i)) for i in range(num_uavs)] # mass of each uav
    Jv = [list(sp.symbols("J_vx[{}] J_vy[{}] J_vz[{}]".format(i,i,i))) for i in range(num_uavs)]
    Ji = [list(sp.Matrix([Jv[i][0], Jv[i][1], Jv[i][2]])) for i in range(num_uavs)]

    t2t, arm_length = sp.symbols('t2t, arm_length')
    arm = 0.707106781 * arm_length
    motor_params = [t2t, arm]
    # paylaod parameter symbols
    mp = sp.symbols('mp')
    if payloadType == "point":
        Ixx, Iyy, Izz = sp.symbols('np.nan, np.nan, np.nan')    
    elif payloadType == "rigid": 
        # payload inertia matrix
        Ixx, Iyy, Izz = sp.symbols('Jp[0] Jp[1] Jp[2]')
    else: 
        print('Wrong payload type! Choose either point or rigid')

    Jp = sp.Matrix([Ixx, Iyy, Izz])
    # cables parameter symbols
    li = [sp.symbols("l[{}]".format(i)) for i in range(num_uavs+1)] # lengths of cables

    # time step:
    dt = sp.symbols('dt')

    # States: 
    # paylaod states: position, quaternion, velocity, angular velocity dim: 13 (point mass 6)
    if writeC: 
        mi = [sp.symbols("m[{}]".format(i)) for i in range(num_uavs)] # mass of each uav
        Jv = [list(sp.symbols("J_vx[{}] J_vy[{}] J_vz[{}]".format(i,i,i))) for i in range(num_uavs)]
        Ji = [list(sp.Matrix([Jv[i][0], Jv[i][1], Jv[i][2]])) for i in range(num_uavs)]
        st_idx = 6
        if payloadType == "point":
            st_idx = 6
            x, y, z, vx, vy, vz = sp.symbols('x[0] x[1] x[2] x[3] x[4] x[5]')
        elif payloadType == "rigid":
            st_idx = 13
            x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('x[0] x[1] x[2]  x[3] x[4] x[5] x[6]\
                                                                        x[7] x[8] x[9] x[10] x[11] x[12]')
            attP = [list(sp.symbols("attPx[{}] attPy[{}] attPz[{}]".format(i,i,i))) for i in range(num_uavs)]
            attPi = [list(sp.Matrix([attP[i][0], attP[i][1], attP[i][2]])) for i in range(num_uavs)]

        cableSt = [list(sp.symbols('x[{}] x[{}] x[{}] x[{}] x[{}] x[{}]'.format(i,i+1, i+2, i+3, i+4, i+5))) for i in range(st_idx,st_idx+6*num_uavs, 6)]
        uavSt   = [list(sp.symbols('x[{}] x[{}] x[{}] x[{}] x[{}] x[{}] x[{}]'.format(i,i+1, i+2, i+3, i+4, i+5, i+6))) for i in range(st_idx+6*num_uavs, st_idx+6*num_uavs+7*num_uavs, 7)]
        action  = [list(sp.symbols('u[{}] u[{}] u[{}] u[{}]'.format(i,i+1,i+2,i+3))) for i in range(0,4*num_uavs,4)]

    else: #WRITE SYMBOLS FOR PYTHON
        x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('pos[0] pos[1] pos[2]  qp[0] qp[1] qp[2] qw[3]\
                                                                            vel[0] vel[1] vel[2] wpx  wpy wpz')
        # cable states: cable unit directional vector, angular velocity, dim: 6n
        cableSt = [list(sp.symbols('qc[{}][0] qc[{}][1] qc[{}][2] wc[{}][0] wc[{}][1] wc[{}][2]'.format(i,i,i,i,i,i))) for i in range(num_uavs)]
        
        # uav rotational states: quaternions, angular velocities, dim: 7n
        uavSt = [list(sp.symbols('q[{}][0] q[{}][1] q[{}][2] q[{}][3] w[{}][0] w[{}][1] w[{}][2]'.format(i,i,i,i,i,i,i))) for i in range(num_uavs)]
        # action
        action = [list(sp.symbols('u[{}][0] u[{}][1] u[{}][2] u[{}][3]'.format(i,i,i,i))) for i in range(num_uavs)]
    
    if payloadType == "point":
        state = [x, y, z, vx, vy, vz, *cableSt, *uavSt]
        params = [num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt]
    elif payloadType == "rigid":
        state = [x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz, *cableSt, *uavSt]
        params = [num_uavs, payloadType, mi, Ji, mp, Jp, attPi, li, motor_params, dt]
    
    else: 
        print('Wrong payload type! Choose either point or rigid')
        exit()

    B = []
    B0 = sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    for i in range(num_uavs):
        u_nominal = mi[i]*9.81/4
        B.append(u_nominal*B0)
    params = [*params, B]

    return state, action, params

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--writeC", action="store_true", help="Generate C code")
    parser.add_argument("--num_uavs", type=int, default=2, help="Number of UAVs")
    parser.add_argument("--ptype", default="point", help="Payload type: point or rigid")
    args = parser.parse_args()

    state, action, params = createSyms(args.num_uavs, args.ptype, args.writeC)
    data = (state, action, params)

    # Compute dynamics
    f, simplified_f, cse_replacements_f = computef(*data)

    # Step 2: Compute Jacobians Jx and Ju in parallel
    print("Computing Jacobians (Jx, Ju) in parallel...")
    # Jx, Ju, cse_replacements_Jx, cse_replacements_Ju = computeJ_parallel(f, *data)

    # # Step 3: Compute step function (discrete dynamics)
    print("Computing step function...")
    step, simplified_step, cse_replacements_step = computeStep(f, *data)

    # # Step 4: Compute Jacobians Fx and Fu for the step function in parallel
    print("Computing step Jacobians (Fx, Fu) in parallel...")
    # Fx, Fu, cse_replacements_Fx, cse_replacements_Fu = computeF_parallel(step, *data)

    cse_replacements_f, f_simplified = sp.cse(f, order='canonical')
    # Unified replacements
    # unified_replacements = deduplicate_replacements(cse_replacements_J, cse_replacements_step, cse_replacements_F)
    replacements = {
        'f': cse_replacements_f,
        'step': cse_replacements_step,
        # 'Jx': cse_replacements_Jx,  # Jx and Ju replacements
        # 'Ju': cse_replacements_Ju,
        # 'Fx': cse_replacements_Fx,  # Fx and Fu replacements
        # 'Fu': cse_replacements_Fu,
    }

    # Write C++ code
    if args.writeC:
        print("writing in c file...")
        writeSptoC(simplified_f, 0, 0, 0, 0, simplified_step, replacements, data)
        # writeSptoC(simplified_f, Jx, Ju, Fx, Fu, simplified_step, replacements, data)
    else:
        writePython(step, args.num_uavs, args.ptype)


if __name__ == "__main__":
    main()
