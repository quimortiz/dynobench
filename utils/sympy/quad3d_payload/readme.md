# States X:
## Payload: 
* [position, velocity]
$$X_{load} = p_{lx}, p_{ly}, p_{lz}, v_{lx}, v_{ly}, v_{lz}$$
## Cable: 
* [unit directional vector, angular velocity]
$$X_{cable} = q_{cx}, q_{cy}, q_{cz}, w_{cx}, w_{cy}, w_{cz} $$
* Note that: $q_c$ is pointing from the quadrotor to payload
* $x_{quad} = x_l - lq_c$, where $l$ is the length of the cable. 
## quadrotor 
* [quaternion, angular veloctity]:
$$ X_{uav} = q_x, q_y, q_z, q_w, w_x, w_y, w_z$$
## Full state vector: 
* [load pos, cable vector, load vel, cable w, quad quaternion, quad w]
$$ X = [p_{l},\quad q_c,\quad v_{l},\quad w_c,\quad q,\quad w]$$
## f
$$v_l, \quad \dot{q}_c,\quad a_l, \quad \dot{w}_c, \quad \dot{q}, \dot{w} $$
* s.t
    $$\dot{q}_c  = w_c \times q_c $$
## Step 
$$X = X_0 + fdt$$