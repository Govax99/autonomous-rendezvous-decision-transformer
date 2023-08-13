import numpy as np
import quat

def dynamics(t, state, parameters, actions):
    J_C = parameters.J_C
    J_T = parameters.J_T
    m_C = parameters.m_C
    OM = parameters.OM
    OM_IL_L = np.array([0, 0, OM])

    p_LC_L = state[0:3]
    v_LC_L = state[3:6]
    q_LC = state[6:10]/np.linalg.norm(state[6:10])
    w_IC_C = state[10:13]
    q_LT = state[13:17]/np.linalg.norm(state[13:17])
    w_IT_T = state[17:20]

    f = actions[0:3]
    tau = actions[3:6]

    R_LC = quat.quat2rotm(q_LC)
    w_LC_L = R_LC @ w_IC_C - OM_IL_L
    A_C = quat.quat_kin_matrix(w_LC_L)

    R_LT = quat.quat2rotm(q_LT)
    w_LT_L = R_LT @ w_IT_T - OM_IL_L
    A_T = quat.quat_kin_matrix(w_LT_L)

    dx = np.concatenate((
        v_LC_L,
        1/m_C*(2*OM*v_LC_L[1] + 3*OM**2*p_LC_L[0] + f[0])[np.newaxis],
        1/m_C*(-2*OM*v_LC_L[0] + f[1])[np.newaxis],
        1/m_C*(-OM**2*p_LC_L[2] + f[2])[np.newaxis],
        1/2*A_C @ q_LC,
        np.linalg.solve(J_C,(-np.cross(w_IC_C, (J_C @ w_IC_C)) + tau)),
        1/2*A_T @ q_LT,
        np.linalg.solve(J_T,(-np.cross(w_IT_T, (J_T @ w_IT_T))))
    ))
    return dx