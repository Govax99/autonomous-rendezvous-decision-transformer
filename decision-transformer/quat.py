import numpy as np

def quat2rotm(q):
    qA = np.array([ [0, -q[2], q[1]],
                    [q[2], 0, -q[0]],
                    [-q[1], q[0], 0]])
    qv = q[0:3][np.newaxis]
    R = qv.T @ qv + q[3]**2*np.eye(3) + 2*q[3]*qA + qA @ qA
    return R

def quat_kin_matrix(w):
    A = np.array([[0, w[2], -w[1], w[0]],
                  [-w[2], 0, w[0], w[1]],
                  [w[1], -w[0], 0, w[2]],
                  [-w[0], -w[1], -w[2], 0]])
    return A

def rotate(v, q):
    qv = q[0:3]
    t = 2*np.cross(qv, v)
    vp = v + q[3]*t + np.cross(qv, t)
    return vp