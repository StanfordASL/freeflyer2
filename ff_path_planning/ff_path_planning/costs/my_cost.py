from pmpc import Problem
import numpy as np

def cost(N: int, xdim: int, udim: int, params = None):
    # let's just use the default cost builder from pmpc
    p = Problem(N=N, xdim=xdim, udim=udim)
    if "Q" in params:
        p.Q = np.array(params["Q"])
    if "R" in params:
        p.R = np.array(params["R"])
    if "X_ref" in params:
        p.X_ref = np.array(params["X_ref"])
    if "U_ref" in params:
        p.U_ref = np.array(params["U_ref"])
    return dict(Q=p.Q, R=p.R, X_ref=p.X_ref, U_ref=p.U_ref)