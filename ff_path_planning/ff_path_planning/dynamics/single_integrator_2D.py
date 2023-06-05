import numpy as np

A, B = np.eye(2), np.eye(2)
XDIM, UDIM = A.shape[-1], B.shape[-1]


def f_fx_fu_fn(x, u, params=None):
    params = dict() if params is None else params
    Bp = B * params["dt"] if "dt" in params else B
    assert x.shape[-1] == XDIM and u.shape[-1] == UDIM
    xp = (A @ x[..., None])[..., 0] + (Bp @ u[..., None])[..., 0]
    fx = np.tile(A, x.shape[:-1] + (1, 1))
    fu = np.tile(Bp, x.shape[:-1] + (1, 1))
    return xp, fx, fu
