import numpy as np

A, B = np.array([[1.0]]), np.array([[1.0]])
XDIM, UDIM = A.shape[-1], B.shape[-1]


def f_fx_fu_fn(x, u, p=None):
    Bp = B * p if p is not None else B
    assert x.shape[-1] == 1 and u.shape[-1] == 1
    xp = (A @ x[..., None])[..., 0] + (Bp @ u[..., None])[..., 0]
    fx = np.tile(A, x.shape[:-1] + (1, 1))
    fu = np.tile(Bp, x.shape[:-1] + (1, 1))
    return xp, fx, fu