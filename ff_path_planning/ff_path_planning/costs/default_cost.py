from pmpc import Problem

def cost(N: int, xdim: int, udim: int, params = None):
    # let's just use the default cost builder from pmpc
    p = Problem(N=N, xdim=xdim, udim=udim)
    return dict(Q=p.Q, R=p.R, X_ref=p.X_ref, U_ref=p.U_ref)