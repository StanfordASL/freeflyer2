# MIT License
#
# Copyright (c) 2023 Stanford Autonomous Systems Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
