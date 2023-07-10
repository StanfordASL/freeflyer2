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

from pmpc import Problem
import numpy as np


def cost(N: int, xdim: int, udim: int, params=None):
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
