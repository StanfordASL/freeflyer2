"""
Free Flyer dynamics
"""
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, patches


# VARIABLES
dim1 = 0.11461
dim2 = 0.0955

Fmax = 0.2

# Return x(t+1) given x(t), u(t)

class Dynamics:
    
    def __init__(self, x0, stateDim=6, inputDim=8):
        self.init = x0
        self._x = x0
        self.stateDimn = stateDim
        self.inputDimn = inputDim
        self.thruster_orien = np.array(([-1,0],[1,0],[0,-1],[0,1],[1,0],[-1,0],[0,1],[0,-1]))
        self.thruster_pos = np.array(([dim2, dim1],[-dim2, dim1],[-dim1, dim2],[-dim1, -dim2],
                                     [-dim2, -dim1],[dim2, -dim1],[dim1, -dim2],[dim1, dim2]))

        self._u = None

    def get_state(self):
        return self._x

    def derive(self, x, u, t):
        return np.zeros((self.stateDimn,1))

    def integrate(self, u, t, dt):
        self._x = self.get_state() + self.derive(self.get_state(), u, t)*dt
        return self._x


class ThrusterDyn(Dynamics):

        # Thrusters Configuration
        #      (2) e_y (1)        ___
        #     <--   ^   -->      /   \
        #    ^  |   |   |  ^     v M  )
        # (3)|--o-------o--|(8)    __/
        #       | free- |
        #       | flyer |   ---> e_x
        #       | robot |
        # (4)|--o-------o--|(7)
        #    v  |       |  v
        #     <--       -->
        #      (5)     (6)

    def __init__(self, x0 = np.zeros((6,1)), m = 16, Ixx = 0.18, r = 0.1):
        super().__init__(x0, stateDim=6, inputDim = 8)
        self._m = m
        self._Ixx = Ixx
        self._r = r

    # with the position, return the velocity
    def derive(self, X, U, t):
        """
        Returns the derivative of the state vector
        Args:
            X => state variable, 6x1 numpy array at time t
            U => input array: 8x1 numpy array, 8 thrusters
            t => time
        Returns:
            xDot: 6x1 x 1 derivative of the state vector
        """
        #unpack the state vector
        x, y, x_dot, y_dot = X[0,0], X[1,0], X[3, 0], X[4, 0] #velocities
        theta, theta_dot = X[2, 0], X[5, 0]             #orientations
        res = self.resultant_force_and_moment(U)
        F_x, F_y = res[0], res[1]  # Force x and y

        xdot_world, ydot_world = self.get_rotmatrix_body_to_world(theta) @ [x_dot, y_dot]

        F = self.get_rotmatrix_body_to_world(theta) @ [F_x, F_y]
        M = res[2]    # Moment about z
        # missing coriolis term? Maybe co
        x_ddot, y_ddot = self.get_rotmatrix_body_to_world(theta) @ [F_x/self._m, F_y/self._m]
        theta_ddot = M/self._Ixx
        deriv = np.array([[x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]]).T

        # Noise
        # for i in range(len(deriv)):
        #     deriv[i] = random.gauss(1, 0.05) * deriv[i]
        return deriv

    def thrusters(self, index):
        # Thrusters Configuration
        #      (2) e_y (1)        ___
        #     <--   ^   -->      /   \
        #    ^  |   |   |  ^     v M  )
        # (3)|--o-------o--|(8)    __/
        #       | free- |
        #       | flyer |   ---> e_x
        #       | robot |
        # (4)|--o-------o--|(7)
        #    v  |       |  v
        #     <--       -->
        #      (5)     (6)
        """
        Returns the resultant force from the thruster specified
        Input:
            index: number thruster we want to get
        Returns:
            resultant (3x1) Force + moment ABOUT ROBOT AXIS from thruster
        """
        assert index <= 8 and index >= 1, "input must be 8x1 mapping of thrusters"
        F = self.thruster_orien[index-1]
        tPos = self.thruster_pos[index-1]

        deg = np.arctan2(tPos[1], tPos[0])
        
        Moment = np.cross(tPos, F*Fmax)
        # Force = tPos[1]/(tPos[0]**2+tPos[1]**2)*np.array([-tPos[0], -tPos[1]]) * Fmax
        Force = F*Fmax # Not too sure about this one
        return np.append(Force, Moment).reshape(-1,1)
        
    def resultant_force_and_moment(self, input):
        """
        Returns the resultant total force and moment that we would predict
        Args:
            thruster command, (8x1), each corresponding to a thruster index
        Returns:
            Force and moment of the FF in its own frame. 
        """
        force_x, force_y, moment = 0 , 0 , 0
        sq_input = input.squeeze()
        assert len(input) == 8, "check size of input, must have 8 binary values"
        for idx in range(len(sq_input)):
            if sq_input[idx] == 1:
                tPos = self.thruster_pos[idx-1]
                thruster_dir = np.rad2deg(np.arctan2(tPos[1], tPos[0]))

                force_x += Fmax * np.cos(thruster_dir)
                force_y += Fmax *np.sin(thruster_dir)
                moment += tPos[0]*Fmax*np.sin(thruster_dir)-tPos[1]*Fmax*np.cos(thruster_dir)
        # force_x = sq_input[1]+sq_input[4]-(sq_input[0] + sq_input[5])
        # force_y = sq_input[3]+sq_input[6] - (sq_input[2]+sq_input[7])
        # print()
        return force_x, force_y, moment
        # for i in range(len(squeezed_input)):
        #     # i gives the index in which I want to actuate -1. 
        #     if squeezed_input[i] == 1:
        #         # print("activated thruster", i+1)
        #         force_x += self.thrusters(i+1)
        # return force_x
                    
    def get_rotmatrix_body_to_world(self, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        return R