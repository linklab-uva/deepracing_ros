import numpy as np
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint, Bounds


class SplinerOptim:
    def __init__(self,  q_d : float, q_ds : float, q_ddelta : float, kappa_min : float):
        # self.d_initial = d_initial
        # self.s_ego = s_ego
        self.q_d = q_d
        self.q_ds = q_ds
        self.q_ddelta = q_ddelta
        self.kappa_min = kappa_min
    