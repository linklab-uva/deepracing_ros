from re import L
import numpy as np
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint, Bounds, OptimizeResult

class CurvatureConstraintWrapper:
    def __init__(self, max_kappas : np.ndarray, global_traj_kappas : np.ndarray, delta_s : np.ndarray):
        #The numerical differentiation pops off the last 2 points
        self.max_kappas = max_kappas[:-2]
        self.global_traj_kappas = global_traj_kappas[:-2]
        #delta_s already loses 1 point, only need to pop off 1 more.
        self.down_sampled_delta_s = delta_s[:-1]

    def fun(self, d : np.ndarray) -> np.ndarray:
        """
        Constraint function to ensure curvature does not exceed max_kappa. Copied from spliner github.
        """
                # Calculate curvature at each point using numerical differentiation
        # k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        # x' = self.down_sampled_delta_s, x'' = 0
        
        y_prime = np.diff(d)
        y_prime = np.where(y_prime == 0, 1e-6, y_prime) # Avoid division by zero
        y_prime_prime = np.diff(y_prime)
        y_prime = y_prime[:-1] # Make it the same length as y_prime_prime
        
        kappa = (self.down_sampled_delta_s * y_prime_prime) / ((self.down_sampled_delta_s ** 2) ** (3/2))
        # np.diff losses last two points so we delete them from self.global_traj_kappas
        total_kappa = self.global_traj_kappas - kappa
        violation = np.abs(total_kappa)
        return violation
    def as_scipy(self, jac="3-point", keep_feasible=False):
        return NonlinearConstraint(self.fun, np.zeros_like(self.max_kappas), self.max_kappas, jac=jac, keep_feasible=keep_feasible)
class CollisionAvoidanceConstraintWrapper:
    def __init__(self, opponent_d : np.ndarray, safety_buffers : np.ndarray):
        self.opponent_d = opponent_d
        self.safety_buffers = safety_buffers

    def fun(self, ego_d : np.ndarray) -> np.ndarray:
        """
        Constraint function to ensure collision avoidance
        """
        return np.abs(self.opponent_d - ego_d)
    def as_scipy(self, jac="3-point", keep_feasible=False):
        return NonlinearConstraint(self.fun, self.safety_buffers, np.inf*np.ones_like(self.safety_buffers), jac=jac, keep_feasible=keep_feasible)
class EndpointConstraintsWrapper:
    def __init__(self, d_current : float):
        self.d_current = d_current
    def as_scipy(self, npoints : int, keep_feasible=False):
        jac = np.zeros((3, npoints), dtype=float)
        jac[0,0] = 1.0
        jac[1,-2] = 1.0
        jac[2,-1] = 1.0
        eq : np.ndarray = np.zeros(3, dtype=float)
        eq[0] = self.d_current
        return LinearConstraint(jac, eq, eq, keep_feasible=keep_feasible)
        # return NonlinearConstraint(self.fun, self.safety_buffers, np.inf*np.ones_like(self.safety_buffers), jac=jac, keep_feasible=keep_feasible)
class SplinerOptim:
    def __init__(self,  q_d : float = 10.0, q_ds : float = 100.0, q_ddelta : float = 1000.0, kappa_max : float = 0.1):
        # self.d_initial = d_initial
        # self.s_ego = s_ego
        self.q_d = q_d
        self.q_ds = q_ds
        self.q_ddelta = q_ddelta
        self.kappa_max = kappa_max
    def objective(self, d : np.ndarray) -> float:
        return np.sum((d) ** 2) * self.q_d  + np.sum(np.diff(np.diff(d))**2) * self.q_ds + (np.diff(d)[0] ** 2) * self.q_ddelta


    def optimize_d(self, opponent_d : np.ndarray, ego_d_guess : np.ndarray, safety_buffers : np.ndarray,
                   lower_bound : np.ndarray, upper_bound : np.ndarray, 
                   global_traj_kappas : np.ndarray, delta_s : np.ndarray, 
                   options={'ftol': 1e-1, 'maxiter': 20, 'disp': False}) -> OptimizeResult:
        """
        Optimize the ego trajectory d using scipy's minimize function with constraints.
        """
        # Create the curvature constraint wrapper
        max_kappas = self.kappa_max*np.ones_like(opponent_d)
        curvature_constraint = CurvatureConstraintWrapper(max_kappas, global_traj_kappas, delta_s)
        collision_constraint = CollisionAvoidanceConstraintWrapper(opponent_d, safety_buffers)
        endpoint_constraints = EndpointConstraintsWrapper(float(ego_d_guess[0]))

        constraints = [curvature_constraint.as_scipy(), collision_constraint.as_scipy(), endpoint_constraints.as_scipy(len(ego_d_guess))]

        bounds = Bounds(lower_bound, upper_bound, keep_feasible=True)
        
        return minimize(self.objective, ego_d_guess, method='SLSQP', constraints=constraints, bounds=bounds, options=options)
