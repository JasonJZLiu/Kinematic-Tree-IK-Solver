"""
@author     Jingzhou Liu
@email      jingzhou.liu@mail.utoronto.ca
@author     Ritvik Singh
@email      ritviks.singh@mail.utoronto.ca

@brief      Provides forward kinematics and inverse kinematics (using SLSQP) functionalities 
            given the urdf of a robot.
"""

import pinocchio
import numpy as np
from scipy.optimize import minimize, Bounds
from typing import List, Optional, Tuple, Union


IK_SOLVER_DEFAULT_CONFIG = {"ftol": 1e-9, 
                            "disp": False, 
                            'eps': 1e-8,
                            'maxiter': 10000,
                            }


class KinematicsSolver:
    def __init__(self, urdf_path: str, ee_link_names: dict, config: Optional[dict] = None):
        """
        Initializes the robot model on which control's to be performed.
        Arguments:
            urdf_path: Path to the file containing URDF of the robot.
            ee_link_names: A dictionary containing the link names to solve kinematics to and 
                           whether to contrain orientation. Ex. ee_link_names = {"link7": True}
        """
        # configuration
        self.config = IK_SOLVER_DEFAULT_CONFIG
        if config is not None:
            self.config.update(config)
        # model: contains constants such as lengths, masses, names etc
        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        # data: contains working memory used by algorithm
        self.data = self.model.createData()
        # create frame handles from the defined model
        self.ee_link_ids = [self.model.getFrameId(link_name) for link_name in ee_link_names.keys()]
        # convenience mapping between ee ids to orientation constraint
        self.orientation_flag_dict = dict()
        for index, ee_link_id in enumerate(self.ee_link_ids):
            self.orientation_flag_dict[ee_link_id] = list(ee_link_names.values())[index]
        # joint limits 
        self.joint_limits = [(self.model.lowerPositionLimit[i], self.model.upperPositionLimit[i]) for i in range (self.model.nq)]
        # converting to SciPy Bounds object
        self.bounds = Bounds(self.model.lowerPositionLimit, self.model.upperPositionLimit)
        # desired end effector poses
        self.x_ee_des = list()
	
    def __str__(self) -> str:
        """
        Display the placement of each joint of the kinematic tree.
        """
        msg = "Kinematics Solver: \n"
        msg += f" Dim. of Generalized Coordinates: {self.model.nq}\n"
        msg += f" Dim. of Generalized Velocities: {self.model.nv}\n"
        msg += f" IK solver parameters:\n"
        # display solver parameters
        for key in self.config.keys():
            msg += ''.join(f"   {key}: {self.config[key]}")
            msg += '\n'
        return msg

    """
    Internals
    """
    def _loss_function(self, q: np.ndarray) -> float:
        """
        Computes the loss function for the optimizer
        Arguments:
            q: The current joint configuration of the robot
        Returns:
            A single value denoting the error between the current task-space pose
            and the desired one
        """
        # update frame information
        pinocchio.framesForwardKinematics(self.model, self.data, np.asarray(q))
        loss = 0
        for index, frame_id in enumerate(self.ee_link_ids):
            # error with orientation constraints
            if self.orientation_flag_dict[frame_id]:
                dMf = self.x_ee_des[index].actInv(self.data.oMf[frame_id])
                err = pinocchio.log(dMf).vector
            # error without orientation constraints
            else:
                err = self.data.oMf[frame_id].translation - np.asarray(self.x_ee_des[index])
            loss += np.linalg.norm(err)**2
        return loss

    """
    Operations
    """
    def compute_forward_kinematics(self, q: Union[np.ndarray, List[float]]) -> List[np.ndarray]:
        """
        Compute end effector positions for the given joint configuration.
        Arguments:
            q: Flat list of angular joint positions.
        Returns:
            List of end-effector positions. Each position is given as an np.array in the format
            of [x, y, z, q_x, q_y, q_z, q_w]
        """
	# update frame information
        pinocchio.framesForwardKinematics(self.model, self.data, np.asarray(q))
        pinocchio.updateFramePlacements(self.model, self.data)
        return [pinocchio.SE3ToXYZQUAT(self.data.oMf[link_id]) for link_id in self.ee_link_ids]
        #return [list(pinocchio.SE3ToXYZQUAT(self.data.oMf[link_id])) for link_id in self.ee_link_ids]

    
    def solve_inverse_kinematics(self, x_ee_des: List[List[float]], 
           q_init: Optional[np.ndarray] = None, loss_tol: float = 1e-3, 
           max_iter: int = 1) -> Tuple[np.ndarray, bool]:
        """
        Computes the required joint angles to achieve the desired task-space pose
        Arguments:
            x_ee_des: Desired end-effector position (x, y, z) if orientation does not need to be constrained.
                      Desired end-effector pose (x, y, z, q_x, q_y, q_z, q_w) if orientation is constrained.
            q_init: Initial guess for the solver. If None, random configuration is picked.
    	    loss_tol: The maximum acceptable loss for the solver.
	        max_iter: The maximum number of times the solver should try to solve in order to get below loss_tol.
        Returns:
            Tuple containing: 
	    	A numpy array representing the joint configurations that achieve the desired task-space pose.
		    A boolean representing whether the solution is within a specified tolerance.
        """
        iterations = 0
        
        # copy desired poses into proper format
        self.x_ee_des = []
        for index, ee_frame_id in enumerate(self.ee_link_ids):
            # position + orientation constraint
            if self.orientation_flag_dict[ee_frame_id]:
                self.x_ee_des.append(pinocchio.XYZQUATToSE3(x_ee_des[index]))
            # position constraint
            else:
                self.x_ee_des.append(x_ee_des[index])

        # compute initial loss 
        if q_init is not None:
            # calculate loss for user given configuration
        	loss = self._loss_function(q_init)
        else:
            loss = float("inf")

        best_config = None
        best_loss = loss
        optim_success = False
        while iterations < max_iter and loss > loss_tol:
            # setting up initial guess 
            if iterations > 0 or q_init is None:
                # sample from within constraints 
                q_init = 1 * np.random.uniform(low=np.array(self.model.lowerPositionLimit), 
                                               high=np.array(self.model.upperPositionLimit))

            # start optimization
            optim_res = minimize(self._loss_function, q_init, method="SLSQP",
                                tol=1e-6,
                                options=self.config,
                                bounds=self.bounds)
            # get the loss of the current solution
            loss = optim_res.fun
            # save relevant values if this is better than all prior solutions
            if loss < best_loss:
                best_config = optim_res.x
                best_loss = loss
                optim_success = optim_res.success
            print("Iteration: {}    Loss: {}    q_init: {}".format(iterations, loss, q_init))
            iterations += 1
           
           
        # returns joint position solution and whether the solution is below a specified tolerance
        return best_config, (best_loss < loss_tol) == optim_success == True, best_loss
