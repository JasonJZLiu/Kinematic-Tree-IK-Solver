
from ik_solver import IKSolver
import numpy as np


if __name__ == "__main__":
    AllegroIKSolver = IKSolver(urdf_file = "allegro_assets/allegro_hand_description_right.urdf", 
                               model_name = "allegro_hand_right", 
                               root_link_name = "palm_link",
                               visualize = True,
                               position_tolerance = 0.001)

    AllegroIKSolver.end_effector_frames = ['link_3_tip', 'link_7_tip', 'link_11_tip', 'link_15_tip']
    desired_EE_poses = [np.array([-0.17066993, -0.32635043,  0.77864744, 0, 0, 0]),
                    np.array([-0.18389982, -0.28697679,  0.76127432 , 0, 0, 0]),
                    np.array([-0.19232582, -0.24936115,  0.73991827, 0, 0, 0]),
                    np.array([-0.16467229, -0.24722305,  0.77046255, 0, 0, 0])]
    desired_root_pose = np.array([-0.25287761, -0.25028728,  0.80106794, 0, 0, 0])
    q_initial_guess = [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]

    q_result = AllegroIKSolver.get_ik_solution(desired_EE_poses, desired_root_pose, q_initial_guess, verbose = True)

    AllegroIKSolver.update_meshcat()

    # input("Wait")
    # desired_EE_poses = [np.array([-0.17066993, -0.32635043,  0.77864744, 0, 0, 0]),
    #                 np.array([-0.18389982, -0.28697679,  0.76127432 , 0, 0, 0]),
    #                 np.array([-0.19232582, -0.24936115,  0.73991827, 0, 0, 0]),
    #                 np.array([-0.16467229, -0.24722305,  0.77046255, 0, 0, 0])]
    # desired_root_pose = np.array([-0.25287761, -0.25028728,  0.80106794, 0, 0, 0])
    # q_initial_guess = [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    # q_result = AllegroIKSolver.get_ik_solution(desired_EE_poses, desired_root_pose, q_initial_guess, verbose = True)
    # AllegroIKSolver.update_meshcat()

    while True:
      pass

#EOF