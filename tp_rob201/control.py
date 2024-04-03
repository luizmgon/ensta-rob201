""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1

    speed = 0.0
    rotation_speed = 0.0

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    K_goal = 0.01
    dist_goal = np.sqrt((current_pose[0]-goal_pose[0])**2 + (current_pose[1]-goal_pose[1])**2)

    if(dist_goal != 0):

        k_grad = K_goal/dist_goal

        gradx = -k_grad*(current_pose[0]-goal_pose[0])
        grady = k_grad*(current_pose[1]-goal_pose[1])

        grad_theta = np.arctan(gradx/grady)

        print("Antes ", grad_theta, gradx, grady,current_pose)

        if(grad_theta > 0): 
            if(gradx < 0): grad_theta -= np.pi
        else:
            if(grady > 0): grad_theta += np.pi


        speed = np.sqrt(gradx**2 + grady**2)
        print((grad_theta - current_pose[2]), grad_theta, current_pose[2])

        command = {"forward": speed if speed < 1 else 1,
            "rotation": (grad_theta - current_pose[2]) / (5*np.pi)}

        

    return command
