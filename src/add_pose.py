
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)
    turn_1 = gtsam.Pose2(0.0, 0.0, math.radians(45))
    move_forward = gtsam.Pose2(2.0, 0.0, 0.0)
    turn_2 = gtsam.Pose2(0.0, 0.0, math.radians(45))
    
    odometry = turn_1.compose(move_forward).compose(turn_2)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    pose_3 = initial_estimate.atPose2(X(3))
    pose_4_guess = pose_3.compose(odometry)
    
    initial_estimate.insert(X(4), pose_4_guess)

    return graph, initial_estimate