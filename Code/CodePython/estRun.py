import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement):

    x = np.array(internalStateIn[0:3])  # x, y, theta
    P = internalStateIn[3]              # state covariance
    Q = internalStateIn[4]              # process noise
    R = internalStateIn[5]              # measurement noise
    r = internalStateIn[6]              # wheel radius
    B = internalStateIn[7]              # wheel base

    theta = x[2]            # current angle
    v = 5 * r * pedalSpeed  # linear velocity
    tan_gamma = np.tan(steeringAngle) # steering angle
    cos_theta = np.cos(theta) 
    sin_theta = np.sin(theta)

    # Prediction Matrix from bicycle model
    x_pred = np.array([
        x[0] + v * dt * cos_theta,
        x[1] + v * dt * sin_theta,
        x[2] + (v / B) * tan_gamma * dt
    ])

    # Linearize jacobian
    A = np.array([
        [1, 0, -v * dt * sin_theta],
        [0, 1, v * dt * cos_theta],
        [0, 0, 1]
    ])
    # Predict covariance
    P_pred = A @ P @ A.T + Q

    # Update Step
    if not (np.isnan(measurement[0]) or np.isnan(measurement[1])):
        meas = np.array(measurement)

        # Measurement Prediction
        h = np.array([
            x_pred[0] + 0.5 * B * np.cos(x_pred[2]),
            x_pred[1] + 0.5 * B * np.sin(x_pred[2])
        ])

        # Measurement Jacobian 
        H = np.array([
            [1, 0, -0.5 * B * np.sin(x_pred[2])],
            [0, 1, 0.5 * B * np.cos(x_pred[2])]
        ])
        
        y = meas - h   
        S = H@P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_update = x_pred + K @ y
        P_update = (np.eye(3) - K @ H) @ P_pred
    else:
        x_update = x_pred
        P_update = P_pred

        


    #### OUTPUTS ####
    internalStateOut = [
        float(x_update[0]),  # x
        float(x_update[1]),  # y
        float(x_update[2]),  # theta
        P_update,            # state covariance
        Q,         # process noise
        R,         # measurement noise
        r,         # wheel radius
        B,         # wheel base
    ]
    x = float(x_update[0])       # Current best estimate for x-position
    y = float(x_update[1])       # Current best estimate for y-position
    theta = float(x_update[2])   # Current best estimate for theta 

    # Angle normalization 
    theta = np.mod(theta + np.pi, 2 * np.pi) - np.pi
    # DO NOT MODIFY THE OUTPUT FORMAT:
    return x, y, theta, internalStateOut 


