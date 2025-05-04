import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement):
    # In this function you implement your estimator. The function arguments
    # are:
    #  time: current time in [s] 
    #  dt: current time step [s]
    #  internalStateIn: the estimator internal state, definition up to you. 
    #  steeringAngle: the steering angle of the bike, gamma, [rad] 
    #  pedalSpeed: the rotational speed of the pedal, omega, [rad/s] 
    #  measurement: the position measurement valid at the current time step
    #
    # Note: the measurement is a 2D vector, of x-y position measurement.
    #  The measurement sensor may fail to return data, in which case the
    #  measurement is given as NaN (not a number).
    #
    # The function has four outputs:
    #  x: your current best estimate for the bicycle's x-position
    #  y: your current best estimate for the bicycle's y-position
    #  theta: your current best estimate for the bicycle's rotation theta
    #  internalState: the estimator's internal state, in a format that can be understood by the next call to this function

    # Example code only, you'll want to heavily modify this.
    # this internal state needs to correspond to your init function:
    x = np.array(internalStateIn[0:3])
    P = internalStateIn[3]
    Q = internalStateIn[4]
    R = internalStateIn[5]
    r = internalStateIn[6]
    B = internalStateIn[7]

    theta = x[2]
    v = 5 * r * pedalSpeed  # linear velocity
    tan_gamma = np.tan(steeringAngle)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # Prediction
    x_pred = np.array([
        x[0] + v * dt * cos_theta,
        x[1] + v * dt * sin_theta,
        x[2] + (v / B) * tan_gamma * dt
    ])

    # Linearized jacobian
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

        # Jacobian 
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
    # Update the internal state (will be passed as an argument to the function
    # at next run), must obviously be compatible with the format of
    # internalStateIn:
    internalStateOut = [
        float(x_update[0]),  # x
        float(x_update[1]),  # y
        float(x_update[2]),  # theta
        P_update,         # state covariance
        Q,         # process noise
        R,         # measurement noise
        r,         # wheel radius
        B,         # wheel base
    ]
    x = float(x_update[0])
    y = float(x_update[1])
    theta = float(x_update[2]) 
    # DO NOT MODIFY THE OUTPUT FORMAT:
    return x, y, theta, internalStateOut 


