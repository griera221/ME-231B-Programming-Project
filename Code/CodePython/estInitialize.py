import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estInitialize():
    # Fill in whatever initialization you'd like here. This function generates
    # the internal state of the estimator at time 0. You may do whatever you
    # like here, but you must return something that is in the format as may be
    # used by your estRun() function as the first returned variable.
    #
    # The second returned variable must be a list of student names.
    # 
    # The third return variable must be a string with the estimator type

    #we make the internal state a list, with the first three elements the position
    # x, y; the angle theta; and our favorite color. 
    x = 0
    y = 0
    theta = 0
    color = 'green' 

    # Constants
    r = 0.425   # wheel radius
    B = 0.8     # Wheel base

    # Initial state
    x0 = np.array([0.0, 0.0, np.pi/4])

    # Initial covariance
    P0 = np.diag([1.0, 1.0, 0.1])   # Have some initial uncertainty

    # Process noise covariance  (TUNE)
    Q = np.diag([0.2, 0.2, 0.05])

    # Measurement noise covariance  (TUNE)
   # R = np.diag([2.1, 2.0])
    R = np.array([[1.09, 1.53], [1.53, 2.99]])  # Have some initial uncertainty
    # note that there is *absolutely no prescribed format* for this internal state.
    # You can put in it whatever you like. Probably, you'll want to keep the position
    # and angle, and probably you'll remove the color.
    # Store internal state as dictionary
    internalState = [
        float(x0[0]),  # x
        float(x0[1]),  # y
        float(x0[2]),  # theta
        P0,         # state covariance
        Q,         # process noise
        R,         # measurement noise
        r,         # wheel radius
        B,         # wheel base
    ]




    # replace these names with yours. Delete the second name if you are working alone.
    studentNames = ['George Riera']
    
    # replace this with the estimator type. Use one of the following options:
    #  'EKF' for Extended Kalman Filter
    #  'UKF' for Unscented Kalman Filter
    #  'PF' for Particle Filter
    #  'OTHER: XXX' if you're using something else, in which case please
    #                 replace "XXX" with a (very short) description
    estimatorType = 'EKF'  
    
    return internalState, studentNames, estimatorType

