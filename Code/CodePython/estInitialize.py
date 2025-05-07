import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estInitialize():

    # Constants
    r = 0.425   # wheel radius
    B = 0.8     # Wheel base

    # Initial state (cyclist startes near the origin and is initially heading North-East)
    x0 = np.array([0.0, 0.0, np.pi/2])

    # Initial covariance
    P0 = np.diag([1.0, 1.0, 0.1])   # Have some initial uncertainty

    # Process noise covariance  (Values were tuned by hand)
    Q = np.diag([0.2, 0.1, 0.05])

    # Measurement noise covariance  (Values Were tuned by hand)
    R = np.array([[1.00, 1.5], [1.5, 3.0]])  

    # Internal State
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

