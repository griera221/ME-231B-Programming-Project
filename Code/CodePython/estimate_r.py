import numpy as np

# Load Run 0 data (stationary bicycle)
data = np.genfromtxt('data/run_000.csv', delimiter=',')

# Extract x and y measurements from columns 3 and 4 (skip NaNs)
valid_meas = data[~np.isnan(data[:, 3]), 3:5]

# Compute the sample covariance of GPS noise
R_empirical = np.cov(valid_meas.T)

print("Estimated measurement noise covariance R:")
print(R_empirical)