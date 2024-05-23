import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import interp1d
from Class.icp import icp_matching

def dynamic_downsample(scan, target_points=100):
    # Adjust the downsampling factor based on the number of points
    if len(scan) > target_points:
        factor = len(scan) // target_points
        return scan[::factor]
    return scan

def resample_scan(scan, target_points=100):
    # Resample the scan to a fixed number of points
    if len(scan) == target_points:
        return scan

    indices = np.linspace(0, len(scan) - 1, target_points).astype(int)
    resampled_scan = scan[indices]
    return resampled_scan

# def icp(A, B, max_iterations=50, tolerance=1e-6):
#     nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(B)
#     prev_error = float('inf')

#     for i in range(max_iterations):
#         distances, indices = nbrs.kneighbors(A)
#         T = B[indices[:, 0]]

#         H = np.dot(T.T, A)
#         U, S, Vt = np.linalg.svd(H)
#         R = np.dot(Vt.T, U.T)
#         t = np.mean(T - np.dot(A, R.T), axis=0)

#         A = np.dot(A, R.T) + t

#         mean_error = np.mean(distances)
#         if abs(prev_error - mean_error) < tolerance:
#             break
#         prev_error = mean_error

#     return R, t

def odometry_lidar_only(scan_prev, scan_curr, target_points=100): 
    scan_prev = dynamic_downsample(scan_prev, target_points)
    scan_curr = dynamic_downsample(scan_curr, target_points)
    
    scan_prev = resample_scan(scan_prev, target_points)
    scan_curr = resample_scan(scan_curr, target_points)
    
    scan_prev = np.array(scan_prev).T
    scan_curr = np.array(scan_curr).T
    R, t = icp_matching(scan_prev, scan_curr)
    
    theta = np.arctan2(R[1, 0], R[0, 0])
    x, y = t

    return x, y, theta