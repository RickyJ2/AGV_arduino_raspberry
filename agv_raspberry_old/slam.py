from time import sleep
from adafruit_rplidar import RPLidar, RPLidarException
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
# from icp2 import icp
from sklearn.neighbors import NearestNeighbors

def initialize_occupancy_grid(grid_size, cell_size):
    grid = np.zeros((grid_size, grid_size), dtype=np.float32)
    return grid

lidar = RPLidar(None, 'COM5', timeout=3)
runLidarThread = False
pose = np.array([0, 0, 0])
grid_size = 100
cell_size = 0.1
occupancy_grid = initialize_occupancy_grid(grid_size, cell_size)

def preprocess_scan(scan_data):
    angles = np.array([point[1] for point in scan_data])
    distances = np.array([point[2] for point in scan_data])
    # Filter out invalid measurements (distance = 0)
    valid_indices = distances > 0
    angles = angles[valid_indices]
    distances = distances[valid_indices]
    # Convert polar coordinates to Cartesian coordinates
    x = distances * np.cos(np.radians(angles))
    y = distances * np.sin(np.radians(angles))

    return np.vstack((x, y)).T

def normalize_points(points, target_size=180):
    if len(points) > target_size:
        indices = np.linspace(0, len(points) - 1, target_size).astype(int)
        normalized_points = points[indices]
    else:
        indices = np.round(np.linspace(0, len(points) - 1, target_size)).astype(int)
        normalized_points = points[indices]
    return normalized_points

def downsample(cartesian_coords, voxel_size):
    # Voxel grid downsampling
    unique_coords = np.round(cartesian_coords / voxel_size) * voxel_size
    _, indices = np.unique(unique_coords, axis=0, return_index=True)
    return cartesian_coords[indices]

def remove_outliers(cartesian_coords, nb_neighbors=20, std_ratio=2.0):
    neighbors = NearestNeighbors(n_neighbors=nb_neighbors).fit(cartesian_coords)
    distances, _ = neighbors.kneighbors(cartesian_coords)
    mean_distances = np.mean(distances, axis=1)
    std_distances = np.std(mean_distances)
    threshold = np.mean(mean_distances) + std_ratio * std_distances
    return cartesian_coords[mean_distances < threshold]

def icp(source_points, target_points, max_iterations=50, tolerance=1e-6):
    src = np.copy(source_points)
    tgt = np.copy(target_points)
    prev_error = 0

    for i in range(max_iterations):
        # Step 1: Find the nearest neighbors
        tree = KDTree(tgt)
        distances, indices = tree.query(src)
        
        # Step 2: Compute the transformation matrix using SVD
        tgt_matched = tgt[indices]
        H = np.dot((tgt_matched - tgt_matched.mean(axis=0)).T, (src - src.mean(axis=0)))
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        t = tgt_matched.mean(axis=0) - np.dot(R, src.mean(axis=0))

        # Step 3: Apply the transformation
        src = np.dot(src, R.T) + t

        # Step 4: Check for convergence
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    transformation = {'R': R, 't': t}
    return transformation, src

def update_occupancy_grid(grid, points, grid_size, cell_size, log_odds_increase=1.0, log_odds_decrease=0.5):
    log_odds_grid = np.zeros((grid_size, grid_size))
    for point in points:
        x_idx = int(point[0] / cell_size) + grid_size // 2
        y_idx = int(point[1] / cell_size) + grid_size // 2
        if 0 <= x_idx < grid_size and 0 <= y_idx < grid_size:
            log_odds_grid[x_idx, y_idx] += log_odds_increase

    free_space = np.zeros_like(grid, dtype=bool)
    for x in range(grid_size):
        for y in range(grid_size):
            if log_odds_grid[x, y] == 0:
                free_space[x, y] = True
    
    grid[free_space] -= log_odds_decrease
    grid[log_odds_grid > 0] += log_odds_increase
    grid = np.clip(grid, -10, 10)  # Clamping log odds to avoid overflow
    
    return grid

def initialize_display(grid_size):
    fig, ax = plt.subplots()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    img = ax.imshow(np.zeros((grid_size, grid_size)), cmap='gray', origin='lower')
    plt.ion()  # Interactive mode on
    plt.show()
    return fig, ax, img

# fig, ax, img = initialize_display(grid_size)

def update_display(grid, img):
    img.set_data(grid)
    plt.draw()
    plt.pause(0.01)

try:
    previousScan = None
    currentScan = None
    for scan in lidar.iter_scans():
        if len(scan) < 100:
            continue
        points = preprocess_scan(scan)
        downSampledCoords = downsample(points, 0.1)
        cleaned_coords  = remove_outliers(downSampledCoords)
        if previousScan is None:
            previousScan = cleaned_coords 
            sleep(1)
            continue
        currentScan = cleaned_coords 
        transformation, aligned_points = icp(currentScan, previousScan)
        print(f"Transforms: {transformation}")
        occupancy_grid = update_occupancy_grid(occupancy_grid, aligned_points, grid_size, cell_size)
        print(f"Occupancy grid min: {np.min(occupancy_grid)}, max: {np.max(occupancy_grid)}")
        # occupancy_grid_display = (occupancy_grid + 10) / 20
        # update_display(occupancy_grid_display, img)
        # print(f"R: {R}, t: {t}")
        # pose = apply_icp_to_pose(pose, R, t)
        # print(f"Pose: {pose[0]}, {pose[1]}, {np.deg2rad(pose[2])}")
        previousScan = currentScan
        sleep(1)
except RPLidarException as e:
    print(f"Lidar error: {e}")
    lidar.stop()
    lidar.disconnect()
except KeyboardInterrupt:
    lidar.stop()
    lidar.disconnect()
except Exception as e:
    print(f"Error: {e}")
    lidar.stop()
    lidar.disconnect()
    