Main Execution:

Constants: Defines some constants like the path to data, start and end indices of frames, maximum distance per frame, number of frames, and maximum number of vehicles.
Base Cloud: Reads the base point cloud from the specified path and removes the ground plane from it.
Visualization: Initializes a visualization window.
Loop Over Frames: Iterates over each frame of point cloud data.
Loading Point Cloud: Loads the point cloud for the current frame.
Ground Plane Removal: Removes the ground plane from the point cloud.
Background Subtraction: Removes background points using the base cloud.
Clustering: Clusters remaining points to identify potential objects (like vehicles).
Visualization: Visualizes the processed point clouds.
Bounding Box and Centroid Estimation: Computes bounding boxes and centroids for detected clusters.
Association with Previous Frame: Matches detected vehicle positions in the current frame with those from the previous frame, if applicable.
Writing Results to CSV: Writes the frame number, vehicle ID, position, and velocity to a CSV file.
Vehicle Trajectories: Stores the positions of detected vehicles over all frames.
Visualization: Plots the trajectories of detected vehicles.

find and remove ground
takes point cloud input 
aims to identify and remove ground from point cloud

segment_plane() = uses RANSAC algo to fit a plane model to point cloud data. ransac iteratively selects random subsets of points, fits a model to each subset and identifies subset that fits data. helps in robustly estimating paramters in ground plane
                  distance threshold specifies max distance a point can be from fitted plane to be considered inliner
other parameters ar to adjust ransac
inlier_cloud = contains points that are part of ground plane
outlier_cloud = contains remaining points that are not part of ground plane

and then both point clouds are returned
otheroptions, ICP (not robust to outliers), least square estimation, hough transform
why ransac? ransac is robustness to outliers, adaptable to various types of geometric models making it suitable for plane segmentation.

segment plane uses RANSAC algoto find model of plane that best fits given criteria from point cloud. method returns coeffiecients of plane equation ax+by+cz+d=0 and indices of points that fits model with specific threshold
__Ransac is used to identify patterns or models within datasets that contain significant number of outlier. when finding a plane in 3d point cloud ransac tries to find plane that best represents main surface within cloud__
process explaination using the RANSAC algorithm to find the model of the plane that best fits the given criteria from the point cloud



