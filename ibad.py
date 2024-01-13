import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Function definitions
def Background_Subtraction ( pcd, base_cloud, distance_threshold = 1.0 ):
    dists = pcd.compute_point_cloud_distance(base_cloud)
    dists = np.asarray(dists)
    ind = np.where(dists > distance_threshold)[0]
    bg_subtracted_pcd = pcd.select_by_index(ind)
    return bg_subtracted_pcd

def Visualize_Point_Cloud (vis, pcd):
    vis.clear_geometries()
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    return

def Visualize (vis):
    vis.poll_events()
    vis.update_renderer()
    return

def Clear_Visualizer (vis):
    vis.clear_geometries()

def Add_To_Visualizer ( vis, pcd, bbox ):
    vis.add_geometry(pcd)
    vis.add_geometry(bbox)
    return

def Noise_Removal ( pcd ):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20,
                                            std_ratio=2)
    inlier_cloud = pcd.select_by_index(ind)
    return inlier_cloud

def Find_And_Remove_Ground_Plane (pcd):
    # Find the ground plane in the pcd and remove it
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    # print("Plane equation: {}x + {}y + {}z + {} = 0".format(a, b, c, d))
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud


def Load_Point_Cloud ( path_to_cloud ):
    return o3d.io.read_point_cloud ( path_to_cloud )
    

def Clustering ( pcd ):
    # Find clusters in the point cloud
    labels = np.array ( pcd.cluster_dbscan (eps=2.0, min_points=3, print_progress=False) )
    # if (labels.size == 0):
        # print ("No clusters found")
        # pass
    # else:
        # max_label = labels.max()
        # print("point cloud has {} clusters".format(max_label + 1))
    return labels

def Color_Point_Cloud ( pcd, labels ):
    # Color the point cloud
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return pcd

def Compute_Centroid ( points ):
    # Compute the 3D centroid of the points
    centroid = np.mean ( points, axis = 0 )
    return centroid

def Compute_Bounding_Box ( points ):
    # Compute the bounding box for the points
    bbox = points.get_oriented_bounding_box()
    return bbox

def Get_Euclidean_Distance ( point_one, point_two ):
    # Compute the euclidean distance between two points
    return np.linalg.norm ( point_one - point_two )

def Associate_Vehicle_To_Previous_Frame ( current_position, previous_positions ):
    least_distance = 100
    prev_idx = 0
    prev_pos = 0
    i = 0
    for pt in previous_positions:
        if ( Get_Euclidean_Distance ( current_position, pt ) < least_distance ):
            prev_pos = pt
            idx = i
            least_distance = Get_Euclidean_Distance ( current_position, pt )
        else:
            pass
        i += 1

    return least_distance, prev_pos, prev_idx

def List_To_Array ( list_ds ):
    return np.asarray ( list_ds )


##########
# Main
##########

# Constants
path_to_data = "dataset/PointClouds/"
start_index = 0
end_index = 499
MAX_DISTANCE_PER_FRAME = 0.5 # meters
NUMBER_OF_FRAMES = end_index - start_index + 1
MAX_VEHICLES = 10
# Load the base cloud
base_cloud = o3d.io.read_point_cloud(path_to_data + str(start_index) + ".pcd")
# Compute non-ground points for base cloud
base_cloud_road, base_cloud_non_road = Find_And_Remove_Ground_Plane ( base_cloud )

# Create visualization window
vis = o3d.visualization.Visualizer()
vis.create_window ()

# Positions is a 3-dimensional array that stores the positions of the vehicles
positions = np.array ( np.array ( MAX_VEHICLES * [ NUMBER_OF_FRAMES * [ np.zeros ( 3 ) ] ] ) )

vehicle_trajectories = []

per_vehicle_trajectories = []
number_of_vehicles_detected = 0

# Create a csv file to store the vehicle positions and velocities
csv_file = open ( "vehicle_positions.csv", "w" )

# Load each point cloud and display it
for frame_number in range ( start_index, end_index ) :
    current_cloud_index = frame_number

    ## Loading point cloud
    print ("Point cloud: " + str (current_cloud_index))
    pcd = Load_Point_Cloud (path_to_data + str(current_cloud_index) + ".pcd")
    
    ## Removing the ground plane
    # print ("Removing the ground plane ...")
    inlier_cloud, outlier_cloud = Find_And_Remove_Ground_Plane (pcd)

    ## Background subtraction
    # print ("Removing BG ...")
    bg_subtracted_pc = Background_Subtraction (outlier_cloud, base_cloud )

    ## Clustering
    # print ("Clustering ...")
    labels = Clustering (bg_subtracted_pc)

    ## Visualization
    # Visualize_Point_Cloud ( vis, pcd )
    Visualize_Point_Cloud ( vis, outlier_cloud )
    # Visualize_Point_Cloud ( vis, inlier_cloud )
    # Visualize_Point_Cloud ( vis, bg_subtracted_pc )


    plt.pause (0.1)

    # Variable to store vehicle positions for this frame
    vehicle_positions = []

    ## Bounding Box and Centroid Estimation
    if ( labels.size != 0 ): # if there were any vehicles detected this frame
        total_clusters = labels.max() + 1
        # For each cluster, extract the 3D points, and compute their bounding box and centroid
        for cluster_id in range ( total_clusters ):
            # Find the indices of the points belonging to the cluster
            cluster_point_indices = np.asarray ( np.where ( labels == cluster_id ) [0] ) 
            # Extract the points belonging to the cluster
            cluster_point_positions = bg_subtracted_pc.select_by_index ( cluster_point_indices )
            cluster_centroid = Compute_Centroid ( cluster_point_positions.points )
            vehicle_positions.append ( cluster_centroid )
    else:
        pass

    vehicle_trajectories.append ( np.asarray ( vehicle_positions ) )


    # Update positions
    if ( len ( vehicle_trajectories ) > 1 ):
        current_frame_vehicle_positions = np.asarray ( vehicle_positions )
        last_frame_vehicle_positions = np.asarray ( vehicle_trajectories[-2] )

        # Associate vehicle positions to previous frame
        vehicle_id = 0
        for pos in current_frame_vehicle_positions:
            distance, prev_pos, prev_idx = Associate_Vehicle_To_Previous_Frame ( pos, last_frame_vehicle_positions )

            if ( distance < MAX_DISTANCE_PER_FRAME ):
                mvec = ( pos - prev_pos ) * 10
                print ("Vehicle [" + str ( vehicle_id ) + "]: pos: "\
                       + str ( pos ) + ", m_vec: " + str ( mvec ) )
                # Write to csv file
                csv_file.write ( str ( frame_number ) + "," + str ( vehicle_id ) + "," + str ( pos[0] ) + "," + str ( pos[1] ) + "," + str ( pos[2] ) + "," + str ( mvec[0] ) + "," + str ( mvec[1] ) + "," + str ( mvec[2] ) + "\n" )
                # associated to previous frame")

            vehicle_id += 1

    else:
        for vehicle_id in range ( len ( vehicle_positions ) ):
            positions [frame_number, vehicle_id] = vehicle_positions [ vehicle_id ]
        per_vehicle_trajectories.append ( np.asarray ( vehicle_positions ) )

 
# print (vehicle_trajectories)
i = 0
vehicle_trajectory_array = []
for frame in vehicle_trajectories:
    # Put each item from frame in vehicle_trajectory_array
    for pos in frame:
        vehicle_trajectory_array.append ( pos )
    # vehicle_trajectory_array.append ( np.asarray ( frame ) )
    # print (str(i) + ": " + str(frame))


    # print (str(i) + ": " + str(np.asarray(frame)))
    i += 1

vehicle_trajectory_array = np.asarray ( vehicle_trajectory_array )
# Convert vehicle_trajectories to 2D array
# vehicle_trajectories = np.asarray ( vehicle_trajectories )

# Display vehicle trajectories
# vehicle_trajectories = np.asarray ( vehicle_trajectories )
# print (np.array(vehicle_trajectories).shape)
print ( vehicle_trajectory_array.shape )
plt.plot (vehicle_trajectory_array[:,0], vehicle_trajectory_array[:,1], 'ro')
plt.show ()

# Close visualization win
