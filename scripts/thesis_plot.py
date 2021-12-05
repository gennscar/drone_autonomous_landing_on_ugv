from cProfile import label
import numpy as np
from matplotlib import pyplot as plt
from ros2bag import BagFileParser

ROS2BAG_FILENAME = "rosbag2_2021_10_26-09_21_48"
ROS2BAG_FILE = "/home/cosimocon/Dev/ros2_px4_ws/rosbags/Campo_Volo_2021_11_05/" + \
    ROS2BAG_FILENAME+"/"+ROS2BAG_FILENAME+"_0.db3"
DRONE_NAMESPACE = "/X500_2"
IS_SIMULATION = False

# Anchors
anchors_pos = np.array([
    [5.0, 0.0, 1.4],
    [0.0, 0.0, 2.09],
    [0.0, -5.0, 1.41],
    [0.0, -5.0, 0.2],
    [5.0, 0.0, 0.2],
    [0.0, 0.0, 0.34]
])


# Open ros2bag
parser = BagFileParser(ROS2BAG_FILE)

# Parsing topics
local = parser.get_messages(
    DRONE_NAMESPACE+"/VehicleLocalPosition_PubSubTopic")
ukf = parser.get_messages(
    DRONE_NAMESPACE+"/UkfPositioning/Odometry")
gps = parser.get_messages(
    DRONE_NAMESPACE+"/GpsPositioning/Odometry")

if IS_SIMULATION:
    local_gt = parser.get_messages(
        DRONE_NAMESPACE+"/VehicleLocalPositionGroundtruth_PubSubTopic")

# Parsing Local
local_t = np.array([(i[0]-local[0][0])*1e-9 for i in local])
local_pos = np.array([[i[1].y, i[1].x, -i[1].z] for i in local])
local_vel = np.array([[i[1].vy, i[1].vx, -i[1].vz] for i in local])

# Parsing Groundtruth
if IS_SIMULATION:
    local_gt_t = np.array(
        [(i[0]-local_gt[0][0])*1e-9 for i in local_gt])
    local_pos_gt = np.array([[i[1].y, i[1].x, -i[1].z] for i in local_gt])
    local_vel_gt = np.array([[i[1].vy, i[1].vx, -i[1].vz] for i in local_gt])

# Parsing UKF
ukf_t = np.array([(i[0]-ukf[0][0])*1e-9 for i in ukf])
ukf_vis = np.array([i[1].pose.pose.orientation.x for i in ukf])

# Parsing GPS
gps_t = np.array([(i[0]-gps[0][0])*1e-9 for i in gps])
gps_pos_cov = np.array(
    [np.linalg.norm(i[1].pose.covariance) for i in gps])
gps_vel_cov = np.array([np.linalg.norm(i[1].twist.covariance) for i in gps])

local_t = np.delete(local_t, np.where(local_t > 140))
gps_t = np.delete(gps_t, np.where(gps_t > 140))

# 2d plot
if True:
    plt.subplot(2, 1, 1)
    plt.plot(local_t, local_pos[:12405, 0],
             label="Vehicle Local Position North")
    plt.plot(local_t, local_pos[:12405, 1],
             label="Vehicle Local Position East")
    plt.plot(local_t, local_pos[:12405, 2],
             label="Vehicle Local Position Down")
    plt.plot(gps_t, gps_pos_cov[:1397]*0.1,
             label="Gps Position Covariance Norm / 10")
    if IS_SIMULATION:
        plt.plot(local_gt_t, local_pos_gt, label="Vehicle Local Position GT")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(local_t, local_vel[:12405, 0],
             label="Vehicle Local Velocity North")
    plt.plot(local_t, local_vel[:12405, 1],
             label="Vehicle Local Velocity East")
    plt.plot(local_t, local_vel[:12405, 2],
             label="Vehicle Local Velocity Down")
    plt.plot(gps_t, gps_vel_cov[:1397]*0.1,
             label="Gps Velocity Covariance Norm / 10")
    if IS_SIMULATION:
        plt.plot(local_gt_t, local_vel_gt, label="Vehicle Local Velocity GT")
    plt.legend()

    plt.show()

# 3D plot
if True:
    plt.subplot(111, projection='3d')
    plt.plot(local_pos[:12405, 0]+4, local_pos[:12405, 1]-4,
             local_pos[:12405, 2], label="Vehicle Local Position")
    if IS_SIMULATION:
        plt.plot(local_pos_gt[:, 0], local_pos_gt[:, 1],
                 local_pos_gt[:, 2], label="Vehicle Local Position Groundtruth")
    plt.scatter(anchors_pos[:, 0], anchors_pos[:, 1],
                zs=anchors_pos[:, 2], label="Anchor", s=100.)

    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.legend()
    plt.show()

# Error plot
if False:
    # Interpolation
    local_pos_int = np.zeros((len(local_gt_t), 3))
    local_pos_int[:, 0] = np.interp(local_gt_t, local_t, local_pos[:, 0])
    local_pos_int[:, 1] = np.interp(local_gt_t, local_t, local_pos[:, 1])
    local_pos_int[:, 2] = np.interp(local_gt_t, local_t, local_pos[:, 2])

    local_vel_int = np.zeros((len(local_gt_t), 3))
    local_vel_int[:, 0] = np.interp(local_gt_t, local_t, local_vel[:, 0])
    local_vel_int[:, 1] = np.interp(local_gt_t, local_t, local_vel[:, 1])
    local_vel_int[:, 2] = np.interp(local_gt_t, local_t, local_vel[:, 2])

    # Error MSE
    error_pos = np.linalg.norm(local_pos_int-local_pos_gt, axis=1)**2
    error_vel = np.linalg.norm(local_vel_int-local_vel_gt, axis=1)**2

    plt.plot(local_gt_t, error_pos, label="Vehicle Local Position MSE")
    plt.plot(local_gt_t, error_vel, label="Vehicle Local Velocity MSE")
    plt.plot(ukf_t, ukf_vis, label="Sensor state")
    plt.legend()
    plt.grid()
    plt.xlabel("Time [s]")
    plt.ylabel("MSE [m]")
    plt.show()
