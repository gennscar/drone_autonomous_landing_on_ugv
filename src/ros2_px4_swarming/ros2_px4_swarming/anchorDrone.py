#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node
from px4_msgs.msg import Timesync, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import UInt64, String
from ros2_px4_interfaces.msg import UnitVector, UwbSensor, VelocityVector
from ros2_px4_interfaces.srv import DroneCustomCommand
from ros2_px4_functions import swarming_functions
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class Drone(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml" (for simulation) or
        "drone_params.yaml" (for test). It defines a data structure including all the drones functionalities. It
        declares useful variables, publishers, subscribers, and services, and then initializes them through the
        "self.initializeDrone()" method. It creates a timer to regulate the frequency of the controller.
        """

        super().__init__("anchorDrone")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),                         # node frequency
                ('QUEUE_SIZE', None),
                ('MAX_TAKEOFF_SPEED', None),            # vertical saturation speed
                ('A', None),                            # swarming algorithm parameters
                ('B', None),                            # -
                ('C', None),                            # -
                ('H_DES', None),                        # swarming altitude
                ('K_H', None),                          # altitude control proportional coefficient
                ('EARTH_RADIUS', None),                 # Earth radius
                ('TARGET_ID', None),                    # ID of the target UWB sensor
                ('MAX_SWARMING_SPEED', None),           # horizontal swarming saturation speed
                ('TIMEOUT', None),                      # timeout for lost drone
                ('UWB_ON', None),                       # if the UWB sensor is on or off on the drone
                ('SINGLE_DRONE_FOR_TEST', None),        # to fly a single drone with this code
                ('BEST_EFFORT', None),                  # if the subscribers' QoS is set to best_effort or reliable
                ('ID', None),                           # drone ID
                ('MAVSYS_ID', None),                    # drone MAVSYS ID
                ('NOT_FLYING_FOR_TEST', None)           # if the drone is thought to just send data without being armed
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.MAX_TAKEOFF_SPEED = self.get_parameter('MAX_TAKEOFF_SPEED').value
        self.A = self.get_parameter('A').value
        self.B = self.get_parameter('B').value
        self.C = self.get_parameter('C').value
        self.H_DES = self.get_parameter('H_DES').value
        self.K_H = self.get_parameter('K_H').value
        self.EARTH_RADIUS = self.get_parameter('EARTH_RADIUS').value
        self.TARGET_ID = self.get_parameter('TARGET_ID').value
        self.MAX_SWARMING_SPEED = self.get_parameter('MAX_SWARMING_SPEED').value
        self.TIMEOUT = self.get_parameter('TIMEOUT').value
        self.UWB_ON = self.get_parameter('UWB_ON').value
        self.SINGLE_DRONE_FOR_TEST = self.get_parameter('SINGLE_DRONE_FOR_TEST').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        self.ID = self.get_parameter('ID').value
        self.MAVSYS_ID = self.get_parameter('MAVSYS_ID').value
        self.NOT_FLYING_FOR_TEST = self.get_parameter('NOT_FLYING_FOR_TEST').value
        # endregion

        # Drone operations: methods to exploit specific drone's functionalities
        self.droneOperations = {
            "idle": self.idle,
            "takeoff": self.takeoff,
            "hover": self.hover,
            "swarming": self.swarming,
            "goTo": self.goTo,
            "setHorizontalVelocity": self.setHorizontalVelocity,
            "returnHome": self.returnHome,
            "land": self.land,
            "restart": self.restart
        }

        # region Variables declaration
        self.N = None                                   # int                               number of members of the swarm
        self.activeDrones = None                        # dict(int: bool)                   ID of the active drones
        self.timestamp = None                           # int                               time since system start in microseconds
        self.loggerCounter = None                       # int                               counter to log at a desired rate
        self.armCounter = None                          # int                               counter to send the arming command multiple times
        self.lastPositionReceivedTimer = None           # dict(int: int)                    time from the last received GPS data from every swarm member
        self.authorizedForOffboard = None               # bool                              is the drone ready for offboard mode?
        self.readyForIdle = None                        # bool                              is the drone ready for idle mode?
        self.readyForTakeoff = None                     # bool                              is the drone ready for takeoff mode?
        self.readyForSwarming = None                    # bool                              is the drone ready for swarming mode?
        self.anchorsPositionReceived = None             # bool                              has the drone received GPS data from every swarm member?
        self.trackingVelocityReceived = None            # bool                              has the drone received data from the tracking controller?
        self.anchorsReadyForTakeoff = None              # dict(int: bool)                   is every swarm member ready for takeoff mode?
        self.anchorsReadyForSwarming = None             # dict(int: bool)                   is every swarm member ready for swarming mode?
        self.lastGoalPositionReceived = None            # Point                             last commanded goal position
        self.lastGoalVelocityReceived = None            # Twist                             last commanded goal velocity
        self.trajectorySetpoint = None                  # TrajectorySetpoint                position and velocity information about the trajectory to be followed
        self.localPosition = None                       # VehicleLocalPosition              drone's local position with respect to its initial location
        self.vehicleStatus = None                       # VehicleStatus                     vehicle status of the current drone
        self.home = None                                # 3 * [VehicleLocalPosition]        home (flying) position of the current drone
        self.anchorsPosition = None                     # dict(int: VehicleGlobalPosition)  GPS positions of the swarm members
        self.unitVectors = None                         # dict(int: UnitVector)             unit vectors indicating the direction from the current drone to every swarm member
        self.trackingVelocity = None                    # VelocityVector                    tracking velocity received by the centralized controller
        self.interDronesDistances = None                # dict(int: float)                  inter-drones distances from GPS positions
        self.droneMode = None                           # str                               current drone mode
        self.droneModeOld = None                        # str                               previous drone mode
        self.nodeDestroyed = None                       # bool                              has the node been destroyed?
        self.NAtStart = None                            # int                               number of drones in the formation at start
        if self.UWB_ON:
            self.uwbDistancesReceived = None            # bool                              has the drone received UWB data from every swarm member?
            self.lastUWBDataReceivedTimer = None        # dict(int: int)                    time from the last received UWB data from every swarm member
            self.uwbDistances = None                    # dict(int: float)                  inter-drones distances from UWB sensor
        # endregion

        # region Subscribers, publishers and services declaration
        # QoS profile
        self.qosProfile = None

        # Subscribers
        self.timesyncSub = None                         # subscription to timesync
        self.numAnchorsSub = None                       # subscription to get number of drones in the swarm
        self.localPositionSub = None                    # subscription to current drone's local position
        self.vehicleStatusSub = None                    # subscription to current drone's status
        self.readyForTakeoffSub = None                  # subscriptions to get which member of the swarm is ready for takeoff
        self.readyForSwarmingSub = None                 # subscriptions to get which member of the swarm is ready for swarming
        self.anchorsPositionSubs = None                 # subscriptions to swarm members' GPS position
        self.trackingVelocitySub = None                 # subscription to tracking velocity centralized controller
        if self.UWB_ON:
            self.uwbSensorSub = None                    # subscription to UWB sensor's data

        # Publishers
        self.vehicleCommandPub = None                   # publisher to send vehicle commands
        self.offboardControlModePub = None              # publisher to send offboard control
        self.readyForTakeoffPub = None                  # publisher to communicate when the current drone is ready for takeoff
        self.readyForSwarmingPub = None                 # publisher to communicate when the current drone is ready for swarming
        self.trajectorySetpointPub = None               # publisher to send trajectory setpoint
        self.vehiclesInfoPub = None                     # publisher to send information about the vehicles of the swarm

        # Services
        self.droneCommandSrv = None                     # service to receive external commands
        # endregion

        self.initializeDrone()

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

        # Publish 100 trajectory setpoints
        for i in range(100):
            self.publishTrajectorySetpoint()

    # region Auxiliary methods
    def initializeDrone(self):
        """
        It initializes variables, QOS profile, publishers, subscribers, and services.
        """

        # Variables initialization
        self.N = None
        self.activeDrones = {}
        self.timestamp = 0
        self.loggerCounter = 0
        self.armCounter = 0
        self.lastPositionReceivedTimer = {}
        self.authorizedForOffboard = True
        self.readyForIdle = True
        self.readyForTakeoff = False
        self.readyForSwarming = False
        self.anchorsPositionReceived = False
        self.trackingVelocityReceived = False
        self.anchorsReadyForTakeoff = {}
        self.anchorsReadyForSwarming = {}
        self.lastGoalPositionReceived = Point()
        self.lastGoalVelocityReceived = Twist()
        self.trajectorySetpoint = TrajectorySetpoint()
        self.localPosition = VehicleLocalPosition()
        self.vehicleStatus = VehicleStatus()
        self.home = [None, None, None]
        self.anchorsPosition = {}
        self.unitVectors = {}
        self.trackingVelocity = None
        self.interDronesDistances = {}
        self.droneMode = "disarmed"
        self.droneModeOld = "disarmed"
        self.nodeDestroyed = False
        self.NAtStart = None
        if self.UWB_ON:
            self.uwbDistancesReceived = False
            self.lastUWBDataReceivedTimer = {}
            self.uwbDistances = {}

        # QOS initialization
        if self.BEST_EFFORT:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )
        else:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )

        # Subscribers initialization
        # self.timesyncSub = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesyncCallback, self.QUEUE_SIZE)
        # self.numAnchorsSub = self.create_subscription(UInt64, "/numAnchorsNode/N", self.numAnchorsCallback, self.QUEUE_SIZE)
        # self.localPositionSub = self.create_subscription(VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.localPositionCallback, self.QUEUE_SIZE)
        # self.vehicleStatusSub = self.create_subscription(VehicleStatus, "VehicleStatus_PubSubTopic", self.vehicleStatusCallback, self.QUEUE_SIZE)
        # self.readyForTakeoffSub = self.create_subscription(UInt64, "/readyForTakeoff", self.anchorReadyForTakeoffCallback, self.QUEUE_SIZE)
        # self.readyForSwarmingSub = self.create_subscription(UInt64, "/readyForSwarming", self.anchorReadyForSwarmingCallback, self.QUEUE_SIZE)
        # self.anchorsPositionSubs = {}
        # self.trackingVelocitySub = self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocity", self.trackingVelocityCallback, self.QUEUE_SIZE)
        # if self.UWB_ON:
        #     self.uwbSensorSub = self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.ID), self.uwbSensorCallback, self.QUEUE_SIZE)
        self.timesyncSub = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesyncCallback, qos_profile=self.qosProfile)
        self.numAnchorsSub = self.create_subscription(UInt64, "/numAnchorsNode/N", self.numAnchorsCallback, qos_profile=self.qosProfile)
        self.localPositionSub = self.create_subscription(VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.localPositionCallback, qos_profile=self.qosProfile)
        self.vehicleStatusSub = self.create_subscription(VehicleStatus, "VehicleStatus_PubSubTopic", self.vehicleStatusCallback, qos_profile=self.qosProfile)
        self.readyForTakeoffSub = self.create_subscription(UInt64, "/readyForTakeoff", self.anchorReadyForTakeoffCallback, qos_profile=self.qosProfile)
        self.readyForSwarmingSub = self.create_subscription(UInt64, "/readyForSwarming", self.anchorReadyForSwarmingCallback, qos_profile=self.qosProfile)
        self.anchorsPositionSubs = {}
        self.trackingVelocitySub = self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocity", self.trackingVelocityCallback, qos_profile=self.qosProfile)
        if self.UWB_ON:
            self.uwbSensorSub = self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.ID), self.uwbSensorCallback, qos_profile=self.qosProfile)

        # Publishers initialization
        # self.vehicleCommandPub = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", self.QUEUE_SIZE)
        # self.offboardControlModePub = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", self.QUEUE_SIZE)
        # self.readyForTakeoffPub = self.create_publisher(UInt64, "/readyForTakeoff", self.QUEUE_SIZE)
        # self.readyForSwarmingPub = self.create_publisher(UInt64, "/readyForSwarming", self.QUEUE_SIZE)
        # self.trajectorySetpointPub = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", self.QUEUE_SIZE)
        # self.vehiclesInfoPub = self.create_publisher(String, "/vehiclesInfo", self.QUEUE_SIZE)
        self.vehicleCommandPub = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", self.qosProfile)
        self.offboardControlModePub = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", self.qosProfile)
        self.readyForTakeoffPub = self.create_publisher(UInt64, "/readyForTakeoff", self.qosProfile)
        self.readyForSwarmingPub = self.create_publisher(UInt64, "/readyForSwarming", self.qosProfile)
        self.trajectorySetpointPub = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", self.qosProfile)
        self.vehiclesInfoPub = self.create_publisher(String, "/vehiclesInfo", self.qosProfile)

        # Services initialization
        self.droneCommandSrv = self.create_service(DroneCustomCommand, "DroneCustomCommand", self.droneCommandCallback)

    def publishVehicleCommand(self, command, param1, param2):
        """
        @param command: command's code
        @param param1: first parameter
        @param param2: second parameter

        It generates and publishes a vehicle command's message, starting from the command's code.
        """

        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.MAVSYS_ID
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicleCommandPub.publish(msg)

    def publishOffboardControlMode(self):
        """
        It generates and publishes the message stating what is controlled (position, velocity, ...) when the drone is in
        offboard mode.
        """

        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboardControlModePub.publish(msg)

    def setTrajectorySetpoint(self, x=float("NaN"), y=float("NaN"), z=float("NaN"), yaw=float("NaN"), yawspeed=float("NaN"),
                              vx=float("NaN"), vy=float("NaN"), vz=float("NaN"), acceleration=3*[float("NaN")], jerk=3*[float("NaN")], thrust=3*[float("NaN")]):
        """
        @param x: x position
        @param y: y position
        @param z: z position
        @param yaw: yaw attitude
        @param yawspeed: yaw velocity
        @param vx: x velocity
        @param vy: y velocity
        @param vz: z velocity
        @param acceleration: acceleration triplet (ax, ay, az)
        @param jerk: jerk triplet (jx, jy, jz)
        @param thrust: thrust triplet (tx, ty, tz)

        It sets the desired value to each field of the trajectory setpoint. If a specific field is not explicitly set,
        it is set to float("NaN"), as the autopilot requires.
        """

        self.trajectorySetpoint.x = float(x)
        self.trajectorySetpoint.y = float(y)
        self.trajectorySetpoint.z = float(z)
        self.trajectorySetpoint.yaw = float(yaw)
        self.trajectorySetpoint.yawspeed = float(yawspeed)
        self.trajectorySetpoint.vx = float(vx)
        self.trajectorySetpoint.vy = float(vy)
        self.trajectorySetpoint.vz = float(vz)
        self.trajectorySetpoint.acceleration = acceleration
        self.trajectorySetpoint.jerk = jerk
        self.trajectorySetpoint.thrust = thrust

    def publishTrajectorySetpoint(self):
        """
        It adds the timestamp to the trajectory setpoint message, and publishes it on the correct topic.
        """

        msg = self.trajectorySetpoint
        msg.timestamp = self.timestamp

        self.publishOffboardControlMode()
        self.trajectorySetpointPub.publish(msg)

    def computeInterDronesDistances(self):
        """
        It computes the distance (in meters) between itself and any other drone in the swarm, starting from their GNSS
        position, and storing the values in the dictionary "self.interDroneDistances".
        """

        for j in self.anchorsPosition.keys():
            if j != self.ID:
                a = (math.sin(math.radians(self.anchorsPosition[j].lat - self.anchorsPosition[self.ID].lat) / 2))**2 +\
                    math.cos(math.radians(self.anchorsPosition[self.ID].lat)) * math.cos(math.radians(self.anchorsPosition[j].lat)) * (math.sin(math.radians(self.anchorsPosition[j].lon - self.anchorsPosition[self.ID].lon) / 2))**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
                self.interDronesDistances[j] = self.EARTH_RADIUS * c

    def armVehicle(self):
        """
        If the vehicle is not armed, and it is authorized to start, it arms the vehicle, otherwise it warns the user and
        sets its mode to idle.
        """

        if self.authorizedForOffboard:
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            self.setTrajectorySetpoint()
            self.armCounter += 1
        else:
            self.get_logger().warn("Vehicle not authorized for offboard")
            self.publishDroneInfo("Vehicle not authorized for offboard")
            self.droneMode = "idle"

    def publishDroneInfo(self, message):
        """
        @param message: message to be displayed

        During tests, it publishes a message to be displayed on the computer running the ground station.
        """

        msg = String()
        msg.data = "Drone " + str(self.ID) + ": " + message
        self.vehiclesInfoPub.publish(msg)

    def droneNotFlyingOperations(self):
        """
        For test, a drone communicates with the swarm that it is flying even though it is still on the ground, so it can
        be hand-moved to check if the swarming algorithm works as expected. With this method, the drone communicates
        that it is ready for takeoff, then ready for swarming, then goes to idle mode and keeps publishing its info,
        global position, and UWB data to the rest of the system.
        """

        if not self.readyForTakeoff:
            # Communicate to the other anchors that I am ready to takeoff
            self.readyForTakeoff = True
            self.anchorsReadyForTakeoff[self.ID] = True
            msg = UInt64()
            msg.data = self.ID
            self.readyForTakeoffPub.publish(msg)
            self.get_logger().info("Setting (fake) takeoff mode")
            self.publishDroneInfo("Setting (fake) takeoff mode")
        else:
            msg = UInt64()
            msg.data = self.ID
            self.readyForTakeoffPub.publish(msg)

        if not self.readyForSwarming:
            # Communicate to the other anchors that I am ready to start swarming
            self.readyForSwarming = True
            self.anchorsReadyForSwarming[self.ID] = True
            msg = UInt64()
            msg.data = self.ID
            self.readyForSwarmingPub.publish(msg)
            self.get_logger().info("Setting (fake) swarming mode")
            self.publishDroneInfo("Setting (fake) swarming mode")
        else:
            msg = UInt64()
            msg.data = self.ID
            self.readyForSwarmingPub.publish(msg)
            self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)

        self.idle()
    # endregion

    # region Services' handling methods
    def idle(self):
        """
        @return: None

        In idle mode, the drone just publishes its info without doing anything else. When turned on, the drone goes in
        idle mode, waiting for tasks.
        """

        return

    def takeoff(self):
        """
        @return: None

        It handles the takeoff phase of the flight. First, it communicates to the other vehicles that it is ready, and
        waits for all of them to be as well. Then, it gets armed and starts moving upwards to the desired position with
        velocity proportional to its distance from the goal, limited to MAX_TAKEOFF_SPEED. Once it has been reached, the
        current position is saved as "self.home", and the mode is set to swarming.
        """

        if not self.readyForTakeoff:
            # Communicate to the other anchors that I am ready to takeoff
            self.readyForTakeoff = True
            self.anchorsReadyForTakeoff[self.ID] = True
        msg = UInt64()
        msg.data = self.ID
        self.readyForTakeoffPub.publish(msg)

        if not self.SINGLE_DRONE_FOR_TEST and len(self.anchorsReadyForTakeoff) < self.N:
            # The other anchors are not ready to takeoff yet
            # msg = UInt64()
            # msg.data = self.ID
            # self.readyForTakeoffPub.publish(msg)
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Waiting other drones to takeoff")
                self.publishDroneInfo("Waiting other drones to takeoff")
            return

        # Check if the commanded altitude is high enough
        if self.lastGoalPositionReceived.z > -self.H_DES:
            self.lastGoalPositionReceived.z = -self.H_DES

        # Arm the drone (if necessary)
        if self.vehicleStatus.arming_state != VehicleStatus.ARMING_STATE_ARMED or self.armCounter < 2:
            self.armVehicle()

        if self.localPosition.z <= self.lastGoalPositionReceived.z:
            # Takeoff completed
            self.home = [self.localPosition.x, self.localPosition.y, self.localPosition.z]
            self.droneMode = "swarming"
            return

        # Compute the takeoff speed and saturate it to a maximum value
        takeoffSpeed = self.lastGoalPositionReceived.z - self.localPosition.z
        if abs(takeoffSpeed) > self.MAX_TAKEOFF_SPEED:
            takeoffSpeed *= (self.MAX_TAKEOFF_SPEED / abs(takeoffSpeed))
        self.setTrajectorySetpoint(vz=takeoffSpeed)

    def hover(self):
        """
        It simply sets the velocity to zero on the three axis, but it could not be very precise. A more precise hovering
        could be obtained by setting the target position to the current (local) one.
        """

        self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)

    def goTo(self):
        """
        It sets the (local) setpoint to the desired one, so the drone reaches the destination with the velocity set by
        the standard position controller.
        """

        self.setTrajectorySetpoint(x=self.lastGoalPositionReceived.x, y=self.lastGoalPositionReceived.y, z=self.lastGoalPositionReceived.z)

    def setHorizontalVelocity(self):
        """
        It sets the setpoint horizontal velocity as desired.
        """

        self.setTrajectorySetpoint(vx=self.lastGoalVelocityReceived.linear.x, vy=self.lastGoalVelocityReceived.linear.y)

    def swarming(self):
        """
        @return: None

        It waits that the whole swarm completed the takeoff phase, then it checks if all the drones are still sending
        their data, and if it is not the case, the ones that are not communicating anymore are removed from the
        formation. It computes the distances from the other drones through "self.computeInterDroneDistances()" and the
        direction unit vectors between them through "swarming_functions.computeUnitVector(...)". Then the actual
        swarming algorithm is applied, using either the distances previously computed or the ones read by the UWB
        sensors. The horizontal swarming velocity saturates at MAX_SWARMING_SPEED and the vertical one at
        MAX_TAKEOFF_SPEED. If present, the tracking velocity is added to the swarming component, and then the total
        velocity is saved as the new trajectory setpoint of the drone.
        """

        if (self.UWB_ON and (not self.anchorsPositionReceived or not self.uwbDistancesReceived)) or (not self.UWB_ON and not self.anchorsPositionReceived) or self.SINGLE_DRONE_FOR_TEST or self.NAtStart == 1:
            # Complete info from the other anchors has not arrived yet
            # self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)
            if self.loggerCounter % self.RATE == 0 and not self.SINGLE_DRONE_FOR_TEST and self.NAtStart != 1:
                self.get_logger().warn("Waiting info for swarming")
                self.publishDroneInfo("Waiting info for swarming")
            self.returnHome()
            return

        if not self.readyForSwarming:
            self.readyForSwarming = True
            self.anchorsReadyForSwarming[self.ID] = True
        # Communicate to the other anchors that I am ready to start swarming
        msg = UInt64()
        msg.data = self.ID
        self.readyForSwarmingPub.publish(msg)

        if not self.SINGLE_DRONE_FOR_TEST and len(self.anchorsReadyForSwarming) < self.N:
            # The other anchors are not ready to swarm yet
            # msg = UInt64()
            # msg.data = self.ID
            # self.readyForSwarmingPub.publish(msg)
            # self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Waiting other drones for swarming")
                self.publishDroneInfo("Waiting other drones for swarming")
            self.returnHome()
            return

        # Check if all the drones are still active
        keysToDelete = []
        for key in self.lastPositionReceivedTimer.keys():
            if self.lastPositionReceivedTimer[key] >= self.TIMEOUT * self.RATE:
                self.N -= 1
                keysToDelete.append(key)
                if self.UWB_ON:
                    for dataStructure in [self.anchorsReadyForTakeoff, self.anchorsReadyForSwarming, self.anchorsPosition, self.unitVectors, self.uwbDistances]:
                        assert type(dataStructure) is dict, self.get_logger().error("Something wrong with the data structures")
                        if key in dataStructure.keys():
                            dataStructure.pop(key)
                else:
                    for dataStructure in [self.anchorsReadyForTakeoff, self.anchorsReadyForSwarming, self.anchorsPosition, self.unitVectors, self.interDronesDistances]:
                        assert type(dataStructure) is dict, self.get_logger().error("Something wrong with the data structures")
                        if key in dataStructure.keys():
                            dataStructure.pop(key)
                for subscriber in [self.anchorsPositionSubs]:
                    assert type(subscriber) is dict, self.get_logger().error("Something wrong with the subscribers")
                    if key in subscriber.keys():
                        subscriber[key].destroy()
                        subscriber.pop(key)
        for key in keysToDelete:
            if key in self.activeDrones.keys():
                self.activeDrones.pop(key)
            if key in self.lastPositionReceivedTimer.keys():
                self.lastPositionReceivedTimer.pop(key)
            if self.UWB_ON:
                if key in self.lastUWBDataReceivedTimer.keys():
                    self.lastUWBDataReceivedTimer.pop(key)

        if self.ID not in self.anchorsPosition.keys():
            self.get_logger().info("Drone %d is no longer part of the swarm" % self.ID)
            self.publishDroneInfo("No longer part of the swarm")
            self.nodeDestroyed = True
            self.destroy_node()
            return

        if self.N == 1:
            self.hover()
            self.droneMode = "hover"
            return

        # Compute the distances between the drones from their GPS positions
        if not self.UWB_ON and self.anchorsPositionReceived:
            self.computeInterDronesDistances()

        # Compute the unit vectors for the swarming algorithm
        for key in self.anchorsPosition.keys():
            if key != self.ID:
                self.unitVectors[key] = swarming_functions.computeUnitVector(self.ID, self.anchorsPosition[self.ID], key, self.anchorsPosition[key], self.get_clock().now().to_msg())

        velEast = 0.0
        velNorth = 0.0
        # Swarming velocity
        if self.UWB_ON:
            for key in self.uwbDistances.keys():
                if key != self.TARGET_ID:
                    componentEast = self.uwbDistances[key] * self.unitVectors[key].east * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                    componentNorth = self.uwbDistances[key] * self.unitVectors[key].north * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                    velEast -= componentEast
                    velNorth -= componentNorth
        else:
            anchorsPositionUpToDate = True
            for key in self.lastPositionReceivedTimer.keys():
                if self.lastPositionReceivedTimer[key] > 1 * self.RATE:
                    anchorsPositionUpToDate = False
                    break
            if anchorsPositionUpToDate:
                for key in self.anchorsPosition.keys():
                    if key != self.ID:
                        componentEast = self.interDronesDistances[key] * self.unitVectors[key].east * (self.A - self.B * math.exp(-self.interDronesDistances[key]**2 / self.C))
                        componentNorth = self.interDronesDistances[key] * self.unitVectors[key].north * (self.A - self.B * math.exp(-self.interDronesDistances[key]**2 / self.C))
                        velEast -= componentEast
                        velNorth -= componentNorth

        # Swarming speed saturation
        speedNorm = math.sqrt(velEast**2 + velNorth**2)
        if speedNorm > self.MAX_SWARMING_SPEED:
            velEast *= (self.MAX_SWARMING_SPEED / speedNorm)
            velNorth *= (self.MAX_SWARMING_SPEED / speedNorm)

        # Vertical speed
        velUp = (-self.H_DES - self.localPosition.z) * self.K_H

        # Vertical speed saturation
        if abs(velUp) > self.MAX_TAKEOFF_SPEED:
            velUp *= (self.MAX_TAKEOFF_SPEED / abs(velUp))

        # Tracking velocity
        if self.trackingVelocityReceived:
            velEast += self.trackingVelocity.east
            velNorth += self.trackingVelocity.north

        self.setTrajectorySetpoint(vx=velNorth, vy=velEast, vz=velUp)

    def returnHome(self):
        """
        If the takeoff was already completed, it returns home, otherwise it hovers at the current position.
        """

        if (self.home[i] is not None for i in range(3)):
            self.setTrajectorySetpoint(x=self.home[0], y=self.home[1], z=self.home[2])
        else:
            self.get_status().warn("Home is not set yet")
            self.droneMode = "hover"

    def land(self):
        """
        It descends to an altitude of 1 meter at a constant speed (1 m/s), then the automatic landing procedure begins,
        and the drone is automatically disarmed by the autopilot when it detects the completed landing.
        """

        if self.localPosition.z >= -1.0:
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.setTrajectorySetpoint()
            self.readyForIdle = True
        else:
            self.setTrajectorySetpoint(vz=1.0)

    def restart(self):
        """
        All the variables, publishers, subscribers, and services are reinitialized.
        """

        self.initializeDrone()
    # endregion

    # region Callbacks
    def timesyncCallback(self, msg):
        """
        @param msg: message

        The timestamp is updated constantly.
        """

        self.timestamp = msg.timestamp

    def timerCallback(self):
        """
        @return: None

        This callback runs at a frequency defined by the timer previously initialized. First it makes sure that ROS2 is
        connected to PX4, and saves the number of drones in the swarm N as sent by the ground station. Then it checks
        whether any drone is not sending its GNSS position on the network. It sets the desired mode in such a way that
        the correct method is called, and, unless it is in idle mode, it publishes the trajectory setpoint.
        """

        self.loggerCounter += 1

        # If ROS2 is not properly connected to PX4
        if self.timestamp == 0:
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Not synced")
                self.publishDroneInfo("Not synced")
            return

        # If the number of drones in the swarm is not known yet
        if self.N is None:
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Waiting for number of anchors")
                self.publishDroneInfo("Waiting for number of anchors")
            return

        # TODO: con questo check non funziona più il test con GPSSimulator.py
        missingGNSSPositionInfo = False
        for key in self.lastPositionReceivedTimer.keys():
            if self.lastPositionReceivedTimer[key] is None:
                missingGNSSPositionInfo = True
                if self.loggerCounter % self.RATE == 0:
                    self.get_logger().warn("No GNSS data from drone %d" % key)
                    self.publishDroneInfo("No GNSS data from drone " + str(key))
            else:
                self.lastPositionReceivedTimer[key] += 1
        if missingGNSSPositionInfo:
            return

        if self.UWB_ON:
            for key in self.lastUWBDataReceivedTimer.keys():
                self.lastUWBDataReceivedTimer[key] += 1

        if self.readyForIdle:
            self.publishOffboardControlMode()
            self.droneMode = "idle"
            self.readyForIdle = False

        if self.NOT_FLYING_FOR_TEST:
            self.droneNotFlyingOperations()
        else:
            self.droneOperations.get(self.droneMode, self.idle)()

        if self.droneMode != self.droneModeOld:
            self.get_logger().info("Setting %s mode" % self.droneMode)
            self.publishDroneInfo("Setting " + self.droneMode + " mode")

        self.droneModeOld = self.droneMode

        if self.droneMode != "idle" and not self.nodeDestroyed:
            self.publishTrajectorySetpoint()

    def localPositionCallback(self, msg):
        """
        @param msg: message

        It updates the local position of the drone when new data are received.
        """

        self.localPosition = msg

    def uwbSensorCallback(self, msg):
        """
        @param msg: message
        @return: None

        It updates the distance from the other UWB sensors in the system when new data are received.
        """

        if int(msg.anchor_pose.header.frame_id) in self.activeDrones.keys():
            self.uwbDistances[int(msg.anchor_pose.header.frame_id)] = msg.range
            self.lastUWBDataReceivedTimer[int(msg.anchor_pose.header.frame_id)] = 0
            if self.N is None:
                return
            if not self.uwbDistancesReceived and \
                    (len(self.uwbDistances) == self.N - 1 and self.TARGET_ID not in self.uwbDistances.keys() or \
                     (len(self.uwbDistances) == self.N and self.TARGET_ID in self.uwbDistances.keys())):
                self.uwbDistancesReceived = True

    def trackingVelocityCallback(self, msg):
        """
        @param msg: message

        It updates the tracking velocity command received by the ground station.
        """

        if not self.trackingVelocityReceived:
            self.trackingVelocityReceived = True

        self.trackingVelocity = msg

    def vehicleStatusCallback(self, msg):
        """
        @param msg: message

        It updates the status of the drone and, if a pilot took control via RC, stops to send it any command switching
        to idle mode.
        """

        previousNavState = self.vehicleStatus.nav_state
        self.vehicleStatus = msg

        if self.authorizedForOffboard and self.vehicleStatus.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                self.vehicleStatus.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and previousNavState == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        # if self.authorizedForOffboard and (self.vehicleStatus.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_RC_STICK or \
        #                                    self.vehicleStatus.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_RC_SWITCH):
            self.authorizedForOffboard = False
            if self.vehicleStatus.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.publishDroneInfo("RC took control")
            self.droneMode = "idle"

    def anchorReadyForTakeoffCallback(self, msg):
        """
        @param msg: message

        It updates the data structure keeping memory of the drones that are ready for takeoff.
        """

        self.anchorsReadyForTakeoff[msg.data] = True

    def anchorReadyForSwarmingCallback(self, msg):
        """
        @param msg: message

        It updates the data structure keeping memory of the drones that are ready for swarming.
        """

        self.anchorsReadyForSwarming[msg.data] = True

    def anchorsPositionCallback(self, msg, droneId):
        """
        @param msg: message
        @param droneId: drone id
        @return: None

        It updates the position of the other drones of the formation, and records that the time passed since the last
        data received is zero.
        """

        self.anchorsPosition[droneId] = msg
        self.lastPositionReceivedTimer[droneId] = 0
        if self.N is None:
            return
        if not self.anchorsPositionReceived and len(self.anchorsPosition) == self.N:
            self.anchorsPositionReceived = True

    def numAnchorsCallback(self, msg):
        """
        @param msg: message

        It initializes the number of drones in the formation "self.N", and the subscribers relative to the position of
        the drones. This callback is run only once, because at the end of its execution the subscription
        "self.numAnchorsSub" is destroyed.
        """

        self.N = msg.data
        self.NAtStart = self.N

        for i in range(self.N):
            self.activeDrones[i] = True
            self.lastPositionReceivedTimer[i] = None
            # self.anchorsPositionSubs[i] = self.create_subscription(VehicleGlobalPosition, "/X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.QUEUE_SIZE)
            self.anchorsPositionSubs[i] = self.create_subscription(VehicleGlobalPosition, "/X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), qos_profile=self.qosProfile)

        self.destroy_subscription(self.numAnchorsSub)

    def droneCommandCallback(self, request, response):
        """
        @param request: service request
        @param response: service response
        @return: response

        The field "request.operation" describes the mode to be set, and the other fields (defined in
        "DroneCustomCommand.srv") are used to update the various setpoints. Then, the response is returned. In case the
        operation field is not recognized, default values are used and hover mode is set to avoid problems.
        """

        if request.operation in self.droneOperations.keys():
            self.droneMode = request.operation
            response.result = "success"

            self.lastGoalPositionReceived.x = request.x
            self.lastGoalPositionReceived.y = request.y
            if request.z == 0:
                self.lastGoalPositionReceived.z = -self.H_DES
            else:
                self.lastGoalPositionReceived.z = request.z
            self.lastGoalVelocityReceived.linear.x = request.vx
            self.lastGoalVelocityReceived.linear.y = request.vy
            self.lastGoalVelocityReceived.linear.z = request.vz
            self.lastGoalVelocityReceived.angular.z = request.wz
        else:
            self.get_logger().warn("Invalid command")
            self.publishDroneInfo("Invalid command")
            self.droneMode = "hover"
            response.result = "failure"

            self.lastGoalPositionReceived.x = 0.0
            self.lastGoalPositionReceived.y = 0.0
            self.lastGoalPositionReceived.z = -self.H_DES
            self.lastGoalVelocityReceived.linear.x = 0.0
            self.lastGoalVelocityReceived.linear.y = 0.0
            self.lastGoalVelocityReceived.linear.z = 0.0
            self.lastGoalVelocityReceived.angular.z = 0.0

        return response
    # endregion


def main():
    rclpy.init(args=None)
    node = Drone()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
