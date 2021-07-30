#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node
from px4_msgs.msg import Timesync, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, UInt64
from ros2_px4_interfaces.msg import UnitVector, UwbSensor, VelocityVector
from ros2_px4_interfaces.srv import DroneCustomCommand
from ros2_px4_functions import swarming_functions


UWB_ON = True


class Drone(Node):
    def __init__(self):
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
                ('ID', None)                            # drone ID
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
        self.ID = self.get_parameter('ID').value
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
        if UWB_ON:
            self.uwbDistancesReceived = None            # bool                              has the drone received UWB data from every swarm member?
            self.lastUWBDataReceivedTimer = None        # dict(int: int)                    time from the last received UWB data from every swarm member
            self.uwbDistances = None                    # dict(int: float)                  inter-drones distances from UWB sensor
        # endregion

        # region Subscribers, publishers and services declaration
        # Subscribers
        self.timesyncSub = None                         # subscription to timesync
        self.numAnchorsSub = None                       # subscription to get number of drones in the swarm
        self.localPositionSub = None                    # subscription to current drone's local position
        self.vehicleStatusSub = None                    # subscription to current drone's status
        self.readyForTakeoffSubs = None                 # subscriptions to get which member of the swarm is ready for takeoff
        self.readyForSwarmingSubs = None                # subscriptions to get which member of the swarm is ready for swarming
        self.anchorsPositionSubs = None                 # subscriptions to swarm members' GPS position
        self.trackingVelocitySub = None                 # subscription to tracking velocity centralized controller
        if UWB_ON:
            self.uwbSensorSub = None                    # subscription to UWB sensor's data

        # Publishers
        self.vehicleCommandPub = None                   # publisher to send vehicle commands
        self.offboardControlModePub = None              # publisher to send offboard control
        self.readyForTakeoffPub = None                  # publisher to communicate when the current drone is ready for takeoff
        self.readyForSwarmingPub = None                 # publisher to communicate when the current drone is ready for swarming
        self.trajectorySetpointPub = None               # publisher to send trajectory setpoint

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
        if UWB_ON:
            self.uwbDistancesReceived = False
            self.lastUWBDataReceivedTimer = {}
            self.uwbDistances = {}

        # Subscribers initialization
        self.timesyncSub = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesyncCallback, self.QUEUE_SIZE)
        self.numAnchorsSub = self.create_subscription(UInt64, "numAnchorsNode/N", self.numAnchorsCallback, self.QUEUE_SIZE)
        self.localPositionSub = self.create_subscription(VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.localPositionCallback, self.QUEUE_SIZE)
        self.vehicleStatusSub = self.create_subscription(VehicleStatus, "VehicleStatus_PubSubTopic", self.vehicleStatusCallback, self.QUEUE_SIZE)
        self.readyForTakeoffSubs = {}
        self.readyForSwarmingSubs = {}
        self.anchorsPositionSubs = {}
        self.trackingVelocitySub = self.create_subscription(VelocityVector, "trackingVelocityCalculator/trackingVelocity", self.trackingVelocityCallback, self.QUEUE_SIZE)
        if UWB_ON:
            self.uwbSensorSub = self.create_subscription(UwbSensor, "uwb_sensor_" + str(self.ID), self.uwbSensorCallback, self.QUEUE_SIZE)

        # Publishers initialization
        self.vehicleCommandPub = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", self.QUEUE_SIZE)
        self.offboardControlModePub = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", self.QUEUE_SIZE)
        self.readyForTakeoffPub = self.create_publisher(Bool, "readyForTakeoff", self.QUEUE_SIZE)
        self.readyForSwarmingPub = self.create_publisher(Bool, "readyForSwarming", self.QUEUE_SIZE)
        self.trajectorySetpointPub = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", self.QUEUE_SIZE)

        # Services initialization
        self.droneCommandSrv = self.create_service(DroneCustomCommand, "DroneCustomCommand", self.droneCommandCallback)

    def publishVehicleCommand(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.ID + 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicleCommandPub.publish(msg)

    def publishOffboardControlMode(self):
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
        msg = self.trajectorySetpoint
        msg.timestamp = self.timestamp

        self.publishOffboardControlMode()
        self.trajectorySetpointPub.publish(msg)

    def computeInterDronesDistances(self):
        for j in self.anchorsPosition.keys():
            if j != self.ID:
                a = (math.sin(math.radians(self.anchorsPosition[j].lat - self.anchorsPosition[self.ID].lat) / 2))**2 +\
                    math.cos(math.radians(self.anchorsPosition[self.ID].lat)) * math.cos(math.radians(self.anchorsPosition[j].lat)) * (math.sin(math.radians(self.anchorsPosition[j].lon - self.anchorsPosition[self.ID].lon) / 2))**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
                self.interDronesDistances[j] = self.EARTH_RADIUS * c

    def armVehicle(self):
        if self.authorizedForOffboard:
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            self.setTrajectorySetpoint()
            self.armCounter += 1
        else:
            self.get_logger().warn("Vehicle not authorized for offboard")
            self.droneMode = "idle"
    # endregion

    # region Services' handling methods
    def idle(self):
        return

    def takeoff(self):
        if not self.readyForTakeoff:
            # Communicate to the other anchors that I am ready to takeoff
            self.readyForTakeoff = True
            self.anchorsReadyForTakeoff[self.ID] = True
            msg = Bool()
            msg.data = True
            self.readyForTakeoffPub.publish(msg)

        if len(self.anchorsReadyForTakeoff) < self.N:
            # The other anchors are not ready to takeoff yet
            msg = Bool()
            msg.data = True
            self.readyForTakeoffPub.publish(msg)
            return

        # # Check if the commanded altitude is high enough
        # if self.lastGoalPositionReceived.z > -3.0:
        #     self.lastGoalPositionReceived.z = -3.0

        # Arm the drone (if necessary)
        if self.vehicleStatus.arming_state != VehicleStatus.ARMING_STATE_ARMED or self.armCounter < 2:
            self.armVehicle()

        if self.localPosition.z <= self.lastGoalPositionReceived.z:
            # Takeoff completed
            self.droneMode = "swarming"
            # self.droneMode = "hover"
            self.home = [self.localPosition.x, self.localPosition.y, self.localPosition.z]
            return

        # Compute the takeoff speed and saturate it to a maximum value
        takeoffSpeed = self.lastGoalPositionReceived.z - self.localPosition.z
        if abs(takeoffSpeed) > self.MAX_TAKEOFF_SPEED:
            takeoffSpeed *= (self.MAX_TAKEOFF_SPEED / abs(takeoffSpeed))
        self.setTrajectorySetpoint(vz=takeoffSpeed)

    def hover(self):
        self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)

    def goTo(self):
        self.setTrajectorySetpoint(x=self.lastGoalPositionReceived.x, y=self.lastGoalPositionReceived.y, z=self.lastGoalPositionReceived.z)

    def setHorizontalVelocity(self):
        self.setTrajectorySetpoint(vx=self.lastGoalVelocityReceived.linear.x, vy=self.lastGoalVelocityReceived.linear.y)

    def swarming(self):
        if (UWB_ON and (not self.anchorsPositionReceived or not self.uwbDistancesReceived)) or (not UWB_ON and not self.anchorsPositionReceived):
            # Complete info from the other anchors has not arrived yet
            self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)
            return
        elif not self.readyForSwarming:
            # Communicate to the other anchors that I am ready to start swarming
            self.readyForSwarming = True
            self.anchorsReadyForSwarming[self.ID] = True
            msg = Bool()
            msg.data = True
            self.readyForSwarmingPub.publish(msg)

        if len(self.anchorsReadyForSwarming) < self.N:
            # The other anchors are not ready to swarm yet
            msg = Bool()
            msg.data = True
            self.readyForSwarmingPub.publish(msg)
            self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)
            return

        # Check if all of the drones are still active
        keysToDelete = []
        for key in self.lastPositionReceivedTimer.keys():
            if self.lastPositionReceivedTimer[key] >= self.TIMEOUT * self.RATE:
                self.N -= 1
                keysToDelete.append(key)
                if UWB_ON:
                    for dataStructure in [self.anchorsReadyForTakeoff, self.anchorsReadyForSwarming, self.anchorsPosition, self.unitVectors, self.uwbDistances]:
                        assert type(dataStructure) is dict, self.get_logger().error("Something wrong with the data structures")
                        if key in dataStructure.keys():
                            dataStructure.pop(key)
                else:
                    for dataStructure in [self.anchorsReadyForTakeoff, self.anchorsReadyForSwarming, self.anchorsPosition, self.unitVectors, self.interDronesDistances]:
                        assert type(dataStructure) is dict, self.get_logger().error("Something wrong with the data structures")
                        if key in dataStructure.keys():
                            dataStructure.pop(key)
                for subscriber in [self.readyForTakeoffSubs, self.readyForSwarmingSubs, self.anchorsPositionSubs]:
                    assert type(subscriber) is dict, self.get_logger().error("Something wrong with the subscribers")
                    if key in subscriber.keys():
                        subscriber[key].destroy()
                        subscriber.pop(key)
        for key in keysToDelete:
            if key in self.activeDrones.keys():
                self.activeDrones.pop(key)
            if key in self.lastPositionReceivedTimer.keys():
                self.lastPositionReceivedTimer.pop(key)
            if UWB_ON:
                if key in self.lastUWBDataReceivedTimer.keys():
                    self.lastUWBDataReceivedTimer.pop(key)

        if self.ID not in self.anchorsPosition.keys():
            self.get_logger().info("Drone %d is no longer part of the swarm" % self.ID)
            self.nodeDestroyed = True
            self.destroy_node()
            return

        # Compute the distances between the drones from their GPS positions
        if not UWB_ON and self.anchorsPositionReceived:
            self.computeInterDronesDistances()

        # Compute the unit vectors for the swarming algorithm
        for key in self.anchorsPosition.keys():
            if key != self.ID:
                self.unitVectors[key] = swarming_functions.computeUnitVector(self.ID, self.anchorsPosition[self.ID], key, self.anchorsPosition[key], self.get_clock().now().to_msg())

        velEast = 0.0
        velNorth = 0.0
        # Swarming velocity
        if UWB_ON:
            for key in self.uwbDistances.keys():
                if key != self.TARGET_ID:
                    componentEast = self.uwbDistances[key] * self.unitVectors[key].east * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                    componentNorth = self.uwbDistances[key] * self.unitVectors[key].north * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                    velEast -= componentEast
                    velNorth -= componentNorth
        else:
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
        if (self.home[i] is not None for i in range(3)):
            self.setTrajectorySetpoint(x=self.home[0], y=self.home[1], z=self.home[2])
        else:
            self.get_status().warn("Home is not set yet")
            self.droneMode = "hover"

    def land(self):
        if self.localPosition.z >= -1.0:
            self.publishVehicleCommand(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.setTrajectorySetpoint()
            self.readyForIdle = True
        else:
            self.setTrajectorySetpoint(vz=0.5)

    def restart(self):
        self.initializeDrone()
    # endregion

    # region Callbacks
    def timesyncCallback(self, msg):
        self.timestamp = msg.timestamp

    def timerCallback(self):
        self.loggerCounter += 1

        # If ROS2 is not properly connected to PX4
        if self.timestamp == 0:
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Not synced")
            return

        # If the number of drones in the swarm is not known yet
        if self.N is None:
            if self.loggerCounter % self.RATE == 0:
                self.get_logger().warn("Number of anchors unknown")
            return

        for key in self.lastPositionReceivedTimer.keys():
            self.lastPositionReceivedTimer[key] += 1
        if UWB_ON:
            for key in self.lastUWBDataReceivedTimer.keys():
                self.lastUWBDataReceivedTimer[key] += 1

        if self.readyForIdle:
            self.publishOffboardControlMode()
            self.droneMode = "idle"
            self.readyForIdle = False

        self.droneOperations.get(self.droneMode, self.idle)()

        if self.droneMode != self.droneModeOld:
            self.get_logger().info("Setting %s mode" % self.droneMode)

        self.droneModeOld = self.droneMode

        if self.droneMode != "idle" and not self.nodeDestroyed:
            self.publishTrajectorySetpoint()

    def localPositionCallback(self, msg):
        self.localPosition = msg

    def uwbSensorCallback(self, msg):
        if int(msg.anchor_pose.header.frame_id) in self.activeDrones.keys():
            self.uwbDistances[int(msg.anchor_pose.header.frame_id)] = msg.range
            self.lastUWBDataReceivedTimer[int(msg.anchor_pose.header.frame_id)] = 0
            if self.N is None:
                return
            if not self.uwbDistancesReceived and \
                    (len(self.uwbDistances) == self.N - 1 and self.TARGET_ID not in self.uwbDistances.keys() or \
                     (len(self.uwbDistances) == self.N and self.TARGET_ID in self.uwbDistances.keys())):
                self.uwbDistancesReceived = True
            # TODO: implementare la callback per sincronizzare le distanze

    def trackingVelocityCallback(self, msg):
        if not self.trackingVelocityReceived:
            self.trackingVelocityReceived = True

        self.trackingVelocity = msg

    def vehicleStatusCallback(self, msg):
        previousNavState = self.vehicleStatus.nav_state
        self.vehicleStatus = msg

        if self.authorizedForOffboard and self.vehicleStatus.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                self.vehicleStatus.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and previousNavState == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # (self.vehicleStatus.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_RC_STICK or
            #  self.vehicleStatus.latest_disarming_reason == VehicleStatus.ARM_DISARM_REASON_RC_SWITCH):
            self.authorizedForOffboard = False
            self.droneMode = "idle"

    def anchorReadyForTakeoffCallback(self, msg, droneId):
        self.anchorsReadyForTakeoff[droneId] = msg.data

    def anchorReadyForSwarmingCallback(self, msg, droneId):
        self.anchorsReadyForSwarming[droneId] = msg.data

    def anchorsPositionCallback(self, msg, droneId):
        self.anchorsPosition[droneId] = msg
        self.lastPositionReceivedTimer[droneId] = 0
        if self.N is None:
            return
        if not self.anchorsPositionReceived and len(self.anchorsPosition) == self.N:
            self.anchorsPositionReceived = True

    def numAnchorsCallback(self, msg):
        self.N = msg.data

        for i in range(self.N):
            self.activeDrones[i] = True
            self.lastPositionReceivedTimer[i] = 0
            self.anchorsPositionSubs[i] = self.create_subscription(VehicleGlobalPosition, "X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.QUEUE_SIZE)
            if i != self.ID:
                self.readyForTakeoffSubs[i] = self.create_subscription(Bool, "X500_" + str(i) + "/readyForTakeoff", partial(self.anchorReadyForTakeoffCallback, droneId=i), self.QUEUE_SIZE)
                self.readyForSwarmingSubs[i] = self.create_subscription(Bool, "X500_" + str(i) + "/readyForSwarming", partial(self.anchorReadyForSwarmingCallback, droneId=i), self.QUEUE_SIZE)

        self.destroy_subscription(self.numAnchorsSub)

    def droneCommandCallback(self, request, response):
        if request.operation in self.droneOperations.keys():
            self.droneMode = request.operation
            response.result = "success"

            self.lastGoalPositionReceived.x = request.x
            self.lastGoalPositionReceived.y = request.y
            self.lastGoalPositionReceived.z = request.z
            self.lastGoalVelocityReceived.linear.x = request.vx
            self.lastGoalVelocityReceived.linear.y = request.vy
            self.lastGoalVelocityReceived.linear.z = request.vz
            self.lastGoalVelocityReceived.angular.z = request.wz
        else:
            self.get_logger().warn("Invalid command")
            self.droneMode = "hover"
            response.result = "failure"

            self.lastGoalPositionReceived.x = 0.0
            self.lastGoalPositionReceived.y = 0.0
            self.lastGoalPositionReceived.z = -3.0
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
