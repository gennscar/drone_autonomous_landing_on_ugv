#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node
from px4_msgs.msg import Timesync, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from ros2_px4_interfaces.msg import UnitVector, UnitVectorArray, UwbSensor, VelocityVector
from ros2_px4_interfaces.srv import DroneCustomCommand

# TODO: cambiare l'altezza con il valore dato dal sensore laser


class Drone(Node):
    def __init__(self):
        super().__init__("anchorDrone")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('MAX_TAKEOFF_SPEED', None),
                ('A', None),
                ('B', None),
                ('C', None),
                ('H_DES', None),
                ('K_H', None),
                ('MAX_SWARMING_SPEED', None),
                ('ID', None),
                ('N', None),
                ('NUM_TARGET', None)
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
        self.MAX_SWARMING_SPEED = self.get_parameter('MAX_SWARMING_SPEED').value
        self.ID = self.get_parameter('ID').value
        self.N = self.get_parameter('N').value
        self.NUM_TARGET = self.get_parameter('NUM_TARGET').value
        # endregion

        # Drone operations
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

        # Useful variables
        self.timestamp = 0
        self.timerCounter = 0
        self.offboardSetpointCounter = 0
        self.droneMode = "disarmed"
        self.droneModeOld = "disarmed"
        self.uwbDistancesReceived = False
        self.unitVectorsReceived = False
        self.trackingVelocityReceived = False
        self.localPosition = VehicleLocalPosition()
        self.trajectorySetpoint = TrajectorySetpoint()
        self.lastGoalPositionReceived = Point()
        self.lastGoalVelocityReceived = Twist()
        self.uwbDistances = {}
        self.unitVectors = {}
        self.trackingVelocity = None
        self.readyForTakeoff = False
        self.anchorsReadyForTakeoff = {}
        self.readyForSwarming = False
        self.anchorsReadyForSwarming = {}
        self.home = [None, None, None]
        self.vehicleStatus = VehicleStatus()
        self.vehicleArmed = False

        # Subscribers initialization
        self.timesyncSub = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesyncCallback, self.QUEUE_SIZE)
        self.localPositionSub = self.create_subscription(VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.localPositionCallback, self.QUEUE_SIZE)
        self.uwbSensorSub = self.create_subscription(UwbSensor, "uwb_sensor_" + str(self.ID), self.uwbSensorCallback, self.QUEUE_SIZE)
        self.unitVectorsSub = self.create_subscription(UnitVectorArray, "unitVectorsCalculator/unitVectors", self.unitVectorsCallback, self.QUEUE_SIZE)
        self.trackingVelocitySub = self.create_subscription(VelocityVector, "trackingVelocityCalculator/trackingVelocity", self.trackingVelocityCallback, self.QUEUE_SIZE)
        self.vehicleStatusSub = self.create_subscription(VehicleStatus, "VehicleStatus_PubSubTopic", self.vehicleStatusCallback, self.QUEUE_SIZE)
        for i in range(self.N):
            if i != self.ID:
                self.create_subscription(Bool, "X500_" + str(i) + "/readyForTakeoff", partial(self.anchorReadyForTakeoffCallback, droneId=i), self.QUEUE_SIZE)
                self.create_subscription(Bool, "X500_" + str(i) + "/readyForSwarming", partial(self.anchorReadyForSwarmingCallback, droneId=i), self.QUEUE_SIZE)

        # Publishers initialization
        self.vehicleCommandPub = self.create_publisher(VehicleCommand, "VehicleCommand_PubSubTopic", self.QUEUE_SIZE)
        self.offboardControlModePub = self.create_publisher(OffboardControlMode, "OffboardControlMode_PubSubTopic", self.QUEUE_SIZE)
        self.trajectorySetpointPub = self.create_publisher(TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", self.QUEUE_SIZE)
        self.readyForTakeoffPub = self.create_publisher(Bool, "readyForTakeoff", self.QUEUE_SIZE)
        self.readyForSwarmingPub = self.create_publisher(Bool, "readyForSwarming", self.QUEUE_SIZE)

        # Services initialization
        self.droneCommandSrv = self.create_service(DroneCustomCommand, "DroneCustomCommand", self.droneCommandCallback)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

        # Publish 100 trajectory setpoints
        for i in range(100):
            self.publishTrajectorySetpoint()

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
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True

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

    def setOffboardMode(self):
        self.publishOffboardControlMode()
        self.get_logger().info("Setting offboard mode")

    def idle(self):
        return

    def armVehicle(self):
        if self.vehicleStatus.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            return
        elif self.vehicleStatus._latest_disarming_reason != VehicleStatus.ARM_DISARM_REASON_RC_STICK and \
                self.vehicleStatus._latest_disarming_reason != VehicleStatus.ARM_DISARM_REASON_RC_SWITCH:
            self.setOffboardMode()
            self.publishVehicleCommand(176, 1.0, 6.0)
            self.publishVehicleCommand(400, 1.0, 0.0)
        else:
            self.get_logger().warn("Disarmed by RC, impossible to arm")
            self.droneMode = "idle"

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

        if self.lastGoalPositionReceived.z > -3.0:
            self.lastGoalPositionReceived.z = -3.0

        self.armVehicle()

        if self.localPosition.z <= self.lastGoalPositionReceived.z:
            self.droneMode = "swarming"
            self.home = [self.localPosition.x, self.localPosition.y, self.localPosition.z]
            return

        takeoffSpeed = self.lastGoalPositionReceived.z - self.localPosition.z
        if self.trajectorySetpoint.vz < -self.MAX_TAKEOFF_SPEED:
            takeoffSpeed = -self.MAX_TAKEOFF_SPEED
        elif self.trajectorySetpoint.vz > self.MAX_TAKEOFF_SPEED:
            takeoffSpeed = self.MAX_TAKEOFF_SPEED
        self.setTrajectorySetpoint(vz=takeoffSpeed)

    def hover(self):
        self.setTrajectorySetpoint(vx=0.0, vy=0.0, vz=0.0)

    def goTo(self):
        self.setTrajectorySetpoint(x=self.lastGoalPositionReceived.x, y=self.lastGoalPositionReceived.y, z=self.lastGoalPositionReceived.z)

    def setHorizontalVelocity(self):
        self.setTrajectorySetpoint(vx=self.lastGoalVelocityReceived.linear.x, vy=self.lastGoalVelocityReceived.linear.y)

    def swarming(self):
        if not self.unitVectorsReceived or not self.uwbDistancesReceived:
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

        velEast = 0.0
        velNorth = 0.0
        # Swarming velocity
        for key in range(self.N):
            if key != self.ID:
                componentEast = self.uwbDistances[key] * self.unitVectors[key].east * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                componentNorth = self.uwbDistances[key] * self.unitVectors[key].north * (self.A - self.B * math.exp(-self.uwbDistances[key]**2 / self.C))
                velEast -= componentEast
                velNorth -= componentNorth

        # Swarming speed saturation
        speedNorm = math.sqrt(velEast**2 + velNorth**2)
        if speedNorm > self.MAX_SWARMING_SPEED:
            velEast *= (self.MAX_SWARMING_SPEED / speedNorm)
            velNorth *= (self.MAX_SWARMING_SPEED / speedNorm)

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

    def prepareForLanding(self):
        if self.localPosition.z >= -1.0:
            self.droneMode = "land"
            self.land()
            return

        self.setTrajectorySetpoint(vz=0.5)

    def land(self):
        self.publishVehicleCommand(21, 0.0, 0.0)

    def restart(self):
        self.timerCounter = 0

    # region Callbacks
    def timesyncCallback(self, msg):
        self.timestamp = msg.timestamp

    def timerCallback(self):
        # If ROS2 is not properly connected to PX4
        if self.timestamp == 0:
            self.get_logger().warn("Not synced")
            return

        if self.timerCounter == 0:
            # self.setOffboardMode()
            self.droneMode = "idle"

        self.droneOperations.get(self.droneMode, self.idle)()

        if self.droneMode != self.droneModeOld:
            self.get_logger().info("Setting %s mode" % self.droneMode)

        self.droneModeOld = self.droneMode

        self.publishTrajectorySetpoint()

        if self.timerCounter == 0:
            self.timerCounter = 1

    def localPositionCallback(self, msg):
        self.localPosition = msg

    def uwbSensorCallback(self, msg):
        self.uwbDistances[int(msg.anchor_pose.header.frame_id)] = msg.range
        if not self.uwbDistancesReceived and len(self.uwbDistances) == self.N - 1:
            self.uwbDistancesReceived = True
        # TODO: implementare la callback per sincronizzare le distanze
        # self.get_logger().info(msg)
        # for dis in self.uwbDistances.values():
        #     self.get_logger().info(str(dis))

    def unitVectorsCallback(self, msg):
        if not self.unitVectorsReceived:
            self.unitVectorsReceived = True

        for uv in msg.data:
            if uv.destination_drone_id == self.ID:
                self.unitVectors[uv.start_drone_id] = uv
            elif uv.start_drone_id == self.ID:
                self.unitVectors[uv.destination_drone_id] = uv
                self.unitVectors[uv.destination_drone_id].east *= -1
                self.unitVectors[uv.destination_drone_id].north *= -1

    def trackingVelocityCallback(self, msg):
        if not self.trackingVelocityReceived:
            self.trackingVelocityReceived = True

        self.trackingVelocity = msg

    def vehicleStatusCallback(self, msg):
        self.vehicleStatus = msg

    def anchorReadyForTakeoffCallback(self, msg, droneId):
        self.anchorsReadyForTakeoff[droneId] = msg.data

    def anchorReadyForSwarmingCallback(self, msg, droneId):
        self.anchorsReadyForSwarming[droneId] = msg.data

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
