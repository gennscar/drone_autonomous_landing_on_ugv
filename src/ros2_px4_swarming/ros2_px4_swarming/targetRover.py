#!/usr/bin/env python3
import math
from random import random, uniform
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry
from ros2_px4_interfaces.srv import TargetCustomCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def computeDistance(currentPosition, goalPosition):
    """
    @param currentPosition: position A
    @param goalPosition: position B
    @return: distance

    This function computes the horizontal Euclidean distance between two points A and B.
    """

    return math.sqrt((currentPosition.x - goalPosition.x)**2 + (currentPosition.y - goalPosition.y)**2)


class Target(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml". It defines a data structure
        including all the rover's functionalities. It declares and initializes useful variables, publishers,
        subscribers, and services. It creates a timer to regulate the frequency of the controller.
        """

        super().__init__("target")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('BEST_EFFORT', None),
                ('MAX_THROTTLE', None),
                ('MAX_ROTATION', None),
                ('KT', None),
                ('KR', None),
                ('DISTANCE_ERROR', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        self.MAX_THROTTLE = self.get_parameter('MAX_THROTTLE').value
        self.MAX_ROTATION = self.get_parameter('MAX_ROTATION').value
        self.KT = self.get_parameter('KT').value
        self.KR = self.get_parameter('KR').value
        self.DISTANCE_ERROR = self.get_parameter('DISTANCE_ERROR').value
        # endregion

        # Target operations
        self.targetOperations = {
            "hold": self.hold,
            "straight": self.straight,
            "circle": self.circle,
            "random": self.random,
            "goTo": self.goTo,
            "rotate": self.rotate,
            "turn": self.turn,
            "square": self.square
        }

        # Useful variables
        self.timerCounter = 0
        self.velocitySetpoint = Twist()
        self.odom = Odometry()
        self.lastGoalPositionReceived = Point()
        self.lastGoalVelocityReceived = Twist()
        self.lastGoalTurnReceived = Vector3()
        self.startingPosition = Point()
        self.beginTurn = False
        self.targetMode = "hold"
        self.targetModeOld = "hold"
        self.lastThrottle = 0.0
        self.lastRotation = 0.0
        self.randomMotionCounter = 1000000
        self.qosProfile = None

        # TODO: questo nodo non funziona se i topic sono in modalità best effort, perché?
        self.BEST_EFFORT = False

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
        self.odomSub = self.create_subscription(Odometry, "odom", self.odomCallback, self.qosProfile)

        # Publishers initialization
        self.velocitySetpointPub = self.create_publisher(Twist, "cmd_vel", self.qosProfile)

        # Services initialization
        self.targetCommandSrv = self.create_service(TargetCustomCommand, "targetCustomCommand", self.targetCommandCallback)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

        # Publish 100 trajectory setpoints
        for i in range(100):
            self.publishVelocitySetpoint()

    # region Auxiliary methods
    def setVelocitySetpoint(self, vx=float("NaN"), yawrate=float("NaN")):
        """
        @param vx: desired velocity
        @param yawrate: desired yaw rate

        It sets velocity and yaw rate to the desired values.
        """

        self.velocitySetpoint.linear.x = float(vx)
        self.velocitySetpoint.angular.z = float(yawrate)

    def publishVelocitySetpoint(self):
        """
        It publishes the velocity setpoint on the correct topic.
        """

        self.velocitySetpointPub.publish(self.velocitySetpoint)

    def computeCurrentYaw(self):
        """
        @return: current orientation

        It converts the orientation data into an angle, and it returns it.
        """

        return math.atan2(2 * (self.odom.pose.pose.orientation.w * self.odom.pose.pose.orientation.z + self.odom.pose.pose.orientation.x * self.odom.pose.pose.orientation.y),
                          1 - 2 * (self.odom.pose.pose.orientation.y**2 + self.odom.pose.pose.orientation.z**2))

    def positionControl(self):
        """
        @return: throttle and rotation

        According to the current position of the target, and the desired destination, this controller defines in a very
        simplified way the throttle and rotation needed at every iteration to correctly reach the setpoint.
        """

        distance = computeDistance(self.odom.pose.pose.position, self.lastGoalPositionReceived)
        if distance <= self.DISTANCE_ERROR:
            self.targetMode = "hold"
            return [0, 0]

        desiredAngle = math.atan2(self.lastGoalPositionReceived.y - self.odom.pose.pose.position.y,
                                  self.lastGoalPositionReceived.x - self.odom.pose.pose.position.x)
        if desiredAngle < -math.pi:
            desiredAngle += 2 * math.pi
        elif desiredAngle > math.pi:
            desiredAngle -= 2 * math.pi

        currentYaw = self.computeCurrentYaw()
        rotation = self.KR * (desiredAngle - currentYaw)
        if rotation < -math.pi:
            rotation += 2 * math.pi
        elif rotation > math.pi:
            rotation -= 2 * math.pi

        if rotation > self.MAX_ROTATION:
            rotation = self.MAX_ROTATION
        elif rotation < -self.MAX_ROTATION:
            rotation = -self.MAX_ROTATION

        throttle = self.KT * distance
        if throttle > self.MAX_THROTTLE:
            throttle = self.MAX_THROTTLE

        if abs(rotation) >= math.pi / 12:
            throttle /= 3

        return [throttle, rotation]
    # endregion

    # region Services' handling methods
    def hold(self):
        """
        It sets the velocity setpoint to (0, 0).
        """

        self.setVelocitySetpoint(vx=0, yawrate=0)

    def straight(self):
        """
        It sets the velocity setpoint to (vx, 0).
        """

        self.setVelocitySetpoint(vx=self.lastGoalVelocityReceived.linear.x, yawrate=0)

    def circle(self):
        """
        It sets the velocity setpoint to (vx, wz).
        """

        self.setVelocitySetpoint(vx=self.lastGoalVelocityReceived.linear.x, yawrate=self.lastGoalVelocityReceived.angular.z)

    def random(self):
        """
        Once every second, it generates new random values of (vx, wz), and it sets them.
        """

        self.randomMotionCounter += 1
        if self.randomMotionCounter >= self.RATE:
            self.lastThrottle = self.MAX_THROTTLE * random()
            self.lastRotation = uniform(-self.MAX_ROTATION, self.MAX_ROTATION)
            self.randomMotionCounter = 0
        self.setVelocitySetpoint(vx=self.lastThrottle, yawrate=self.lastRotation)

    def goTo(self):
        """
        It gets (vz, wz) from the position controller, and it sets them.
        """

        [throttle, rotation] = self.positionControl()
        self.setVelocitySetpoint(vx=throttle, yawrate=rotation)

    def rotate(self):
        """
        It sets the velocity setpoint to (0, wz).
        """

        self.setVelocitySetpoint(vx=0, yawrate=self.lastGoalVelocityReceived.angular.z)

    def turn(self):
        """
        It computes the yaw rate proportionally to the angle to be rotated, and then sets the setpoint to (0, wz).
        """

        rotation = (self.lastGoalTurnReceived.z - self.computeCurrentYaw()) * self.KR
        self.setVelocitySetpoint(vx=0, yawrate=rotation)

    def square(self):
        """
        It makes the rover move straight for 5 meters, then turn of 90° and repeat this loop forever.
        """

        if computeDistance(self.odom.pose.pose.position, self.startingPosition) < 5:
            self.setVelocitySetpoint(vx=0.5, yawrate=0)
            if not self.beginTurn:
                self.beginTurn = True
        else:
            if self.beginTurn:
                self.lastGoalTurnReceived.z = self.computeCurrentYaw() + math.radians(90)
                self.beginTurn = False
            angleError = self.lastGoalTurnReceived.z - self.computeCurrentYaw()
            if angleError < math.radians(1) or angleError > math.radians(359):
                self.startingPosition = self.odom.pose.pose.position
            elif angleError < math.radians(10) or angleError > math.radians(350):
                self.setVelocitySetpoint(vx=0, yawrate=math.radians(45))
            else:
                self.setVelocitySetpoint(vx=0, yawrate=math.radians(90))
    # endregion

    # region Callbacks
    def timerCallback(self):
        """
        It is called at a predefined rate, it calls the right method according to the operation mode of the rover, and
        it constantly publishes the velocity setpoint.
        """

        self.targetOperations.get(self.targetMode, self.hold)()

        if self.targetMode != self.targetModeOld:
            self.get_logger().info("Setting %s mode" % self.targetMode)

        self.targetModeOld = self.targetMode

        self.publishVelocitySetpoint()

    def targetCommandCallback(self, request, response):
        """
        @param request: service request
        @param response: service response
        @return: response

        The field "request.operation" describes the mode to be set, and the other fields (defined in
        "TargetCustomCommand.srv") are used to update the various setpoints. Then, the response is returned. In case the
        operation field is not recognized, default values are used and hold mode is set to avoid problems.
        """

        if request.operation in self.targetOperations.keys():
            self.targetMode = request.operation
            response.result = "success"

            self.lastGoalVelocityReceived.linear.x = request.throttle
            self.lastGoalVelocityReceived.angular.z = request.yawrate
            self.lastGoalPositionReceived.x = request.x
            self.lastGoalPositionReceived.y = request.y

            self.lastGoalTurnReceived.z = self.computeCurrentYaw() + math.radians(request.angle)
            self.startingPosition = self.odom.pose.pose.position

        else:
            self.get_logger().warn("Invalid command")
            self.targetMode = "hold"
            response.result = "failure"

            self.lastGoalVelocityReceived.linear.x = 0.0
            self.lastGoalVelocityReceived.angular.z = 0.0
            self.lastGoalPositionReceived.x = 0.0
            self.lastGoalPositionReceived.y = 0.0
            self.lastGoalTurnReceived.z = 0.0
            self.startingPosition = self.odom.pose.pose.position

        return response

    def odomCallback(self, msg):
        """
        @param msg: message

        It saves the data coming from the odometry sensor of the rover.
        """

        self.odom = msg
    # endregion


def main():
    rclpy.init(args=None)
    node = Target()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
