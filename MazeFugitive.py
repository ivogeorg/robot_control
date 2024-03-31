from math import isinf, degrees, radians
from geometry_msgs.msg import Twist, Point, Quaternion
from robot_control_class import RobotControl

# Notes:
# 1. There can be two solutions, one hard-coded and one universal.
#    Do hard-coded first, based on the initial position and orientation
#    of the TurtleBot.
# 2. There will be two key components for the universal solution, which
#    actually moves into mapping the environment, planning, and navigation:
#    a. Finding the best direction to move straight in, based on a
#       procedure using the full laser readings and turning the 
#       robot to get 360 degrees, also using the history of the
#       motion.
#    b. Finding the best time to stop and turn, based on a procedure,
#       similar to the one in (a).

# Notes 2:
# 1. No need to rotate to get a 360 deg distance reading. Start with the
#    current position in the middle of the corridor and current orientation
#    along it. Move in this direction first. This solution needs only the
#    laser readings.
# 2. Create a state machine (enum?) so that the robot can maintain a known
#    state at all times and have clear actions to transition to a new state.
#    Start with the following set of states: {
#    a. CORRIDOR: The robot is in the middle (0 approximately equal to 719)
#       and 360 > {0, 719}. This is the starting state.
#    b. CORNER_ARRIVING: The robot enters a corner when either 0 > 719 or
#       vice versa. It has to move to the center of the new corridor by
#       achieving 360 approximately equal to the smaller of {0, 719}.
#    c. TURNING: At the end of the CORNER_ARRIVING, the robot enters this
#       state and turns 90 deg in the direction of the larger of {0, 719}
#       before the turn. After the turn, the robot enters the next state,
#       CORNER_LEAVING.
#    d. CORNER_LEAVING: 360 and one of the sides {0, 719} are larger than
#       the other side. Moving forward until the sides are approximately
#       equal to enter the CORRIDOR state again.
#    e. EXITING: If during CORNER_ARRIVING one of the sides readings is inf
#       the robot has found the exit. After performing the turn toward inf,
#       the 360 becomes inf, and robot can enter the EXITING state.
#    f. OUT: When the robot is exiting, the side readings are not so 
#       important as it won't be turning anymore. Once the three main 
#       readings {0, 360, 719} are all inf, the robot is out of the maze 
#       and can stop.
# 3. State transitions:
#    a. CORRIDOR -> CORNER_ARRIVING
#    b. CORNER_ARRIVING -> TURNING
#    c. TURNING -> {CORNER_LEAVING, EXITING} depending on an inf reading
#    d. CORNER_LEAVING -> CORRIDOR
#    e. EXITING -> OUT
# 4. The definition of a corner might not be general enough to solve a 
#    T-intersection, like the one right by the exit if the robot is
#    arriving from the right. To incorporate a T-intersection, look at
#    both side readings in CORNER_ARRIVING and turn toward the larger one,
#    which is likely to be inf. This is taken care of in the (c) transition
#    above.
# 5. This solution relies heavily on the fidelity of the rotate function,
#    but notice that it is using odometry messages, which allows for a
#    more flexible solution, if there is time. The turns are within
#    angular_tolerance of the target turn angle. If this error accumulates
#    over time, what constraints for states and transitions might need to
#    have tolerances to accommodate it? It is not necessary to stick to the
#    center of the corridor, but how can the corners be generalized so that
#    they are functional no matter where exactly the robot is?
# 6. A more robust solution could ensure that there is a dynamic correction
#    of the trajectory, for example equidistance to the walls on both sides
#    while in state CORRIDOR. This will require close monitoring of the 
#    laser readings and applying corrections to the pose of the robot.
# 7. For a robust solution, the state transitions have to be both tight and
#    comprehensive. Draw a state diagram and distinguish well between the 
#    alternative paths at branching points.

# LIDARs:
# 1. Turtlebot 
#    - kobuki
#    - counterclockwise
#    - range 180 degrees
#    - 720 points
#    - indices                 360
#                               ^
#                               |
#                               |
#                     719 <----bot----> 0 
# 2. Summit XL
#    - hokuyo_base 

class MazeFugitive(RobotControl):

    # Assume Turtlebot
    def __init__(self, speed) -> None:
        super().__init__()
        self.speed = speed
        # distance from wall
        right_dist = self.get_laser(0)
        left_dist = self.get_laser(719)
        self.dist = (right_dist + left_dist) / 1.5
        # last rotation from odometry
        point = Point()
        (point, rotation) = self.get_odom()
        self.last_rotation = rotation


    def move_straight_distance(self, speed, dist) -> None:
        """Move forward until front laser reads dist"""
        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.cmd.linear.x = speed

        # loop until dist, note sleep(1) in method
        while (self.dist <= self.get_front_laser()):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)

        # set velocity to zero to stop the robot
        self.stop_robot()

        print("Robot stopped at " + str(self.get_front_laser()) + "(dist = " + str(self.dist) + ")")


    def move_out(self, speed) -> None:
        """Move forward until front and side lasers read inf"""
        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.cmd.linear.x = speed

        # loop until dist, note sleep(1) in method
        while (not isinf(self.get_laser(0)) and
               not isinf(self.get_laser(360)) and
               not isinf(self.get_laser(719))):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)

        # set velocity to zero to stop the robot
        self.stop_robot()

        print("Robot stopped outside")

    def rotate_corrected(self, deg) -> None:
        """Attempt to correct for rotational drift"""
        point = Point()
        (point, rotation) = self.get_odom()
        self.rotate(deg - degrees(rotation - self.last_rotation))
        (point, rotation) = self.get_odom()
        self.last_rotation = rotation


    def hardcoded_maze_escape(self, speed, dist) -> None:
        # Hardcoded trajectory assuming robot pose
        self.move_straight_distance(speed, dist)
        self.rotate_corrected(-90.0)
        self.move_straight_distance(speed, dist)
        self.rotate_corrected(-90.0)
        self.move_straight_distance(speed, dist)
        self.rotate_corrected(90.0)
        self.move_out(speed)


if __name__ == "__main__":
    robot = MazeFugitive(0.3)
    # point = Point()
    # (point, rotation) = robot.get_odom()
    # print("Odometry:")
    # print(robot.get_odom())
    # print("Odometry point:")
    # print(point)
    # print("Odometry rotation:")
    # print(rotation)
    # robot.move_straight_distance(robot.speed, robot.dist)
    # (point, rotation) = robot.get_odom()
    # print("Odometry:")
    # print(robot.get_odom())
    # print("Odometry point:")
    # print(point)
    # print("Odometry rotation:")
    # print(rotation)
    robot.hardcoded_maze_escape(robot.speed, robot.dist)

