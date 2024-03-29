from robot_control_class import RobotControl


class MazeFugitive:
    pass


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