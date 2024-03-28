from robot_control_class import RobotControl

class MoveRobot:
    def __init__(self, linear, angular, speed, time, init_x, init_y, init_orientation) -> None:
        self.robot_control = RobotControl(robot_name="summit")
        self.init_x = init_x
        self.init_y = init_y
        self.init_orientation = init_orientation
        self.linear = linear  # "forward" or "backward"
        self.angular = angular  # "clockwise" or "counter-clockwise"
        self.speed = speed  # of straight motion (m/s) or rotation (rad/s)
        self.time = time  # for straight motion
        self.turn_time = 7   # estimate for time to turn 90 deg

        # code to move to init position and face the right way

    def do_square(self) -> None:
        for _ in range(3):  # 3 times go straight and turn
            self.go_straight()
            self.turn()


    def go_straight(self) -> None:
        self.robot_control.move_straight_time(self.linear, self.speed, self.time)


    def turn(self) -> None:
        self.robot_control.turn(self.angular, self.speed, self.turn_time)

outer_square_robot = MoveRobot("forward", "clockwise", 0.3, 4, 50, 40, "north")  # fake coordinates
outer_square_robot.do_square()
inner_square_robot = MoveRobot("forward", "clockwise", 0.3, 2, 48, 38, "north")  # fake coordinates
inner_square_robot.do_square()
