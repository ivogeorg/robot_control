from robot_control_class import RobotControl

summit_rc = RobotControl(robot_name="summit")

print (summit_rc.move_straight_time("forward", 0.3, 5))
print (summit_rc.turn("clockwise", 0.3, 7))

