from robot_control_class import RobotControl

summit_rc = RobotControl(robot_name="summit")

print (summit_rc.move_straight_time("forward", 0.3, 4))
print (summit_rc.turn("counter-clockwise", 0.3, 7.8))
print (summit_rc.move_straight_time("forward", 0.3, 6))
print (summit_rc.turn("counter-clockwise", 0.3, 7.8))
print (summit_rc.move_straight_time("forward", 0.3, 8))

