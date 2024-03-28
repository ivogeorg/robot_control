from robot_control_class import RobotControl

rc = RobotControl()

# if no wall < 1 m, keep moving, else stop

# initial reading to start moving
to_wall = rc.get_laser(360)
if to_wall >= 1:
    rc.move_straight()

while rc.get_laser(360) >= 1:
    continue

print ("Robot stopped at a distance of ", rc.get_laser(360), " m from the wall!")

