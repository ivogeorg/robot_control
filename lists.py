from robot_control_class import RobotControl

rc = RobotControl()

positions = [0, 360, 719]

laser_full = rc.get_laser_full()

for p in positions:
    print ("Position ", p, ": ", laser_full[p])

