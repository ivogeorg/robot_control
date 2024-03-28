from robot_control_class import RobotControl

rc = RobotControl()

positions = [0, 100, 200, 300, 400, 500, 600, 719]

laser_full = rc.get_laser_full()

readings = {}

for p in positions:
    readings[p] = laser_full[p]

print (readings)



