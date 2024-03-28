from robot_control_class import RobotControl

rc = RobotControl()

full_180 = rc.get_laser_full()

max_distance = 0
min_distance = float("inf")

for dir in full_180:
    if dir > max_distance:
        max_distance = dir
    if dir < min_distance:
        min_distance = dir

print ("The largest distance from the robot is ", max_distance)
print ("The smallest distance from the robot is ", min_distance)



