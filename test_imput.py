from robot_control_class import RobotControl

rc = RobotControl()

pos = int(input("What laser position reading do you want (0-719)? "))

print ("The distance for postion ", pos, " is ", rc.get_laser(pos))




