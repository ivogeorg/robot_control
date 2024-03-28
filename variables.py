from robot_control_class import RobotControl

rc = RobotControl()

laser1 = rc.get_laser(0)

print ("The distance measured in direction 0 is ", laser1, " m.")

laser2 = rc.get_laser(180)

print ("The distance measured in direction 180 is ", laser2, " m.")

laser2 = rc.get_laser(370)

print ("The distance measured in direction 370 is ", laser2, " m.")

