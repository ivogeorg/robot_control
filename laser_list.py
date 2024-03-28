from robot_control_class import RobotControl


summit_rc = RobotControl(robot_name="summit")

def readings(r1, r2, r3):
    reading_indices = [r1, r2, r3]
    return [summit_rc.get_laser_summit(ri) for ri in reading_indices]


read_list = []

for _ in range(3):
    read_list.append(int(input("Reading index (0-1000): ")))

distances = readings(read_list[0], read_list[1], read_list[2])

print ("The distance readings for the provided indices are ", distances)

