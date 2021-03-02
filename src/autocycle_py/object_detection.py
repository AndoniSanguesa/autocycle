import numpy as np
import rospy
from autocycle.msg import Object
from autocycle.srv import DetectObjects

height = 10000              # vertical dimension in millimeters
width = 20000               # horizontal dimension in millimeters
cell_dim = 20               # dimension of cells in millimeters (cells are squares)

cell_col = int(np.ceil(height/cell_dim))
cell_row = int(np.ceil(width/cell_dim))

# Tunable parameters to determine if something is an object.
col_diff = 20           # Expected max difference between two adjacent cells in a column.
counter_reps = 3        # Number of reps required to dictate it is an object.
same_obj_diff = 20      # maximum diff between horizontal cells to be considered the same object


def object_detection(points):
    pub = rospy.Publisher('cycle/objects', Object, queue_size=25)
    cells = np.zeros((cell_row, cell_col))
    to_pub = []

    for p in points.data:
        # creates list with x, y, and z coordinate
        x, y, z = p.x, p.y, p.z
        x = x//cell_col + width//2    # Add offset such that the points are translated to cords such as lidar mounting offset accounted for
        y = y//cell_row     # Add offset such that the points are translated to cords such as lidar mounting offset accounted for

        # Dictating the z value for the cell. Currently only finds the minimum value of the cell.
        if 0 <= x < cell_col and 0 <= y < cell_row:
            cells[x, y] = z if cells[x, y] == 0 else min(z, cells[x, y])

    close_arr = np.zeros((1, cell_col))

    for col in range(cell_col):
        prev = 0        # Previous cell.
        closest = 0     # Minimum z value for an object in the column.
        counter = 0     # Counts the instances in which adjacent cells do not exceed diff.
        min_obj = 0     # Minimum dist to detected object
        for row in range(cell_row):
            min_obj = cells[row, col] if counter == 0 else min(min_obj, cells[row, col])
            if abs(cells[row, col] - prev) > col_diff:
                counter = 0
                min_obj = 0
            else:
                counter += 1
            if prev > cells[row, col] + col_diff:
                closest = min(cells[row, col], closest)
            if counter > counter_reps:
                closest = min(min_obj, closest)
            prev = cells[row, col]
        close_arr[0, col] = closest

    left_bound = 0      # left most cord of object
    right_bound = 0     # right most cord of object
    prev = 0            # previous cell's z value

    for col in range(cell_col):
        if close_arr[0, col] != 0:
            if prev == 0:
                left_bound = col
                right_bound = col
                prev = close_arr[0, col]
            elif same_obj_diff > abs(prev - close_arr[0, col]):
                right_bound += 1
                prev = close_arr[0, col]
            else:
                obj = Object(left_bound*cell_dim - width/2, right_bound * cell_dim - width/2, 
                             close_arr[0, left_bound], close_arr[0, right_bound])
                to_pub.append(obj)
                prev = 0
        elif prev != 0:
            obj = Object(left_bound * cell_dim - width / 2, right_bound * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound])
            to_pub.append(obj)
            prev = 0
    pub.publish(to_pub)
    return True

def start():
    # Registers node with the master
    rospy.init_node("object_detection")
    
    # Creates Service to be called
    rospy.Service("object_detection", DetectObjects, object_detection)
    
    # Waits to be called
    rospy.spin()
