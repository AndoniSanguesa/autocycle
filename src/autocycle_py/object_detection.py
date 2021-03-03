import numpy as np
import rospy
from autocycle.msg import ObjectList, Object
from autocycle.srv import DetectObjects

height = 100              # vertical dimension in millimeters
width = 200               # horizontal dimension in millimeters
cell_dim = 20               # dimension of cells in millimeters (cells are squares)

cell_row = int(np.ceil(height/cell_dim))
cell_col = int(np.ceil(width/cell_dim))

# Tunable parameters to determine if something is an object.
col_diff = 20           # Expected max difference between two adjacent cells in a column.
counter_reps = 1        # Number of reps required to dictate it is an object.
same_obj_diff = 20      # maximum diff between horizontal cells to be considered the same object


def object_detection(points):
    pub = rospy.Publisher('cycle/objects', ObjectList, queue_size=1)
    cells = np.zeros((cell_row, cell_col))
    to_pub = []

    for p in points.data:
        # creates list with x, y, and z coordinate
        x, y, z = p.x, p.y, p.z
        x = int((x + (width//2))//cell_dim)
        y = int((y + (height//2))//cell_dim)

        # Dictating the z value for the cell. Currently only finds the minimum value of the cell.
        if 0 <= x < cell_col and 0 <= y < cell_row:
            cells[y, x] = z if cells[y, x] == 0 else min(z, cells[y, x])
    close_arr = np.zeros((1, cell_col))

    for col in range(cell_col):
        prev = 0        # Previous cell.
        closest = 500000     # Minimum z value for an object in the column.
        counter = 0     # Counts the instances in which adjacent cells do not exceed diff.
        min_obj = 0     # Minimum dist to detected object
        for row in range(cell_row):
            if cells[row, col] == 0:
                continue
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
                obj = Object(left_bound*cell_dim - width/2, (right_bound + 1) * cell_dim - width/2, 
                             close_arr[0, left_bound], close_arr[0, right_bound])
                to_pub.append(obj)
                rospy.loginfo("BRUH at DETECTION")
                if close_arr[0, col] != 0:
                    left_bound = col
                    right_bound = col
                    prev = close_arr[0, col]
                else:
                    prev = 0
                
        elif prev != 0:
            obj = Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound])
            to_pub.append(obj)
            rospy.loginfo("BRUH at DETECTION")
            prev = 0
    if prev != 0:
        obj = Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                     close_arr[0, left_bound], close_arr[0, right_bound])
        to_pub.append(obj)
        rospy.loginfo("BRUH at DETECTION")
    pub.publish(to_pub)
    return []

def start():
    # Registers node with the master
    rospy.init_node("object_detection")
    
    # Creates Service to be called
    # Creates Service to be called
    rospy.Service("object_detection", DetectObjects, object_detection)
    
    # Waits to be called
    rospy.spin()
