import numpy as np
import rospy
from autocycle.msg import ObjectList, Object
from autocycle.srv import DetectObjects

height = 100  # vertical dimension in millimeters
width = 200  # horizontal dimension in millimeters
cell_dim = 20  # dimension of cells in millimeters (cells are squares)

cell_row = int(np.ceil(height / cell_dim))
cell_col = int(np.ceil(width / cell_dim))

# Tunable parameters to determine if something is an object.
col_diff = 20  # Expected max difference between two adjacent cells in a column.
counter_reps = 1  # Number of reps required to dictate it is an object.
same_obj_diff = 20  # maximum diff between horizontal cells to be considered the same object

box_dist = 20  # distance in each dimesion surrounding line segment
trans_mat = np.eye(3)  # translation matrix
rot_mat = np.zeros((3, 3))  # rotation matrix
rot_mat[2, 2] = 1
point_mat = np.zeros((3, 4))  # matrix with point 1 in col 0 and point 2 in col 1
point_mat[2, :] = [1, 1, 1, 1]


def diff(p1, p2):
    return (p1[0] - p2[0],
            p1[1] - p2[1],
            p1[0] * p2[1] - p2[0] * p1[1])


def box_verts(x1, x2, z1, z2):
    global point_mat
    point_mat[0, 0] = x1
    point_mat[1, 0] = z1
    point_mat[0, 1] = x1
    point_mat[1, 1] = z1
    point_mat[0, 2] = x2
    point_mat[1, 2] = z2
    point_mat[0, 3] = x2
    point_mat[1, 3] = z2

    # Translation matrix translates line segment so (x1, z1) is at the origin
    trans_mat[0, 2] = -x1
    trans_mat[1, 2] = -z1

    # Find angle, theta, between x axis and segment. Rotate counterclockwise -theta.
    slope = (z2 - z1) / (x2 - x1)
    theta = np.arctan(slope)
    rot_mat[0, 0] = np.cos(-theta)
    rot_mat[0, 1] = np.sin(-theta)
    rot_mat[1, 0] = -rot_mat[0, 1]
    rot_mat[1, 1] = rot_mat[0, 0]

    point_mat = rot_mat @ trans_mat @ point_mat
    # bot left
    point_mat[0, 0] -= box_dist
    point_mat[1, 0] -= box_dist

    # top left
    point_mat[0, 1] -= box_dist
    point_mat[1, 1] += box_dist

    # top right
    point_mat[0, 2] += box_dist
    point_mat[1, 2] += box_dist

    # bottom right
    point_mat[0, 3] += box_dist
    point_mat[1, 3] -= box_dist

    rot_mat[0, 0] = np.cos(theta)
    rot_mat[0, 1] = np.sin(theta)
    rot_mat[1, 0] = -rot_mat[0, 1]
    rot_mat[1, 1] = rot_mat[0, 0]

    trans_mat[0, 2] = x1
    trans_mat[1, 2] = z1

    point_mat = trans_mat @ rot_mat @ point_mat
    return ((point_mat[0, 0], point_mat[1, 0]),
            (point_mat[0, 1], point_mat[1, 1]),
            (point_mat[0, 2], point_mat[1, 2]),
            (point_mat[0, 3], point_mat[1, 3]))


def intersection(x1, x2, z1, z2, objects):
    if x1 < x2:
        points = box_verts(x1, x2, z1, z2)
    elif x1 > x2:
        points = box_verts(x2, x1, z2, z1)
    else:
        if z1 < z2:
            points = ((x1 - box_dist, z1 - box_dist),
                      (x2 - box_dist, z2 + box_dist),
                      (x2 + box_dist, z2 + box_dist),
                      (x1 + box_dist, z1 - box_dist))
        else:
            points = ((x2 - box_dist, z2 - box_dist),
                      (x1 - box_dist, z1 + box_dist),
                      (x1 + box_dist, z1 + box_dist),
                      (x2 + box_dist, z2 - box_dist))
    p2p3 = (points[2][0] - points[1][0], points[2][1] - points[1][1])
    p2p1 = (points[0][0] - points[1][0], points[0][1] - points[1][1])

    point_diffs = (
    diff(points[0], points[1]), diff(points[0], points[2]), diff(points[1], points[3]), diff(points[2], points[3]))

    for x3, x4, z3, z4 in objects:
        # intersection = []
        if x3 > x4:
            temp = (x3, z3)
            x3, z3 = (x4, z4)
            x4, z4 = temp
        diffs_obj = (x3 - x4, z3 - z4, x3 * z4 - x4 * z3)

        for x in range(4):
            if (D := point_diffs[x][0] * diffs_obj[1] - point_diffs[x][1] * diffs_obj[0]) != 0:
                x = (point_diffs[x][2] * diffs_obj[1] - point_diffs[x][1] * diffs_obj[2]) / D
                if x3 <= x <= x4:
                    y = (point_diffs[x][0] * diffs_obj[2] - point_diffs[x][2] * diffs_obj[0]) / D
                    p2m = (x - points[0][0], y - points[0][1])
                    if 0 <= np.dot(p2m, p2p3) < np.dot(p2p3, p2p3) and 0 <= np.dot(p2m, p2p1) < np.dot(p2p1, p2p1):
                        return True
    return False

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
        prev = 0                # Previous cell.
        closest = 500000        # Minimum z value for an object in the column.
        counter = 0             # Counts the instances in which adjacent cells do not exceed diff.
        min_obj = 0             # Minimum dist to detected object
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
                if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound], objects):
                    obj = Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                close_arr[0, left_bound], close_arr[0, right_bound])
                    to_pub.append(obj)
                if close_arr[0, col] != 0:
                    left_bound = col
                    right_bound = col
                    prev = close_arr[0, col]
                else:
                    prev = 0
        elif prev != 0:
            if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound], objects):
                obj = Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                            close_arr[0, left_bound], close_arr[0, right_bound])
                to_pub.append(obj)
            prev = 0
    if prev != 0:
        if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound], objects):
                obj = Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                            close_arr[0, left_bound], close_arr[0, right_bound])
                to_pub.append(obj)
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
