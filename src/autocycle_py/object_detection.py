import numpy as np
import rospy
from autocycle_extras.msg import ObjectList, Object
from autocycle_extras.srv import DetectObjects, GetTrackingFrame

height = 10000  # vertical dimension in millimeters
width = 20000  # horizontal dimension in millimeters
cell_dim = 50  # dimension of cells in millimeters (cells are squares)

cell_row = int(np.ceil(height / cell_dim))
cell_col = int(np.ceil(width / cell_dim))

# Tunable parameters to determine if something is an object.
col_diff = 50                       # Expected max difference between two adjacent cells in a column.
for_jump_diff = col_diff * 1.5      # Expected min difference between cells in a column to indicate a jump forward.
counter_reps = 2                    # Number of reps required to dictate it is an object.
same_obj_diff = 150                 # maximum diff between horizontal cells to be considered the same object

group_dist = 1500

class Graph:
    def __init__(self, V):
        self.v = V
        self.adj = [[] for _ in range(V)]
        
    def add_edge(self, a, b):
        self.adj[a].append(b)
        self.adj[b].append(a)
    
    def dfs(self, temp, v, visited):
        visited[v] = True
        temp.append(v)
        for x in self.adj[v]:
            if not visited[x]:
                temp = self.dfs(temp, x, visited)
        return temp
        
    def connected_comps(self):
        visited = [False for _ in range(self.v)]
        connect = []
        
        for v in range(self.v):
            if not visited[v]:
                temp = []
                connect.append(self.dfs(temp, v, visited))
        return connect

def p2seg(p, objection):
    seg = (objection.x1 - objection.x2, objection.z1 - objection.z2)
    p = (objection.x1 - p[0], objection.z1 - p[1])
    seg_len = np.sqrt((objection.x1 - objection.x2)**2 + (objection.z1 - objection.z2)**2)
    if seg_len == 0:
        return np.sqrt((objection.x1 - p[0])**2 + (objection.z1 - p[1])**2)
    t = seg[0] / seg_len * p[0] / seg_len + seg[1] / seg_len * p[1] / seg_len
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    nearest = (seg[0]*t, seg[1]*t)
    return np.sqrt((nearest[0] - p[0])**2 + (nearest[1] - p[1])**2)

def seg_dist(obj1, obj2):
    return min(p2seg((obj1.x1, obj1.z1), obj2), 
               p2seg((obj1.x2, obj1.z2), obj2),
               p2seg((obj2.x1, obj2.z1), obj1),
               p2seg((obj2.x2, obj2.z2), obj1))

def left_most(points):
    min_ind = 0
    for i in range(1,len(points)):
        if points[i][0] < points[min_ind][0]:
            min_ind = i
        elif points[i][0] == points[min_ind][0]:
            if points[i][1] > points[min_ind][1]:
                min_ind = i
    return min_ind

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2

def conv_hull(objects):
    points = []
    for o in objects:
        points.append((o.x1,o.z1))
        points.append((o.x2,o.z2))
    n = len(points)
    if n < 3:
        return objects
    
    l = left_most(points)
    
    hull = []
    p = l
    q = 0
    
    while(True):
        hull.append(p)
        q = (p + 1) % n
        for i in range(n):
            if(orientation(points[p], points[i], points[q]) == 2):
                q = i
        p = q
        if(p == l):
            break
    objects = []
    prev = -1
    for ind in range(len(hull)):
        objects.append(Object(points[hull[prev]][0],points[hull[ind]][0], points[hull[prev]][1], points[hull[ind]][1]))
        prev = ind
    return objects

def condense_objects(objects):
    gr = Graph(len(objects))
    for x in range(len(objects)):
        for y in range(x+1, len(objects)):
            if seg_dist(objects[x], objects[y]) < group_dist:
                   gr.add_edge(x,y)
    groups = gr.connected_comps()
    grouped_objects = []
    for gro in groups:
        temp = []
        for y in gro:
            temp.append(objects[y])
        grouped_objects.append(temp)
    
    new_objects = []
    for group in grouped_objects:
        new_objects += conv_hull(group)
    return new_objects

box_dist = 10000  # distance in each dimesion surrounding line segment

def diff(p1, p2):
    return (p1[1] - p2[1],
            p2[0] - p1[0],
            -1 * (p1[0] * p2[1] - p2[0] * p1[1]))

def rotate_point(point, theta):
    return point[0] * np.cos(theta) - point[1] * np.sin(theta), point[0] * np.sin(theta) + point[1] * np.cos(theta)

def trans_point(point, trans):
    return point[0] + trans[0], point[1] + trans[1]

def get_bounding_box(x1, x2, z1, z2):
        o1 = (x1, z1)
        o2 = (x2, z2)
        mov = o2
        o1 = (o1[0] - mov[0], o1[1] - mov[1])
        # Rotates o1 about o2/origin so that o1 is directly above o2
        if o1[1] == 0:
            theta = (1 / 2) * np.pi if o1[0] > 0 else -(1 / 2) * np.pi
        else:
            theta = np.arctan(o1[0] / o1[1]) if o1[1] >= 0 else np.arctan(o1[0] / o1[1]) + np.pi

        o1 = rotate_point(o1, theta)
        p1 = (-box_dist, o1[1] + box_dist)
        p2 = (box_dist, o1[1] + box_dist)
        p3 = (box_dist, -box_dist)
        p4 = (-box_dist, -box_dist)

        # Rotates p1, p2, and p3 by negative theta (original orientation)
        p1 = rotate_point(p1, -theta)
        p2 = rotate_point(p2, -theta)
        p3 = rotate_point(p3, -theta)
        p4 = rotate_point(p4, -theta)

        # Translates points back to relative positions
        p1 = trans_point(p1, mov)
        p2 = trans_point(p2, mov)
        p3 = trans_point(p3, mov)
        p4 = trans_point(p4, mov)
        
        return (p1, p2, p3, p4)

def intersection(x1, x2, z1, z2, objects):
    if x1 < x2:
        points = get_bounding_box(x1, x2, z1, z2)
    elif x1 > x2:
        points = get_bounding_box(x2, x1, z2, z1)
    else:
        if z1 < z2:
            points = ((x1 - box_dist, z1 - box_dist),
                      (x1 + box_dist, z1 - box_dist),
                      (x2 + box_dist, z2 + box_dist),
                      (x2 - box_dist, z2 + box_dist))
        else:
            points = ((x2 - box_dist, z2 - box_dist),
                      (x2 + box_dist, z2 - box_dist),
                      (x1 + box_dist, z1 + box_dist),
                      (x1 - box_dist, z1 + box_dist))
    p2p3 = (points[2][0] - points[1][0], points[2][1] - points[1][1])
    p2p1 = (points[0][0] - points[1][0], points[0][1] - points[1][1])
    point_diffs = (diff(points[1], points[0]), diff(points[2], points[1]), diff(points[3], points[2]), diff(points[3], points[0]))
    
    box_contraints = ((min(points[0][0], points[1][0]),
                       max(points[0][0], points[1][0]),
                       min(points[0][1], points[1][1]),
                       max(points[0][1], points[1][1])),
                      (min(points[1][0], points[2][0]),
                       max(points[1][0], points[2][0]),
                       min(points[1][1], points[2][1]),
                       max(points[1][1], points[2][1])),
                      (min(points[2][0], points[3][0]),
                       max(points[2][0], points[3][0]),
                       min(points[2][1], points[3][1]),
                       max(points[2][1], points[3][1])),
                      (min(points[3][0], points[0][0]),
                       max(points[3][0], points[0][0]),
                       min(points[3][1], points[0][1]),
                       max(points[3][1], points[0][1])))
    for o in objects:
        x3, x4, z3, z4 = o.x1, o.x2, o.z1, o.z2
        p2m = (x3 - points[1][0], z3 - points[1][1])
        p2m2 = (x4 - points[1][0], z4 - points[1][1])
        if (0 <= np.dot(p2m, p2p3) < np.dot(p2p3, p2p3) and 0 <= np.dot(p2m, p2p1) < np.dot(p2p1, p2p1)) or (0 <= np.dot(p2m2, p2p3) < np.dot(p2p3, p2p3) and 0 <= np.dot(p2m2, p2p1) < np.dot(p2p1, p2p1)):
            return True
        if x3 > x4:
            temp = (x3, z3)
            x3, z3 = (x4, z4)
            x4, z4 = temp
        diffs_obj = (z3 - z4, x4 - x3, -1 * (x3 * z4 - x4 * z3))
        if z3 > z4:
            temp = z3
            z3 =  z4
            z4 = temp
        for doni in range(4):
            if (D := point_diffs[doni][0] * diffs_obj[1] - point_diffs[doni][1] * diffs_obj[0]) != 0:
                x = (point_diffs[doni][2] * diffs_obj[1] - point_diffs[doni][1] * diffs_obj[2]) / D
                z = (point_diffs[doni][0] * diffs_obj[2] - point_diffs[doni][2] * diffs_obj[0]) / D
                if x3 <= x <= x4 and z3 <= z <= z4 and box_contraints[doni][0] <= x <= box_contraints[doni][1] and box_contraints[doni][2] <= z <= box_contraints[doni][3]:
                    return True        
    return False

started = False
iden = 0
iden2 = 0
pub = rospy.Publisher('cycle/objects', ObjectList, queue_size=1)

def object_detection(points):
    global iden, iden2, started, pub
    tracking_frame_getter = rospy.ServiceProxy("get_tracking_frame", GetTrackingFrame)

    if started:
        new_tracking = tracking_frame_getter(iden)
        iden = new_tracking.iden
        objects = new_tracking.obj_lst
    else:
        objects = []
        started = True
    
    cells = np.zeros((cell_row, cell_col))
    to_pub = []
    
    for p in points.data:
        # creates list with x, y, and z coordinate
        z = p.z
        x = int((p.x + (width // 2)) // cell_dim)
        y = int((p.y + (height // 2)) // cell_dim)

        # Dictating the z value for the cell. Currently only finds the minimum value of the cell.
        if 0 <= x < cell_col and 0 <= y < cell_row:
            cells[y, x] = z if cells[y, x] == 0 else min(z, cells[y, x])
    close_arr = np.zeros((1, cell_col))

    # bruh = open("/home/ubuntu/Autocycle/bet.txt", "w")
    # bruh.write(str(cells.tolist()))
    # bruh.close()

    max_dist = 20000  # A really big number
    for col in range(cell_col):
        prev = 0                # Previous cell.
        closest = max_dist      # Minimum z value for an object in the column.
        counter = 0             # Counts the instances in which adjacent cells do not exceed diff.
        min_obj = 0             # Minimum dist to detected object

        for row in range(cell_row):
            if cells[row, col] == 0:
                counter = 0
                min_obj = 0
                prev = 0
                continue
            min_obj = cells[row, col] if counter == 0 else min(min_obj, cells[row, col])
            if abs(cells[row, col] - prev) > col_diff:
                counter = 0
                min_obj = 0
            else:
                counter += 1
            if prev > cells[row, col] + for_jump_diff and row - 2 > 0 and cells[row-2,col] != 0:
                    closest = min(cells[row, col], closest)
            if counter > counter_reps:
                closest = min(min_obj, closest)
            prev = cells[row, col]
        close_arr[0, col] = closest
    #print(close_arr)
    left_bound = 0      # left most cord of object
    right_bound = 0     # right most cord of object
    prev = max_dist           # previous cell's z value
    z_boys = []
    for col in range(cell_col):
        if close_arr[0, col] < max_dist:
            if prev == max_dist:
                left_bound = col
                right_bound = col
                prev = close_arr[0, col]
            elif same_obj_diff > abs(prev - close_arr[0, col]):
                right_bound += 1
                prev = close_arr[0, col]
            else:
                if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                    close_arr[0, left_bound], close_arr[0, right_bound], objects):
                    z_boys.append(Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                close_arr[0, left_bound], close_arr[0, right_bound]))
                prev = max_dist
        elif prev < max_dist:
            if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound], objects):
                z_boys.append(Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                close_arr[0, left_bound], close_arr[0, right_bound]))
            prev = max_dist
    if prev < max_dist:
        if not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_arr[0, left_bound], close_arr[0, right_bound], objects):
                z_boys.append(Object(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                close_arr[0, left_bound], close_arr[0, right_bound]))
    print("THIS SHOULD BE PUBLISHING")
    z_boys = condense_objects(z_boys)
    bruh = pub.publish(to_pub, iden2)
    iden2 += 1
    print(bruh)
    for o in to_pub:
        print(f"({o.x1}, {o.x2}, {o.z1}, {o.z2})")
    return []

def start():
    # Registers node with the master
    rospy.init_node("object_detection")
    
    # Creates Service to be called
    rospy.Service("object_detection", DetectObjects, object_detection)

    # Waits for the tracking frame getter service to be active
    rospy.wait_for_service("get_tracking_frame")
   
    # Waits for the get object list service to be active
    rospy.wait_for_service("object_list_getter")

    # Waits to be called
    rospy.spin()
