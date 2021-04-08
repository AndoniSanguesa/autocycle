import bezier
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.font_manager import FontProperties
import time
import rospy
from autocycle_extras.srv import GetData
from autocycle_extras.msg import Curve, ObjectList
from std_srvs.srv import Empty


def rotate_point(point, theta):
    return point[0] * np.cos(theta) - point[1] * np.sin(theta), point[0] * np.sin(theta) + point[1] * np.cos(theta)


def trans_point(point, trans):
    return point[0] + trans[0], point[1] + trans[1]


class Obstacle:
    def __init__(self, points):
        self.points = points
        self.control_points = []
        self.bound_box = []
        self.bound_max_min = []
        self.shown = False
        self.side = 0
        self.dir = 1
        self.err = 0.2

        self.get_bounding_box()
        self.p2p3 = (self.bound_box[2][0] - self.bound_box[1][0], self.bound_box[2][1] - self.bound_box[1][1])
        self.p2p1 = (self.bound_box[0][0] - self.bound_box[1][0], self.bound_box[0][1] - self.bound_box[1][1])
        self.next_side([0], [0])
        self.calculate_control_point()

    # Creates bounding box that will be used for intersection
    def get_bounding_box(self):
        offset = 0.6

        # o1 is the point where o1.x < o2.x
        if self.points[0][1] >= self.points[0][0]:
            o1 = (self.points[0][0], self.points[1][0])
            o2 = (self.points[0][1], self.points[1][1])
        else:
            o1 = (self.points[0][1], self.points[1][1])
            o2 = (self.points[0][0], self.points[1][0])

        # Translates points so that 'o2' is centered on 0,0
        mov = o2
        o1 = (o1[0] - mov[0], o1[1] - mov[1])
        # Rotates o1 about o2/origin so that o1 is directly above o2
        if o1[1] == 0:
            theta = (1 / 2) * np.pi if o1[0] > 0 else -(1 / 2) * np.pi
        else:
            theta = np.arctan(o1[0] / o1[1]) if o1[1] >= 0 else np.arctan(o1[0] / o1[1]) + np.pi

        o1 = rotate_point(o1, theta)
        p1 = (-offset, o1[1] + offset)
        p2 = (offset, o1[1] + offset)
        p3 = (offset, -offset)
        p4 = (-offset, -offset)

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

        # Sets the bounding box points
        self.bound_box = [p1, p2, p3, p4]
        zipped = list(zip(*self.bound_box))
        xs = zipped[0]
        ys = zipped[1]
        self.bound_max_min = [(min(xs), max(xs)), (min(ys), max(ys))]

    # Calculates the control points associated with this obstacle
    def calculate_control_point(self):
        self.control_points.clear()
        # o1 is the point where o1.x < o2.x
        if self.side == 0:
            o1 = (self.points[0][0], self.points[1][0])
            o2 = (self.points[0][1], self.points[1][1])
        else:
            o1 = (self.points[0][1], self.points[1][1])
            o2 = (self.points[0][0], self.points[1][0])

        # Translates points so that 'o2' is centered on 0,0
        mov = o2
        o1 = (o1[0] - mov[0], o1[1] - mov[1])

        # Rotates o1 about o2/origin so that o1 is directly above o2
        if not o1[0] == 0 and (o1[1] == 0 or abs(np.arctan(o1[1] / o1[0])) < (1 / 4) * np.pi):
            cp1 = (o1[0] / 2, self.err + 0.5)
            cp2 = (cp1[0] - 0.5, cp1[1] - 0.5)
            cp3 = (cp1[0] + 0.5, cp1[1] - 0.5)

            # Translates control points to their original location
            cp1 = trans_point(cp1, mov)
            cp2 = trans_point(cp2, mov)
            cp3 = trans_point(cp3, mov)

        else:
            theta = np.arctan(o1[0] / o1[1]) if o1[1] > 0 else np.arctan(o1[0] / o1[1]) + np.pi

            o1 = rotate_point(o1, theta)
            cp1 = (o1[0], o1[1] + self.err)
            cp2 = (cp1[0] - 0.5, cp1[1] - 0.5)
            cp3 = (cp1[0] + 0.5, cp1[1] - 0.5)

            # Rotates control points back to original orientation
            cp1 = rotate_point(cp1, -theta)
            cp2 = rotate_point(cp2, -theta)
            cp3 = rotate_point(cp3, -theta)

            # Translates control points to their original location
            cp1 = trans_point(cp1, mov)
            cp2 = trans_point(cp2, mov)
            cp3 = trans_point(cp3, mov)

        self.control_points = [cp1, cp2, cp3]

    def adjust_err(self):
        self.err += 0.8
        self.calculate_control_point()

    # Determines if a set of x and y coordinates at any point intersect with the obstacle
    def intersect(self, xs, ys):
        for i in range(len(xs)):
            # Throws out any x and y values too far away to bother checking
            if self.bound_max_min[0][0] <= xs[i] <= self.bound_max_min[0][1] \
                    and self.bound_max_min[1][0] <= ys[i] <= self.bound_max_min[1][1]:
                if self.points[0][0] == self.points[0][1]:
                    self.shown = True
                    return True
                # ~~Magic~~ Code
                point = (xs[i], ys[i])
                p2m = (point[0] - self.bound_box[1][0], point[1] - self.bound_box[1][1])
                if 0 < np.dot(p2m, self.p2p3) < np.dot(self.p2p3, self.p2p3) and \
                        0 < np.dot(p2m, self.p2p1) < np.dot(self.p2p1, self.p2p1):
                    self.shown = True
                    return True
        return False

    # Determines which end point of the obstacle is closest to the provided x and y value
    def next_side(self, x, y):
        s1_pnt_ind = find_x_ind(xs, self.points[0][0])
        s2_pnt_ind = find_x_ind(xs, self.points[0][1])

        diff_s1 = (self.points[0][0] - xs[s1_pnt_ind]) ** 2 + (self.points[1][0] - ys[s1_pnt_ind]) ** 2
        diff_s2 = (self.points[0][1] - xs[s2_pnt_ind]) ** 2 + (self.points[1][1] - ys[s2_pnt_ind]) ** 2
        # (diff_s1, diff_s2)
        if diff_s1 <= diff_s2:
            self.side = 0
        else:
            self.side = 1

        if self.points[1][self.side] < self.points[1][self.side - 1]:
            self.dir = -1
        else:
            self.dir = 1

        self.calculate_control_point()

    def get_obst_points(self):
        return self.points

    def get_control_points(self):
        return self.control_points


# Looks for closest x_val in an array through binary search
def find_closest_x(x_vals, target):
    start = 0
    end = len(x_vals) - 1
    while start != end:
        half_ind = start + math.floor((end - start) / 2)
        half_val = x_vals[half_ind]
        if end == start + 1:
            if abs(x_vals[end] - target) > abs(x_vals[start] - target):
                return start
            else:
                return end
        if half_val > target:
            end = half_ind - 1
        elif half_val < target:
            start = half_ind + 1
        else:
            return half_ind
    return start


# The CurveAssistant stores all the information for a given curve and
# allows the user to perform calculations using it

class CurveAssistant:
    def __init__(self, end_dist):
        self.end_dist = end_dist
        self.control_points = []
        self.obstacles = []
        self.heading = 0
        self.coordinates = [[], []]
        self.compute_control_points()

    # Updates obstacle coordinates contained in the curve graph
    def obstacle_coordinates(self):
        # Clears the coordinates for recalculation
        self.coordinates[0].clear()
        self.coordinates[1].clear()

        # Gets each set of coordinates and stores them
        for obstacle in self.obstacles:
            coord = obstacle.get_obst_points()
            self.coordinates[0].append(coord[0])
            self.coordinates[1].append(coord[1])
        return self.coordinates

    def get_curve(self):
        self.compute_control_points()
        nodes = self.get_fortran()
        return bezier.Curve(nodes, self.get_num_control_points() - 1)

    # Creates obstacle object and produces initial control points
    def create_obstacle(self, points):
        self.obstacles.append(Obstacle(points))
        self.compute_control_points()

    def clear_obstacles(self):
        self.obstacles.clear()

    # Returns control Points
    def get_control_points(self):
        self.compute_control_points()
        return self.control_points

    # Returns the last control point
    def get_last_control_point(self):
        return self.get_control_points()[-1]

    # Updates the control Points
    def compute_control_points(self):
        # Clears control points
        self.control_points.clear()
        self.control_points.append([0, 0])
        # self.control_points.append([1, 0])

        # Adds all control points associated to each obstacle
        for obstacle in self.obstacles:
            if obstacle.shown:
                coord = obstacle.get_control_points()
                self.control_points.extend(coord)

        # These lines should always be computed last
        self.control_points.append([np.cos(self.heading) * self.end_dist, np.sin(self.heading) * self.end_dist])

    # Returns the number of control points
    def get_num_control_points(self):
        return len(self.control_points)

    # Returns the fortran array needed by the bezier library
    def get_fortran(self):
        xs = []
        ys = []
        for point in self.control_points:
            xs.append(point[0])
            ys.append(point[1])
        return np.asfortranarray([
            xs,
            ys,
        ])

    # Converts an xy point into an nt point
    def convert_nt(self, x, y):
        scal = np.sqrt(2) / 2
        x1 = scal * (x - y)
        y1 = scal * (x + y)
        return [x1, y1]

    # Returns the end distance
    def get_end_dist(self):
        return self.end_dist


iden = 0
resolution = 0.04

## OPEN TESTDATA FILE. ONLY FOR TESTING UNTIL INTEGRATION ##
time0 = time.time()
fontP = FontProperties()
fontP.set_size('small')
labels = []
# Distance to the end of the graph (our max viewing distance)
end_dist = 50000

# The final x and y values of the Bezier plot
x_vals = []
y_vals = []

# Desired heading
des_heading = 0

# Creates a CurveAssistant that will allow us to access our data
curveas = CurveAssistant(end_dist)

# The coordinate of the final control point at the end of the graph
lastpoint = curveas.get_last_control_point()

tot_time = 0


def plot():
    for obstacle in curveas.obstacles:
        # Converts the x-y points into the nt points
        points = obstacle.points
        nt_1 = curveas.convert_nt(points[0][0], points[1][0])
        nt_2 = curveas.convert_nt(points[0][1], points[1][1])
        nt_obst = [[nt_1[0], nt_2[0]], [nt_1[1], nt_2[1]]]
        plt.plot(nt_obst[0], nt_obst[1], linewidth=4)

    # Plots and displays bezier curve
    nt_last_point = curveas.convert_nt(lastpoint[0], lastpoint[1])
    plt.plot([0, nt_last_point[0]], [0, nt_last_point[1]], color='green', linestyle='dashed')
    nt_bez_list = [[], []]
    for index in range(len(x_vals)):
        # Converts xy points to nt points for the bezier curve
        nt_bez_cur = curveas.convert_nt(x_vals[index], y_vals[index])
        nt_bez_list[0].append(nt_bez_cur[0])
        nt_bez_list[1].append(nt_bez_cur[1])

    # Plots Bezier Curve
    plt.plot(nt_bez_list[0], nt_bez_list[1])
    # Plots Control points after converting from xy to nt
    ctr_points = curveas.get_control_points()
    ind = 0
    for x in range(len(ctr_points)):
        nt_ctr = curveas.convert_nt(ctr_points[x][0], ctr_points[x][1])
        plt.plot(nt_ctr[0], nt_ctr[1], 'ro')
        ind += 1
    labels.extend(['Global Path', 'Actual Path', 'Control Point'])
    plt.legend(labels, prop=fontP)

    # Sets the plot axis to not be dumb
    plt.axis('square')
    plt.axis([0, nt_last_point[0], 0, nt_last_point[1]])
    plt.show()
    plt.clf()


# Calculates x and y points for the bezier curve
def calculate_curve():
    # Empties the x and y values for the next curve
    x_vals.clear()
    y_vals.clear()

    # The coordinates of the obstacles
    curveas.compute_control_points()

    # List of control points
    nodes = curveas.get_fortran()

    # This command creates a curve object which allows us to check the characteristics of the curve
    curve = bezier.Curve(nodes, curveas.get_num_control_points() - 1)

    index = 0

    while index <= 1.001:
        curpoint = curve.evaluate(index * 1.00)

        # Adds x and y values to their respective array
        x_vals.append(curpoint[0][0])
        y_vals.append(curpoint[1][0])
        index += resolution


def is_obstacle_block():
    """
    :returns a list of obstacles which currently intersect the path
    """
    obst_block = []
    for obstacle in curveas.obstacles:
        if obstacle.intersect(x_vals, y_vals):
            obst_block.append(obstacle)
    return obst_block


def get_dervs(x, acc=0.000001):
    curve = curveas.get_curve()
    subs = lambda x: list(map(lambda x: x[0], curve.evaluate(x)))
    this_val = subs(x)
    next_val = subs(x + acc)
    prev_val = subs(x - acc)
    return (((next_val[0] - 2 * this_val[0] + prev_val[0]) / (acc ** 2),
             (next_val[1] - 2 * this_val[1] + prev_val[1]) / (acc ** 2)),
            ((this_val[0] - next_val[0]) / acc,
             (this_val[1] - next_val[1]) / acc))


def get_deltas():
    global step
    ## Calls the data getter to give us the latest roll
    num = 1.02 * np.cos(0)
    step = 0.01  # decrease step size for greater precision
    dervs = [get_dervs(i) for i in np.arange(step, 1 + step, step)]
    cosdelt = np.cos(0.08)
    curve_deltas = [[i for i in np.arange(step, 1 + step, step)], [0] * (int(1 / step))]
    cnt = 0
    for i in range(int(1 / step)):
        derv = dervs[i]
        calc1 = abs(derv[1][0] * derv[0][1] - derv[1][1] * derv[0][0])
        if calc1 != 0:
            calc2 = (cosdelt * (((derv[1][0] ** 2 + derv[1][1] ** 2) ** (3 / 2)) / calc1))
            if calc2 != 0:
                curve_deltas[1][cnt] = num / calc2
                cnt += 1

    curve_deltas = [curve_deltas[0][0:cnt], curve_deltas[1][0:cnt]]

    return curve_deltas


def create_environment(req):
    global resolution, des_heading, iden, tot_time

    start_time = time.time()

    resolution = 0.03

    ## Resets obstacle list for the current curve object
    curveas.obstacles = []

    ## Creates the Service Client that will get speed data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    new_data = rospy.ServiceProxy("collect_data", Empty)

    heading = des_heading - data_getter("heading").data
    curveas.heading = heading

    pub = rospy.Publisher('cycle/curve', Curve, queue_size=1)

    obj_lst = sorted(req.obj_lst, key=lambda o: (o.x1 + o.x2) / 2)
    # Creates objects
    for o in obj_lst:
        curveas.create_obstacle(((o.z1 / 1000, o.z2 / 1000), (o.x1 / 1000, o.x2 / 1000)))
        print(f"Object : ({o.z1}, {o.x1}, {o.z2}, {o.x2})")
    rospy.loginfo("Object data accepted. Generating Path")
    calculate_curve()

    # Plots obstacles if they intersect the Bezier curve
    ind = 1
    count = 0
    lim = 20
    for obstacle in curveas.obstacles:
        # Allows for computation of control points if and only if the curve intersects the object
        labels.append("Object " + str(ind))
        obstacle.next_side(x_vals, y_vals)
        ind += 1
    block_list = is_obstacle_block()
    while block_list:
        for obstacle in block_list:
            obstacle.adjust_err()
        calculate_curve()
        count += 1
        if count > lim:
            break
        block_list = is_obstacle_block()
    # plot()
    resolution = 0.001
    rospy.loginfo("Path generated.")
    end_time = time.time()
    deltas = get_deltas()
    rospy.loginfo(f"TIME TO PROCESS: {end_time - start_time}")
    tot_time += end_time - start_time
    pub.publish(deltas[0], deltas[1], curveas.get_curve().length, iden, end_time - start_time)
    iden += 1
    rospy.loginfo(f"FRAMES PER SECOND: {iden / tot_time}")
    data_getter.close()
    new_data()
    new_data.close()
    return


def start():
    global des_heading
    # Initialize the node and register it with the master.
    rospy.init_node("bezier")

    # Waits for data getter service
    rospy.wait_for_service('get_data')

    rospy.wait_for_service("collect_data")

    # Creates the service client that will collect data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    new_data = rospy.ServiceProxy("collect_data", Empty)

    ## Sets desired heading (for now the intial heading)
    des_heading = data_getter("heading").data

    rospy.Subscriber("cycle/object_frame", ObjectList, create_environment)

    ## Closes this data getter
    data_getter.close()

    new_data()

    rospy.spin()
