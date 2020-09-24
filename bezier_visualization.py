import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from Assistant import CurveAssistant


class Visualization:
    def __init__(self, lines):
        # The obstacle data in the format of a list of string lines (parsed from datafile)
        self.lines = lines

        # Makes the graphs look not dumb. Also created array for the graph labels
        self.fontP = FontProperties()
        self.fontP.set_size("small")
        self.labels = []

        # Distance to the end of the graph (our max viewing distance)
        self.end_dist = int(self.lines[0])

        # Distances from the bike to the objects
        self.dist_to_edge = []

        # Shortest distance from the edges of the objects to the global path
        self.edge_to_path = []

        # Lengths of the objects
        self.edge_len = []

        # The final x and y values of the Bezier plot
        self.x_vals = []
        self.y_vals = []

        # Creates a CurveAssistant that will allow us to access our data
        self.curveas = CurveAssistant(self.end_dist)

        # The coordinate of the final control point at the end of the graph
        self.lastpoint = self.curveas.get_last_control_point()

        # This is the  `quality of the graph`. Denotes the distance between sampled points (should be re-assigned later)
        self.resolution = 0

    def get_data(self):
        """
        Retrieves obstacle data from file `TestData` and stores it in global variables

        :return:
        """
        data = self.lines[1].split()
        for x in range(int(data[0])):
            self.dist_to_edge.append(float(data[x * 3 + 1]))
            self.edge_to_path.append(float(data[x * 3 + 2]))
            self.edge_len.append(float(data[x * 3 + 3]))

    def plot(self):
        """
        Takes data resulting from `create_environment()` (for curve and obstacles) and plots it.

        :return:
        """
        # Plots the obstacles
        for obstacle in self.curveas.obstacles:
            # Converts the x-y points into the nt points
            points = obstacle.edge_points
            nt_1 = self.curveas.convert_nt(points[0][0], points[1][0])
            nt_2 = self.curveas.convert_nt(points[0][1], points[1][1])
            nt_obst = [[nt_1[0], nt_2[0]], [nt_1[1], nt_2[1]]]
            plt.plot(nt_obst[0], nt_obst[1], linewidth=4)

        # Plots the Current Path
        nt_last_point = self.curveas.convert_nt(self.lastpoint[0], self.lastpoint[1])
        plt.plot(
            [0, nt_last_point[0]], [0, nt_last_point[1]], color="green", linestyle="dashed"
        )

        # Plots and displays bezier curve
        nt_bez_list = [[], []]
        for index in range(len(self.x_vals)):
            # Converts xy points to nt points for the bezier curve
            nt_bez_cur = self.curveas.convert_nt(self.x_vals[index], self.y_vals[index])
            nt_bez_list[0].append(nt_bez_cur[0])
            nt_bez_list[1].append(nt_bez_cur[1])

        # Plots Bezier Curve
        plt.plot(nt_bez_list[0], nt_bez_list[1])

        # Plots Control points after converting from xy to nt
        ctr_points = self.curveas.get_control_points()
        ind = 0
        for x in range(len(ctr_points)):
            nt_ctr = self.curveas.convert_nt(ctr_points[x][0], ctr_points[x][1])
            plt.plot(nt_ctr[0], nt_ctr[1], "ro")
            ind += 1
        self.labels.extend(["Current Path", "Global Path", "Predicted Path", "Control Point"])
        plt.legend(self.labels, prop=self.fontP)

        # Sets the plot axis to not be dumb
        plt.axis("square")
        plt.axis([0, nt_last_point[0], 0, nt_last_point[1]])
        plt.show()
        plt.clf()

    def calculate_curve(self):
        """
        Calculates x and y points for a bezier curve

        :return:
        """
        # Empties the x and y values for the next curve
        self.x_vals.clear()
        self.y_vals.clear()

        # This command creates a curve object which allows us to check the characteristics of the curve
        curve = self.curveas.get_curve()

        # Variables keep track of x/y values and help in determining extrema
        curpoint = [[0]]
        index = 0
        diff = [0, 0]
        ys = [0, 0]

        while curpoint[0][0] < self.end_dist:
            curpoint = curve.evaluate(index * 1.00)
            x = curpoint[0][0]
            y = curpoint[1][0]

            # To determine where the extrema occurs on the bezier curve
            ys[0] = ys[1]
            ys[1] = y
            diff[0] = diff[1]
            diff[1] = ys[1] - ys[0]
            if diff[1] * diff[0] < 0:
                self.curveas.extrema = [x, y]

            # Adds x and y values to their respective array
            self.x_vals.append(curpoint[0][0])
            self.y_vals.append(curpoint[1][0])
            index += self.resolution

    def create_environment(self):
        """
        Creates obstacles and heuristically produces curve by continuous calls to `calculate_curve()`

        :return:
        """
        # Temporarily reassigns the global variable `resolution` to get a rough sketch of the graph
        self.resolution = 0.4
        self.get_data()
        self.curveas.clear_obstacles()
        # Creates objects
        for x in range(len(self.edge_len)):
            self.curveas.create_obstacle(self.dist_to_edge[x], self.edge_to_path[x], self.edge_len[x])
        self.calculate_curve()
        # Plots obstacles if they intersect the Bezier curve
        ind = 1
        for obstacle in self.curveas.obstacles:
            # Allows for computation of control points if and only if the curve intersects the object
            self.labels.append("Object " + str(ind))
            obstacle.next_side(self.curveas.extrema[0], self.curveas.extrema[1])
            while obstacle.intersect(self.x_vals, self.y_vals):
                obstacle.adjust_gamma()
                self.calculate_curve()
            ind += 1
        self.resolution = 0.001
        self.calculate_curve()
        self.plot()
