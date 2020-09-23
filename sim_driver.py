import bezier_visualization
import numpy as np
import sys

"""This will simulate a vehicle following a calculated bezier curve"""


def write_new_data(dist_to_edge, edge_to_path, edge_len, resolution):
    for frame in range(len(dist_to_edge)):
        if not (
            len(dist_to_edge[frame]) == len(edge_to_path[frame]) == len(edge_len[frame])
        ):
            print("The number of items in all arrays must be the same!")
            exit()

    try:
        dataFile = open("TestData", "w+")
    except FileNotFoundError:
        print("The file could not be found, check the name.")
        exit()
    except:
        print(
            "It is likely that the file is open somewhere else. Close it and try again."
        )
        exit()

    dataFile.write(str(resolution) + "\n")
    for frame in range(len(edge_len)):
        dataFile.write(str(len(edge_len[frame])) + " ")
        for datum in range(len(edge_len[frame])):
            dataFile.write(
                str(dist_to_edge[frame][datum])
                + " "
                + str(edge_to_path[frame][datum])
                + " "
                + str(edge_len[frame][datum])
                + " "
            )
        dataFile.write("\n")


# The angle to the globally determined point
glob_angle = 0.00

# The furthest distance from origin that the path is rendered to
resolution = 10

# Distances from the bike to the objects
dist_to_edge = [[4, 6]]

# The distance from the current path to edge of the obstacle most counter clockwise from the current path
edge_to_path = [[1, 4]]

# Lengths of the objects
edge_len = [[6, 4]]

# x_dist from original point
x_dist = 0

stdin = sys.stdin

while x_dist < 10:
    # writes the next frame


    write_new_data(dist_to_edge, edge_to_path, edge_len, resolution)

    print("Glob_ANGLE:", glob_angle)

    # runs the visualization and gets back the data in the form of a CurveAssistant object
    curveas = bezier_visualization.run_visualization(glob_angle)

    # gets the Curve object from the CurveAssistant
    curve = curveas.get_curve()

    # evaluates the x and y coordinates of the next simulated position of the bike
    next_point = curve.evaluate(0.01)

    # angle from the x-axis to the parallel line extending from the new position
    par_angle = np.rad2deg(
        np.arctan(next_point[1] / (curveas.end_dist - next_point[0]))
    )[0]

    # evaluates the derivative of the bezier curve at the new position
    print("Info For Next Frame:\n______________________________________")
    print("x-point: ", next_point[0])
    hodo = curve.evaluate_hodograph(0.01)
    split_diriv = curveas.convert_nt(hodo[0][0], hodo[1][0])
    diriv = split_diriv[1]/split_diriv[0]
    print("DIRIV: ", diriv)

    # angle from the above line to the tangent line at the new position
    tan_angle = np.rad2deg(np.arctan(1/diriv))
    print("TAN: " , tan_angle)

    # the total angle to the final goal position
    glob_angle = (par_angle + tan_angle) * -1.00

    # moves the objects closer to the bike to simulate their movement
    for ind in range(len(dist_to_edge)):
        dist_to_edge[ind] = dist_to_edge[ind] - next_point[0]

    # moves the objects further or closer to the current path to simulate their relative movement
    for ind in range(len(edge_to_path)):
        edge_to_path[ind] = edge_to_path[ind] - next_point[1]

    # adds the distance travelled by the bike since the last frame
    x_dist += next_point[0]

    print("______________________________________")
    cur_stdin = sys.stdin
    sys.stdin = stdin
    next = ""
    while next != "n":
        next = input("To print next frame type 'n' then 'enter'")
    sys.stdin = cur_stdin
    print("______________________________________")
