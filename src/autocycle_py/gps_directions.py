import googlemaps
import numpy
import polyline
from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np

# import rospy
# from autocycle_extras.msg import GPS
# from autocycle_extras.srv import DesiredGPS

# Put API key here. Do not push to a public repo
GOOGLE_API_KEY = ""

gmaps = googlemaps.Client(key=GOOGLE_API_KEY)
position_destination = [0, 0]


def update_gps(data):
    global position_destination

    position_destination = [data.latitude, data.longitude]


def get_desired_gps(data):
    # Each time this callback is called, the next desired longitude and latitude are returned
    return 0, 0


def get_gps_data():
    # Initializes the ROS node
    # rospy.init_node("calculate_deltas")

    # Creates Subscriber for new paths to generate deltas for
    # rospy.Subscriber("cycle/gps", GPS, update_gps)

    # Creates service that will provide the next desired GPS position
    # des_gps = rospy.Service("get_desired_gps", DesiredGPS, get_desired_gps)

    return
    # will likely only need a continuous stream of geo-coordinates


def get_directions():
    directions = gmaps.directions(position_destination[0], position_destination[1])
    return directions[0]


def smoothed_path():
    overview_polyline = get_directions()['overview_polyline']['points']
    return polyline.decode(overview_polyline)


def test_location():
    position_destination[0] = (38.993176, -76.933367)  # Cypress
    position_destination[1] = (38.991369, -76.947012)  # Ellicott Hall


def test():
    steps_dict = get_directions()['legs'][0]['steps']
    overview_path = smoothed_path()
    for x in steps_dict:
        print((round(x['start_location']['lat'], 5), round(x['start_location']['lng'], 5)))
        print(str((round(x['start_location']['lat'], 5),
                   round(x['start_location']['lng'], 5)) in overview_path) + " start")
        print(str((round(x['end_location']['lat'], 5), round(x['end_location']['lng'], 5)) in overview_path) + " end")
    print(overview_path)


def interpolated_path(path, k, curr_position, t, c):
    h = [2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2]
    p = [curr_position, path[k + 1]]
    m = [tuple(x * (1 - c) / (path[k + 1][0] - path[k][0]) for x in tuple(numpy.subtract(path[k + 1], path[k]))), tuple(
        x * (1 - c) / (path[k + 2][0] - curr_position[0]) for x in tuple(numpy.subtract(path[k + 2], curr_position)))]
    left = tuple(numpy.add(tuple(x * h[0] for x in p[0]), tuple(x * h[1] for x in m[0])))
    right = tuple(numpy.add(tuple(x * h[2] for x in p[1]), tuple(x * h[3] for x in m[1])))
    return tuple(numpy.add(left, right))


if __name__ == "__main__":
    test_location()
    test_interpolation()
