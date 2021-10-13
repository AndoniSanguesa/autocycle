import googlemaps
# import rospy
# from autocycle_extras.msg import GPS

gmaps = googlemaps.Client(key="")
cur_gps_coords = (0, 0)


def update_gps(data):
    global cur_gps_coords

    cur_gps_coords = (data.long, data.lat)


def get_gps_data():
    # Initializes the ROS node
    # rospy.init_node("calculate_deltas")

    # Creates Subscriber for new paths to generate deltas for
    # rospy.Subscriber("cycle/gps", GPS, update_gps)

    return
    # will likely only need a continuous stream of geo-coordinates
