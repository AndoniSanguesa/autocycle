import googlemaps

# import rospy
# from autocycle_extras.msg import GPS

gmaps = googlemaps.Client(key="")
position_destination = [0, 0]


def update_gps(data):
    global position_destination

    position_destination = [data.lat, data.long]


def get_gps_data():
    # Initializes the ROS node
    # rospy.init_node("calculate_deltas")

    # Creates Subscriber for new paths to generate deltas for
    # rospy.Subscriber("cycle/gps", GPS, update_gps)

    return
    # will likely only need a continuous stream of geo-coordinates


def get_directions():
    directions = gmaps.directions(position_destination[0], position_destination[1])
    print(directions)


def test():
    position_destination[0] = (38.993176, -76.933367)  # Cypress
    position_destination[1] = (38.991369, -76.947012)  # Ellicott Hall
    get_directions()


if __name__ == "__main__":
    test()