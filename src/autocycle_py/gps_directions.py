import googlemaps
import serial

# import rospy
# from autocycle_extras.msg import GPS, DesiredGPS

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
    #des_gps = rospy.Service("get_desired_gps", DesiredGPS, get_desired_gps)

    return
    # will likely only need a continuous stream of geo-coordinates


def get_directions():
    directions = gmaps.directions(position_destination[0], position_destination[1])
    print(directions)


if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            position_destination = line.rsplit(",")
            get_directions()
