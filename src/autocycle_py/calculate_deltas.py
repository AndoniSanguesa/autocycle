import rospy
from autocycle_extras.msg import CalcDeltas
from autocycle_extras.srv import TCK
from scipy import interpolate as interp



tck = []
full_len = 0
update_delta_proxy = None


def calculate_deltas(data):
    global tck, full_len
    xs = list(data.path_x)
    ys = list(data.path_y)
    xs.insert(0, 0)
    ys.insert(0, 0)
    tck = interp.splprep([xs, ys], s=0.5)[0]
    full_len = (sum(interp.splint(0, 1, tck, full_output=0))) * 2
    t = tck[0].tolist()
    c = tck[1].tolist()

    update_delta_proxy(t, c, tck[2].item(), full_len)


def dummy_fun(data):
    return []


def start():
    global update_delta_proxy

    # Initializes the ROS node
    rospy.init_node("calculate_deltas")

    # Waits for the service that allows for deltas to be updated to start
    rospy.wait_for_service("update_deltas")

    # Creates Subscriber for new paths to generate deltas for
    rospy.Subscriber("cycle/calc_deltas", CalcDeltas, calculate_deltas)

    # Creates Dummy Service to confirm subscriber is ready to go
    dummy = rospy.Service("calc_deltas", TCK, dummy_fun)

    # Creates service proxy for delta updating
    update_delta_proxy = rospy.ServiceProxy("update_deltas", TCK)

    # ROS waits for callbacks
    rospy.spin()

