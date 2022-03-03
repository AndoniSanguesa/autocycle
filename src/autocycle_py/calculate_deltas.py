import rospy
from autocycle_extras.msg import CalcDeltas
from autocycle_extras.srv import TCK
from autocycle_extras.srv import SRVCalcDeltas
from std_msgs.msg import Empty
from scipy import interpolate as interp
import time


tck = []
full_len = 0
update_delta_proxy = None
ready_pub = None

def calculate_deltas(data):
    global tck, full_len
    #print("ORIG XS and YS")
    print("HELLO HERE")

    xs = list(data.path_x)
    ys = list(data.path_y)
    #print(xs, ys)
    new_xs = []
    new_ys = []
    for i in range(len(xs)-1):
        if xs[i] != xs[i+1] or ys[i] != ys[i+1]:
            new_xs.append(xs[i])
            new_ys.append(ys[i])
    xs = new_xs
    ys = new_ys
    #print(xs, ys)
    tck = interp.splprep([xs, ys], s=0.5)[0]
    full_len = (sum(interp.splint(0, 1, tck, full_output=0))) * 2
    print("FULL LENGTH", full_len)
    t = tck[0].tolist()
    c1 = tck[1][0].tolist()
    c2 = tck[1][1].tolist()
    #print("XS AND YS")
    #print(xs, ys)
    #print("TCK")
    #print(tck)
    #print("Full Length:")
    #print(full_len)
    update_delta_proxy(t, c1, c2, tck[2], full_len)
    return []



def dummy_fun(data):
    return []


def start():
    global update_delta_proxy, ready_pub

    # Initializes the ROS node
    rospy.init_node("calculate_deltas")

    # Waits for the service that allows for deltas to be updated to start
    rospy.wait_for_service("update_deltas")

    rospy.Service("calc_deltas", SRVCalcDeltas, calculate_deltas)

    # Creates Subscriber for new paths to generate deltas for
    rospy.Subscriber("cycle/calc_deltas", CalcDeltas, calculate_deltas)

    # Creates service proxy for delta updating
    update_delta_proxy = rospy.ServiceProxy("update_deltas", TCK)
    
    # ROS waits for callbacks
    rospy.spin()

