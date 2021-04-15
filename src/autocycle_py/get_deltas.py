import rospy
import re
from autocycle_extras.srv import GetData, Action, GetCurve
import time
import numpy as np

iden = -1
tck = []
dist_travelled = 0


def get_dervs(x, acc=0.000001):
    """Calculates the derivatives of the curve"""
    subs = lambda x: interp.splev(x, tck)
    this_val = subs(x)
    next_val = subs(x + acc)
    prev_val = subs(x - acc)
    return (((next_val[0] - 2 * this_val[0] + prev_val[0]) / (acc ** 2),
             (next_val[1] - 2 * this_val[1] + prev_val[1]) / (acc ** 2)),
            ((this_val[0] - next_val[0]) / acc,
             (this_val[1] - next_val[1]) / acc))


def get_delta(i):
    """Calculates steering angle"""
    global step
    ## Calls the data getter to give us the latest roll
    num = 1.02 * np.cos(0)
    step = 0.01  # decrease step size for greater precision
    cosdelt = np.cos(0.08)
    derv = get_dervs(i)
    calc1 = abs(derv[1][0] * derv[0][1] - derv[1][1] * derv[0][0])
    if calc1 != 0:
        calc2 = (cosdelt * (((derv[1][0] ** 2 + derv[1][1] ** 2) ** (3 / 2)) / calc1))
        if calc2 != 0:
            return num / calc2


def update_distance(time):
    global dist_travelled, length

    ## Creates the Service Client that will get speed data
    data_getter = rospy.ServiceProxy("get_data", GetData)

    ## Updates the dist_travelled
    dist_travelled += (data_getter("vel").data*time)

    ## Closes the getter
    data_getter.close()


def start():
    global dist_travelled, iden, tck
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Waits for the data getter to be done setting up
    rospy.wait_for_service('get_data')

    # Waits for the send_action
    rospy.wait_for_service("send_action")
    
    # Waits for curve getter service
    rospy.wait_for_service("get_curve")

    # Creates the Service client that will send actions to the bike!
    action_sender = rospy.ServiceProxy("send_action", Action)
    
    # Creates the service cline that will get the curve
    curve_getter = rospy.ServiceProxy("get_curve", GetCurve)

    while iden == -1:
        result = curve_getter(iden)
        iden = result.iden
        tck = [result.t, result.c, result.k]


    # Initial Time
    time_i = time.time()


    # Continues sending commands to bike and updating deltas
    while not rospy.is_shutdown():
        result = curve_getter(iden)
        if result.iden != -1:
            iden = result.iden
            dist_travelled = 0
            tck = [result.t, result.c, result.k]
            time_i = time.time()
        
        # Time to adjust for
        time_f = time.time()

        # Adjusts for distance travelled
        update_distance(time_f-time_i)

        # Updates initial time
        time_i = time_f
        
        # Gets closest x to the distance travelled
        delta = get_delta(dist_travelled)

        rospy.loginfo(f"Delta Sent: {delta}")

        # Sends commands to the `action` node
        action_sender([True, True, False], delta, 4.5, "")
        



