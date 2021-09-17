import rospy
from autocycle_extras.srv import GetDelta, Path
from autocycle_extras.msg import CalcDeltas
import numpy as np
from scipy import interpolate as interp
import time

tck = []
full_len = 0

def get_dervs(x, acc=0.000001):
    """Calculates the derivatives of the curve"""
    if not full_len:
        return 0
    u = x/full_len
    subs = lambda x: interp.splev(x, tck)
    this_val = subs(u)
    next_val = subs(u + acc)
    prev_val = subs(u - acc)
    return (((next_val[0] - 2 * this_val[0] + prev_val[0]) / (acc ** 2),
             (next_val[1] - 2 * this_val[1] + prev_val[1]) / (acc ** 2)),
            ((this_val[0] - next_val[0]) / acc,
             (this_val[1] - next_val[1]) / acc))


def get_delta(i, vel):
    """Calculates steering angle"""
    global step
    ## Calls the data getter to give us the latest roll
    dervs = get_dervs(i)

    wheel_base = 1
    radius_of_turn = ((dervs[1][0] ** 2 + dervs[1][1] ** 2) ** (3 / 2)) / (
            (dervs[1][0] * dervs[0][1]) - (dervs[1][1] * dervs[0][0]))
    g = 32.2
    load_front_ax = 1
    load_rear_ax = 1
    corn_stiff_front = 1
    corn_stiff_rear = 1

    return wheel_base/radius_of_turn + ((load_front_ax/corn_stiff_front)-(load_rear_ax/corn_stiff_rear))*((vel**2)/(g*radius_of_turn))

def calculate_deltas(data):
    global tck, full_len
    xs = list(data.path_x)
    ys = list(data.path_y)
    xs = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0]
    ys = [0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 1.0, 0.0]
    xs.insert(0, 0)
    ys.insert(0, 0)
    tck = interp.splprep([xs, ys], s=0.5)[0]
    full_len = (sum(interp.splint(0, 1, tck, full_output=0))) * 2
    return []


def delta(data):
    return get_delta(data.x, data.vel) if tck else -1

def start():
    global dist_travelled, iden, tck
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Service that will update the parameters for calculating deltas
    calc_delta = rospy.Service('calc_delta', Path, calculate_deltas)

    # Service that will return the appropriate delta for a given x-value and roll
    g_delta = rospy.Service("get_delta", GetDelta, delta)
        
    rospy.spin()


