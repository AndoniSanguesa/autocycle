import rospy
from autocycle_extras.srv import GetDelta
from autocycle_extras.msg import CalcDeltas
import numpy as np
from scipy import interpolate as interp
import time

tck = []

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


def get_delta(i, roll):
    """Calculates steering angle"""
    global step
    ## Calls the data getter to give us the latest roll
    num = 1.02 * np.cos(roll)
    step = 0.01  # decrease step size for greater precision
    cosdelt = np.cos(0.08)
    derv = get_dervs(i)
    calc1 = abs(derv[1][0] * derv[0][1] - derv[1][1] * derv[0][0])
    if calc1 != 0:
        calc2 = (cosdelt * (((derv[1][0] ** 2 + derv[1][1] ** 2) ** (3 / 2)) / calc1))
        if calc2 != 0:
            return num / calc2


def calculate_deltas(data):
    global tck
    xs = list(data.path_x)
    ys = list(data.path_y)
    xs.insert(0, 0)
    ys.insert(0, 0)
    tck = interp.splprep([xs, ys], s=0.5)[0]
    return []


def delta(data):
    print(f"THIS IS THE TCK VALUE: {tck}")
    return get_delta(data.x, data.roll) if tck else -1


def start():
    global dist_travelled, iden, tck
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Service that will update the parameters for calculating deltas
    calc_delta = rospy.Subscriber("cycle/path", CalcDeltas, calculate_deltas)

    # Service that will return the appropriate delta for a given x-value and roll
    g_delta = rospy.Service("get_delta", GetDelta, delta)
        
    rospy.spin()


