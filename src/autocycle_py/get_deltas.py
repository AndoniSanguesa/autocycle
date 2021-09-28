import rospy
from autocycle_extras.srv import GetDelta, TCK
from scipy import interpolate as interp

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


def delta(data):
    return get_delta(data.x, data.vel) if tck else -1


def update_deltas(data):
    global tck, full_len
    tck = (data.t, [data.c1, data.c2], data.k)
    full_len = data.full_len
    return []


def start():
    # Registers Node with the master
    rospy.init_node('get_deltas')

    # Service that will return the appropriate delta for a given x-value and roll
    fetch_delta = rospy.Service("get_delta", GetDelta, delta)

    # Service that will update the tck and full_len values
    new_deltas = rospy.Service("update_deltas", TCK, update_deltas)
        
    rospy.spin()


